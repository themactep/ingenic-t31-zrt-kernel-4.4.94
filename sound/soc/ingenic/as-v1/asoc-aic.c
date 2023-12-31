/*
 *  sound/soc/ingenic/asoc-aic.c
 *  ALSA Soc Audio Layer -- ingenic aic device driver
 *
 *  Copyright 2014 Ingenic Semiconductor Co.,Ltd
 *	cli <chen.li@ingenic.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <sound/dmaengine_pcm.h>
#include "asoc-aic.h"

#define INGENIC_I2S_RATE (24*1000000)
static const char *aic_no_mode = "no mode";
static const char *aic_i2s_mode = "i2s mode";
static const char *aic_spdif_mode = "spdif mode";
static const char *aic_ac97_mode = "ac97 mode";

const char* aic_work_mode_str(enum aic_mode mode)
{
	switch (mode) {
	default:
	case AIC_NO_MODE:
		return aic_no_mode;
	case AIC_I2S_MODE:
		return aic_i2s_mode;
	case AIC_SPDIF_MODE:
		return aic_spdif_mode;
	case AIC_AC97_MODE:
		return aic_ac97_mode;
	}
}
EXPORT_SYMBOL_GPL(aic_work_mode_str);

enum aic_mode aic_set_work_mode(struct device *aic,
		enum aic_mode module_mode, bool enable)
{
	struct ingenic_aic *ingenic_aic = dev_get_drvdata(aic);
	enum aic_mode working_mode;

	spin_lock(&ingenic_aic->mode_lock);
	if  (module_mode != AIC_AC97_MODE &&
			module_mode != AIC_I2S_MODE &&
			module_mode != AIC_SPDIF_MODE)
		goto out;

	if (enable && ingenic_aic->aic_working_mode == AIC_NO_MODE) {
		ingenic_aic->aic_working_mode = module_mode;
	} else if (!enable && ingenic_aic->aic_working_mode == module_mode) {
		ingenic_aic->aic_working_mode = AIC_NO_MODE;
	}
out:
	working_mode = ingenic_aic->aic_working_mode;
	spin_unlock(&ingenic_aic->mode_lock);
	return working_mode;
}
EXPORT_SYMBOL_GPL(aic_set_work_mode);

int aic_set_rate(struct device *aic, unsigned long freq)
{
	struct ingenic_aic *ingenic_aic = dev_get_drvdata(aic);
	int ret;
	if (ingenic_aic->clk_rate != freq) {
		ret = clk_set_rate(ingenic_aic->clk, freq);
		if (ret == -EBUSY) {
			clk_disable_unprepare(ingenic_aic->clk);
			ret = clk_set_rate(ingenic_aic->clk, freq);
			clk_prepare_enable(ingenic_aic->clk);
		}
		ingenic_aic->clk_rate = clk_get_rate(ingenic_aic->clk);
	}
	return ingenic_aic->clk_rate;
}
EXPORT_SYMBOL_GPL(aic_set_rate);

static irqreturn_t ingenic_aic_irq_thread(int irq, void *dev_id)
{
	struct ingenic_aic *ingenic_aic = (struct ingenic_aic *)dev_id;

	if ((ingenic_aic->mask & 0x8) && __aic_test_ror(ingenic_aic->dev)) {
		ingenic_aic->ror++;
		dev_printk(KERN_DEBUG, ingenic_aic->dev,
				"recieve fifo [overrun] interrupt time [%d]\n",
				ingenic_aic->ror);
	}

	if ((ingenic_aic->mask & 0x4) && __aic_test_tur(ingenic_aic->dev)) {
		ingenic_aic->tur++;
		dev_printk(KERN_DEBUG, ingenic_aic->dev,
				"transmit fifo [underrun] interrupt time [%d]\n",
				ingenic_aic->tur);
	}

	if ((ingenic_aic->mask & 0x2) && __aic_test_rfs(ingenic_aic->dev)) {
		dev_printk(KERN_DEBUG, ingenic_aic->dev,
				"[recieve] fifo at or above threshold interrupt time\n");
	}

	if ((ingenic_aic->mask & 0x1) && __aic_test_tfs(ingenic_aic->dev)) {
		dev_printk(KERN_DEBUG, ingenic_aic->dev,
				"[transmit] fifo at or blow threshold interrupt time\n");
	}

	/*sleep, avoid frequently interrupt*/
	msleep(200);
	__aic_clear_all_irq_flag(ingenic_aic->dev);
	__aic_set_irq_enmask(ingenic_aic->dev, ingenic_aic->mask);
	return IRQ_HANDLED;
}

static irqreturn_t ingenic_aic_irq_handler(int irq, void *dev_id)
{
	struct ingenic_aic *ingenic_aic = (struct ingenic_aic *)dev_id;

	ingenic_aic->mask = __aic_get_irq_enmask(ingenic_aic->dev);
	if (ingenic_aic->mask && (ingenic_aic->mask & __aic_get_irq_flag(ingenic_aic->dev))) {
		/*Disable all aic interrupt*/
		__aic_set_irq_enmask(ingenic_aic->dev, 0);
		return IRQ_WAKE_THREAD;
	}
	return IRQ_NONE;
}

extern int ingenic_dma_pcm_register(struct device *dev,  const struct snd_dmaengine_pcm_config *config);
extern void ingenic_dma_pcm_unregister(struct device *dev);

static int ingenic_aic_probe(struct platform_device *pdev)
{
	struct ingenic_aic *ingenic_aic;
	struct resource *res = NULL;
	struct device_node *subdev_node = NULL;
	int ret, nr_child, i = 0;

	nr_child = of_get_child_count(pdev->dev.of_node);

	ingenic_aic = devm_kzalloc(&pdev->dev, sizeof(struct ingenic_aic) + nr_child * sizeof(void *),
			GFP_KERNEL);
	if (!ingenic_aic)
		return -ENOMEM;

	ingenic_aic->dev = &pdev->dev;
	ingenic_aic->subdevs = nr_child;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOENT;

	ingenic_aic->vaddr_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ingenic_aic->vaddr_base))
		return -ENOMEM;

	ingenic_aic->clk_gate = devm_clk_get(&pdev->dev, "gate_aic");
	if (IS_ERR(ingenic_aic->clk_gate)) {
		dev_err(&pdev->dev, "gate_aic get clk fail\n");
		return PTR_ERR(ingenic_aic->clk_gate);
	}

	ingenic_aic->clk = devm_clk_get(&pdev->dev, "mux_i2st");
	if (IS_ERR(ingenic_aic->clk)) {
		dev_err(&pdev->dev, "mux_i2st get clk fail\n");
		return PTR_ERR(ingenic_aic->clk);
	}
	clk_set_rate(ingenic_aic->clk, INGENIC_I2S_RATE);

	spin_lock_init(&ingenic_aic->mode_lock);

	ingenic_aic->irqno = platform_get_irq(pdev, 0);
	if (ingenic_aic->irqno >= 0)
		ret = devm_request_threaded_irq(&pdev->dev, ingenic_aic->irqno,
				ingenic_aic_irq_handler, ingenic_aic_irq_thread,
				IRQF_SHARED , pdev->name, (void *)ingenic_aic);
	if (!ret)
		dev_info(&pdev->dev, "register aic irq, ret %d, irqno is %d\n", ret, ingenic_aic->irqno);

	platform_set_drvdata(pdev, (void *)ingenic_aic);

	clk_prepare_enable(ingenic_aic->clk);
	clk_prepare_enable(ingenic_aic->clk_gate);

	for_each_child_of_node(pdev->dev.of_node, subdev_node)
		ingenic_aic->psubdev[i++] = of_platform_device_create(subdev_node, NULL, &pdev->dev);

	ret = ingenic_dma_pcm_register(&pdev->dev, NULL);
	if (ret) {
		dev_err(&pdev->dev, "ingenic_dma_pcm_register fail %d\n", ret);
		int i;
		for (i = 0; i < ingenic_aic->subdevs; i++)
			platform_device_unregister(ingenic_aic->psubdev[i]);
		clk_disable_unprepare(ingenic_aic->clk_gate);
		clk_disable_unprepare(ingenic_aic->clk);
		return ret;
	}

	dev_info(&pdev->dev, "Aic core probe success\n");
	return 0;
}

static int ingenic_aic_remove(struct platform_device *pdev)
{
	struct ingenic_aic * ingenic_aic = platform_get_drvdata(pdev);
	int i;

	ingenic_dma_pcm_unregister(&pdev->dev);

	if (!ingenic_aic)
		return 0;

	for (i = 0; i < ingenic_aic->subdevs; i++)
		platform_device_unregister(ingenic_aic->psubdev[i]);

	platform_set_drvdata(pdev, NULL);

	clk_disable_unprepare(ingenic_aic->clk_gate);
	clk_disable_unprepare(ingenic_aic->clk);
	return 0;
}

#ifdef CONFIG_PM
int ingenic_aic_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ingenic_aic * ingenic_aic = platform_get_drvdata(pdev);
	clk_disable(ingenic_aic->clk_gate);
	return 0;
}

int ingenic_aic_resume(struct platform_device *pdev)
{
	struct ingenic_aic * ingenic_aic = platform_get_drvdata(pdev);
	clk_enable(ingenic_aic->clk_gate);
	return 0;
}
#endif

static const struct of_device_id aic_dt_match[] = {
	{ .compatible = "ingenic,aic", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, aic_dt_match);

static struct platform_driver ingenic_asoc_aic_driver = {
	.driver = {
		.name   = "asoc-aic",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(aic_dt_match),
	},
	.probe  = ingenic_aic_probe,
	.remove = ingenic_aic_remove,
#ifdef CONFIG_PM
	.suspend = ingenic_aic_suspend,
	.resume = ingenic_aic_resume,
#endif
};
module_platform_driver(ingenic_asoc_aic_driver);

MODULE_DESCRIPTION("INGENIC ASOC AIC core driver");
MODULE_AUTHOR("cli<chen.li@ingenic.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-asoc-aic");
