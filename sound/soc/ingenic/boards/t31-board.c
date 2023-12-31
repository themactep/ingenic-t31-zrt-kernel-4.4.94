/*
 * Copyright (C) 2014 Ingenic Semiconductor Co., Ltd.
 *	http://www.ingenic.com
 * Author: cli <chen.li@ingenic.com>
 * Modified by: Santiago H. <santiagohssl@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "../as-v1/asoc-aic.h"

struct t31_icdc {
	struct snd_soc_card card;
	int spk_gpio;
	int spk_en_level;
};

int t31_i2s_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params) {
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S|SND_SOC_DAIFMT_CBM_CFM);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, INGENIC_I2S_INNER_CODEC,
								 24000000, SND_SOC_CLOCK_OUT);
	if (ret)
		return ret;
	return 0;
};

int t31_i2s_hw_free(struct snd_pcm_substream *substream)
{
	/*notify release pll*/
	return 0;
};

static struct snd_soc_ops t31_i2s_cdc_ops = {
	.hw_params = t31_i2s_hw_params,
	.hw_free = t31_i2s_hw_free,
};

static int t31_spk_power(struct snd_soc_dapm_widget *w, struct snd_kcontrol *kcontrol, int event)
{
	struct t31_icdc *t31 = container_of(w->dapm->card, struct t31_icdc, card);

	if (!gpio_is_valid(t31->spk_gpio))
		return 0;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		gpio_direction_output(t31->spk_gpio, t31->spk_en_level);
		printk("gpio speaker enable %d\n", gpio_get_value(t31->spk_gpio));
	} else {
		gpio_direction_output(t31->spk_gpio, t31->spk_en_level);
		printk("gpio speaker disable %d\n", gpio_get_value(t31->spk_gpio));
	}
	return 0;
}

static const struct snd_soc_dapm_widget t31_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", t31_spk_power),
	SND_SOC_DAPM_MIC("Mic Buildin", NULL),
	SND_SOC_DAPM_MIC("DMic", NULL),
};

static int t31_i2s_cdc_dai_link_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dapm_context *dapm = &card->dapm;

	snd_soc_dapm_enable_pin(dapm, "Speaker");
	snd_soc_dapm_enable_pin(dapm, "Mic Buildin");
	snd_soc_dapm_enable_pin(dapm, "DMic");
	return 0;
}

static int snd_t31_probe(struct platform_device *pdev)
{
	struct device_node *snd_node = pdev->dev.of_node;
	struct t31_icdc *t31;
	struct snd_soc_card *card;
	struct snd_soc_dai_link *dai_link;
	enum of_gpio_flags flags;
	int num_links;
	int ret = 0, i;

	num_links = of_property_count_strings(snd_node, "ingenic,dai-link");
	if (num_links < 0)
		return num_links;
	BUG_ON(!num_links);

	t31 = (struct t31_icdc *)devm_kzalloc(&pdev->dev,
			sizeof(struct t31_icdc) +
			sizeof(struct snd_soc_dai_link) * num_links,
			GFP_KERNEL);
	if (!t31)
		return -ENOMEM;
	card = &t31->card;
	dai_link = (struct snd_soc_dai_link *)(t31 + 1);

	card->num_dapm_widgets = ARRAY_SIZE(t31_dapm_widgets);
	card->dapm_widgets = t31_dapm_widgets;
	card->num_links = num_links;
	card->dai_link = dai_link;
	card->owner = THIS_MODULE;
	card->dev = &pdev->dev;

	ret = snd_soc_of_parse_card_name(card, "ingenic,model");
	if (ret)
		return ret;

	ret = snd_soc_of_parse_audio_routing(card, "ingenic,audio-routing");
	if (ret) {
		dev_err(&pdev->dev, "audio-routing invalid\n");
		return ret;
	}

	t31->spk_gpio = of_get_named_gpio_flags(card->dev->of_node, "ingenic,spken-gpio", 0, &flags);
	if (gpio_is_valid(t31->spk_gpio)) {
		unsigned long init_flags;
		t31->spk_en_level = (flags == OF_GPIO_ACTIVE_LOW ? 0 : 1);
		init_flags = (flags == OF_GPIO_ACTIVE_LOW ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW);
		ret = devm_gpio_request_one(card->dev, t31->spk_gpio, init_flags, "Speaker_en");
		if (ret)
			pr_warn("dorado speaker enable pin(%d) request failed\n",t31->spk_gpio);
		else
			pr_info("dorado speaker enable pin(%d) request ok\n", t31->spk_gpio);
	}

	for (i = 0; i < card->num_links; i++) {
		dai_link[i].cpu_of_node = of_parse_phandle(snd_node, "ingenic,cpu-dai" , i);
		dai_link[i].platform_of_node = of_parse_phandle(snd_node, "ingenic,platform", i);
		dai_link[i].codec_of_node = of_parse_phandle(snd_node, "ingenic,codec", i);
		ret = of_property_read_string_index(snd_node, "ingenic,codec-dai", i,
				&(dai_link[i].codec_dai_name));
		if (ret || !dai_link[i].cpu_of_node ||
				!dai_link[i].codec_of_node ||
				!dai_link[i].platform_of_node)
			return -ENODEV;
		ret = of_property_read_string_index(snd_node, "ingenic,dai-link", i,
				&(dai_link[i].name));
		if (ret)
			return -ENODEV;
		ret = of_property_read_string_index(snd_node, "ingenic,stream", i,
				&(dai_link[i].stream_name));
		if (ret)
			dai_link[i].stream_name = dai_link[i].name;

		dev_info(&pdev->dev, "dai_link %s\n", dai_link[i].name);
		dev_info(&pdev->dev, "stream_name %s\n", dai_link[i].stream_name);
		dev_info(&pdev->dev, "cpu %s(%s)\n", dai_link[i].cpu_of_node->name,
				dai_link[i].cpu_of_node->full_name);
		dev_info(&pdev->dev, "platform %s(%s)\n", dai_link[i].platform_of_node->name,
				dai_link[i].platform_of_node->full_name);
		dev_info(&pdev->dev, "codec dai %s\n", dai_link[i].codec_dai_name);
		dev_info(&pdev->dev, "codec %s(%s)\n", dai_link[i].codec_of_node->name,
				dai_link[i].codec_of_node->full_name);

		if (!strcmp(dai_link[i].name, "i2s-icdc") ||
				!strcmp(dai_link[i].codec_dai_name, "icdc-d3-hifi")) {
			dev_err(&pdev->dev, "found i2s-icdc on link %s, with dai %s, idx %d\n", dai_link[i].name, dai_link[i].codec_dai_name, i);
			dai_link->ops = &t31_i2s_cdc_ops;
			dai_link->init = t31_i2s_cdc_dai_link_init;
		}
	}

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, t31);
	dev_info(&pdev->dev, "Sound Card successed\n");
	return ret;
}

static int snd_t31_remove(struct platform_device *pdev)
{
	struct t31_icdc *t31 = platform_get_drvdata(pdev);
	snd_soc_unregister_card(&t31->card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id sound_dt_match[] = {
	{ .compatible = "ingenic,t31-sound", .data = NULL },
	{},
};
MODULE_DEVICE_TABLE(of, sound_dt_match);

static struct platform_driver snd_t31_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "t31-sound",
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(sound_dt_match),
	},
	.probe = snd_t31_probe,
	.remove = snd_t31_remove,
};
module_platform_driver(snd_t31_driver);

MODULE_AUTHOR("sccheng<shicheng.cheng@ingenic.com>");
MODULE_DESCRIPTION("ALSA SoC t31 Snd Card");
MODULE_LICENSE("GPL");
