/*
 * Copyright (c) 2012 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 *  dorado board lcd setup routines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pwm_backlight.h>
#include <linux/lcd.h>
#include <linux/of_gpio.h>
#include <soc/gpio.h>
#include <linux/fb.h>
#include <linux/backlight.h>

#include "../ingenicfb.h"
#include "../jz_dsim.h"
#include "../jz_mipi_dsi/jz_mipi_dsih_hal.h"

#define ST7703_SUPPORT_LANE_4_FPS_60 0
#define ST7703_SUPPORT_LANE_4_FPS_30 1
#define ST7703_SUPPORT_LANE_4_FPS_20 2
#define ST7703_SUPPORT_LANE_2_FPS_30 3

#define JZ_LCD_FPS_MAX  ST7703_SUPPORT_LANE_4_FPS_60

static struct dsi_cmd_packet st7703_cmd_list[] = {
	{0x99, 50, 0x00, {0x00}},  //sleep 5ms
	{0x39, 0x04, 0x00, { 0xB9, 0xF1, 0x12, 0x83, } },
#ifdef CONFIG_MIPI_4LANE
	{0x39, 0x1C, 0x00, { 0xBA, 0x33, 0x81, 0x05, 0xF9, 0x0E, 0x0E, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x25, 0x00, 0x91, 0x0A, 0x00, 0x00, 0x02, 0x4F, 0xD1, 0x00, 0x00, 0x37, } },
#endif
#ifdef CONFIG_MIPI_2LANE
	{0x39, 0x1C, 0x00, { 0xBA, 0x31, 0x81, 0x05, 0xF9, 0x0E, 0x0E, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x25, 0x00, 0x91, 0x0A, 0x00, 0x00, 0x02, 0x4F, 0xD1, 0x00, 0x00, 0x37, } },
#endif
	{0x39, 0x02, 0x00, { 0xB8, 0x26, } },
	{0x39, 0x04, 0x00, { 0xBF, 0x02, 0x11, 0x00, } },
	{0x39, 0x0B, 0x00, { 0xB3, 0x0C, 0x10, 0x0A, 0x50, 0x03, 0xFF, 0x00, 0x00, 0x00, 0x00, } },
	{0x39, 0x0A, 0x00, { 0xC0, 0x73, 0x73, 0x50, 0x50, 0x00, 0x00, 0x08, 0x70, 0x00, } },
	{0x39, 0x02, 0x00, { 0xBC, 0x46, } },
	{0x39, 0x02, 0x00, { 0xCC, 0x0B, } },
	{0x39, 0x02, 0x00, { 0xB4, 0x80, } },
	{0x39, 0x04, 0x00, { 0xB2, 0xC8, 0x12, 0x30,}},
	{0x39, 0x0F, 0x00, { 0xE3, 0x07, 0x07, 0x0B, 0x0B, 0x03, 0x0B, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x80, 0xC0, 0x10, }},
	{0x39, 0x0D, 0x00, { 0xC1, 0x25, 0x00, 0x32, 0x32, 0x77, 0xE1, 0xFF, 0xFF, 0xCC, 0xCC, 0x77,0x77, }},
	{0x39, 0x03, 0x00, { 0xB5, 0x0A, 0x0A, }},
	{0x39, 0x03, 0x00, { 0xB6, 0x50, 0x50,}},
	{0x29, 0x40, 0x00, { 0xE9, 0xC2,0x10,0x0F,0x00,0x00,0xB2,0xB8,0x12,0x31,0x23,0x48,0x8B, 0xB2, 0xB8, 0x47, 0x20, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x02, 0x46, 0x02, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0xF8, 0x13, 0x57, 0x13, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, }},
	{0x29, 0x3E, 0x00, { 0xEA, 0x00,0x1A,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x75,0x31,0x31,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x8F,0x64,0x20,0x20,0x88, 0x88,0x88,0x88,0x88,0x88,0x88,0x8F,0x23,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, }},
	{0x39, 0x23, 0x00, { 0xe0, 0x03, 0x13,0x17,0x26,0x2F,0x38,0x47,0x3A,0x07,0x0C,0x0E,0x12,0x14,0x12,0x14,0x12,0x1A,0x03,0x13, 0x17,0x26,0x2F,0x38,0x47,0x3A,0x07,0x0C,0x0E,0x12,0x14,0x12,0x14,0x12,0x1A, }},
	{0x15, 0x11, 0x00, { 0x00 }}, 	//Sleep out
	{0x99, 250, 0x00, {0x00}},		//delay 250ms
	{0x15, 0x29, 0x00, { 0x00}},  	// Display On
	{0x99, 50, 0x00, {0x00}},		//delay 50ms
};

struct board_gpio {
	short gpio;
	short active_level;
};

struct panel_dev {
	/* ingenic frame buffer */
	struct device *dev;
	struct lcd_panel *panel;

	/* common lcd framework */
	struct lcd_device *lcd;
	struct backlight_device *backlight;
	int power;

	struct regulator *vcc;
	struct board_gpio vdd_en;
	struct board_gpio bl;
	struct board_gpio rst;
	struct board_gpio oled;
	struct board_gpio lcd_pwm;

	struct mipi_dsim_lcd_device *dsim_dev;
};

struct panel_dev *panel;

#define lcd_to_master(a)     (a->dsim_dev->master)
#define lcd_to_master_ops(a) ((lcd_to_master(a))->master_ops)

struct st7703 {
	struct device *dev;
	unsigned int power;
	unsigned int id;

	struct lcd_device *ld;
	struct backlight_device *bd;

	struct mipi_dsim_lcd_device *dsim_dev;
};

struct fb_videomode jzfb_st7703_videomode[] = {
	//[0] 4lane 60fps
	{
		.name = "st7703",
		.refresh = 60,
		.xres = 720,
		.yres = 1280,
		.pixclock = KHZ2PICOS(56000),  //60
		.left_margin = 40,
		.right_margin = 40,
		.upper_margin = 16,
		.lower_margin = 14,
		.hsync_len = 20,
		.vsync_len = 4,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0
	},

	//[1] 4lane 30fps
	{
		.name = "st7703",
		.refresh = 30,
		.xres = 720,
		.yres = 1280,
		.pixclock = KHZ2PICOS(28000),
		/* .left_margin = 40+20, */
		/* .right_margin = 40+80, */
		.left_margin = 40,
		.right_margin = 40,
		.upper_margin = 16,
		.lower_margin = 14,
		.hsync_len = 20,
		.vsync_len = 4,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0
	},
	//[2] 4lane 20fps
	{
		.name = "st7703",
		.refresh = 20,
		.xres = 720,
		.yres = 1280,
		.pixclock = KHZ2PICOS(18800),
		.left_margin = 40,
		.right_margin = 40,
		.upper_margin = 16,
		.lower_margin = 14,
		.hsync_len = 20,
		.vsync_len = 4,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0
	},
	//[3] 2lane 30fps
	{
		.name = "st7703",
		.refresh = 10,
		.xres = 720,
		.yres = 1280,
		.pixclock = KHZ2PICOS(28000),
		.left_margin = 40,
		.right_margin = 40,
		.upper_margin = 16,
		.lower_margin = 14,
		.hsync_len = 20,
		.vsync_len = 4,
		.sync = FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		.vmode = FB_VMODE_NONINTERLACED,
		.flag = 0
	},

};

struct jzdsi_data jzdsi_pdata = {
	.modes = &jzfb_st7703_videomode[JZ_LCD_FPS_MAX],
#ifdef CONFIG_MIPI_2LANE
	.video_config.no_of_lanes = 2,
#endif
#ifdef CONFIG_MIPI_4LANE
	.video_config.no_of_lanes = 4,
#endif
	.video_config.virtual_channel = 0,
	.video_config.color_coding = COLOR_CODE_24BIT,
#if 0
	.video_config.video_mode = VIDEO_NON_BURST_WITH_SYNC_PULSES,
#else
	.video_config.video_mode = VIDEO_BURST_WITH_SYNC_PULSES,
#endif
	.video_config.receive_ack_packets = 0,	/* enable receiving of ack packets */
	/*loosely: R0R1R2R3R4R5__G0G1G2G3G4G5G6__B0B1B2B3B4B5B6,
	 * not loosely: R0R1R2R3R4R5G0G1G2G3G4G5B0B1B2B3B4B5*/
	.video_config.is_18_loosely = 0,
	.video_config.data_en_polarity = 1,

	.dsi_config.max_lanes = 4,
	.dsi_config.max_hs_to_lp_cycles = 100,
	.dsi_config.max_lp_to_hs_cycles = 40,
	.dsi_config.max_bta_cycles = 4095,
	.dsi_config.color_mode_polarity = 1,
	.dsi_config.shut_down_polarity = 1,
	.dsi_config.max_bps = 460,
	//	.max_bps = 650,  // 650 Mbps
	.bpp_info = 24,

};

struct tft_config st7703_cfg = {
	.pix_clk_inv = 0,
	.de_dl = 0,
	.sync_dl = 0,
	.color_even = TFT_LCD_COLOR_EVEN_RGB,
	.color_odd = TFT_LCD_COLOR_ODD_RGB,
	.mode = TFT_LCD_MODE_PARALLEL_888,
};

struct lcd_panel lcd_panel = {
	.num_modes = 1,
	.modes = &jzfb_st7703_videomode[JZ_LCD_FPS_MAX],
	.dsi_pdata = &jzdsi_pdata,
	//.smart_config = &smart_cfg,
	.tft_config = &st7703_cfg,

	.lcd_type = LCD_TYPE_TFT ,
	.width = 720,
	.height = 1280,
	//.bpp = 24,

	//.pixclk_falling_edge = 0,
	//.data_enable_active_low = 0,

};

/**************************************************************************************************/
#ifdef CONFIG_BACKLIGHT_PWM

static struct platform_pwm_backlight_data backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 100,
	.pwm_period_ns	= 30000,
};

struct platform_device backlight_device = {
	.name		= "pwm-backlight",
	.dev		= {
		.platform_data	= &backlight_data,
	},
};

#endif

#define POWER_IS_ON(pwr)    ((pwr) <= FB_BLANK_NORMAL)
static int panel_set_power(struct lcd_device *lcd, int power)
{
	return 0;
}

static int panel_get_power(struct lcd_device *lcd)
{
	struct panel_dev *panel = lcd_get_data(lcd);

	return panel->power;
}

static struct lcd_ops panel_lcd_ops = {
	.early_set_power = panel_set_power,
	.set_power = panel_set_power,
	.get_power = panel_get_power,
};

static int of_panel_parse(struct device *dev)
{
	struct panel_dev *panel = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int ret = 0;

	panel->bl.gpio = of_get_named_gpio_flags(np, "ingenic,bl-gpio", 0, &flags);
	if (gpio_is_valid(panel->bl.gpio)) {
		panel->bl.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = devm_gpio_request(dev, panel->bl.gpio, "backlight");
		if (ret < 0) {
			dev_err(dev, "Failed to request backlight pin!\n");
			return ret;
		}
	} else
		dev_warn(dev, "invalid gpio backlight.gpio: %d\n", panel->bl.gpio);

	panel->rst.gpio = of_get_named_gpio_flags(np, "ingenic,rst-gpio", 0, &flags);
	if (gpio_is_valid(panel->rst.gpio)) {
		panel->rst.active_level = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
		ret = devm_gpio_request(dev, panel->rst.gpio, "reset");
		if (ret < 0) {
			dev_err(dev, "Failed to request rst pin!\n");
			return ret;
		}
	} else {
		dev_warn(dev, "invalid gpio rst.gpio: %d\n", panel->rst.gpio);
	}

	return 0;
}

static void panel_dev_panel_init(struct panel_dev *lcd)
{
	int i, j;
	struct dsi_master_ops *ops = lcd_to_master_ops(lcd);
	struct dsi_device *dsi = lcd_to_master(lcd);
	unsigned char buf[MAX_WORD_COUNT];
	unsigned char dsi_command_param[MAX_WORD_COUNT] = {0};

	memset(buf, 0, sizeof(buf));
#if 0
	//set maximum transfer datasize
	dsi_command_param[0] = 4;  //lsb
	dsi_command_param[1] = 0;  //msb
	mipi_dsih_gen_wr_packet(dsi, 0, 0x37, dsi_command_param, 2);
	printk("set maximum return packet size ok now...\n");

	//read display ID
	mipi_dsih_gen_rd_packet(dsi, 0, 0x14, 0, 0x04, 4, buf);
	printk("read buffer: \ncmd: 0x04, parameter:0x%x,0x%x,0x%x,0x%x\n", buf[0], buf[1], buf[2], buf[3]);

#endif

	for (i = 0; i < ARRAY_SIZE(st7703_cmd_list); i++) {
		if (st7703_cmd_list[i].packet_type == 0x99) {
			/* printk("\nsleep now. time: %dms\n", st7703_cmd_list[i].cmd0_or_wc_lsb+st7703_cmd_list[i].cmd1_or_wc_msb); */
			if (st7703_cmd_list[i].cmd0_or_wc_lsb+st7703_cmd_list[i].cmd1_or_wc_msb == 510)
				msleep(600);
			else
				msleep(st7703_cmd_list[i].cmd0_or_wc_lsb+st7703_cmd_list[i].cmd1_or_wc_msb);
			continue;
		}

		ops->cmd_write(dsi, st7703_cmd_list[i]);

#if 0
		//short packet read here
		if (st7703_cmd_list[i].packet_type == 0x15) {
			//we read command here, see whether we send command configure ok.
			mipi_dsih_gen_rd_packet(dsi, 0, 0x14, 0, st7703_cmd_list[i].cmd0_or_wc_lsb, 4, buf);
			printk("read buffer: \ncmd: 0x%x, parameter:0x%x,0x%x,0x%x,0x%x\n", st7703_cmd_list[i].cmd0_or_wc_lsb, buf[0], buf[1], buf[2], buf[3]);
		}
		//long packet read here
		if (st7703_cmd_list[i].packet_type == 0x39 || st7703_cmd_list[i].packet_type == 0x29) {
			//we read command here, see whether we send command configure ok.
			printk("return value : %d\t", mipi_dsih_gen_rd_packet(dsi, 0, 0x14, 0, st7703_cmd_list[i].cmd_data[0], 110, buf));
			printk("read buffer(data_type: 0x%x, cmd: 0x%x)\n", st7703_cmd_list[i].packet_type, st7703_cmd_list[i].cmd_data[0]);
			for (j = 0 ; j < 110; j++)
				printk("paramter list[%d]:0x%x\t", j, buf[j]);
			printk("\n");
		}
#endif
		memset(buf, 0, sizeof(buf));
	}

}

static int panel_dev_ioctl(struct mipi_dsim_lcd_device *dsim_dev, int cmd)
{
	return 0;
}

static void panel_dev_set_sequence(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct st7703 *lcd = dev_get_drvdata(&dsim_dev->dev);

	panel_dev_panel_init(panel);
	msleep(120);

	lcd->power = FB_BLANK_UNBLANK;
}

static void panel_dev_power_on(struct mipi_dsim_lcd_device *dsim_dev, int power)
{
	struct panel_dev *panel_st7703 = dev_get_drvdata(panel->dev);

	if (power == 1) {
		//reset
		if (gpio_is_valid(panel_st7703->rst.gpio)) {
			gpio_direction_output(panel_st7703->rst.gpio, !panel_st7703->rst.active_level);
			mdelay(120);
			gpio_direction_output(panel_st7703->rst.gpio, panel_st7703->rst.active_level);
			mdelay(50);
			gpio_direction_output(panel_st7703->rst.gpio, !panel_st7703->rst.active_level);
			mdelay(120);
		}
		//open backlight
		if (gpio_is_valid(panel_st7703->bl.gpio))
			gpio_direction_output(panel_st7703->bl.gpio, panel_st7703->bl.active_level);
	} else {
		if (gpio_is_valid(panel_st7703->bl.gpio))
			gpio_direction_output(panel_st7703->bl.gpio, !panel_st7703->bl.active_level);
	}
}

static int panel_dev_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct st7703 *lcd;

	lcd = devm_kzalloc(&dsim_dev->dev, sizeof(struct st7703), GFP_KERNEL);
	if (IS_ERR_OR_NULL(lcd)) {
		dev_err(&dsim_dev->dev, "Failed to allocate st7703 structure!\n");
		return -ENOMEM;
	}

	lcd->dsim_dev = dsim_dev;
	lcd->dev = &dsim_dev->dev;

	lcd->ld = lcd_device_register("st7703", lcd->dev, lcd,
								  &panel_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "Failed to register lcd ops.");
		return PTR_ERR(lcd->ld);
	}

	dev_set_drvdata(&dsim_dev->dev, lcd);

	dev_dbg(lcd->dev, "Now probed st7703 panel.\n");

	panel->dsim_dev = dsim_dev;

	return 0;
}

static int panel_dev_suspend(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct board_gpio *vdd_en = &panel->bl;

	gpio_direction_output(vdd_en->gpio, !vdd_en->active_level);

	return 0;
}

static int panel_dev_resume(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct board_gpio *vdd_en = &panel->bl;

	gpio_direction_output(vdd_en->gpio, vdd_en->active_level);

	return 0;
}

static struct mipi_dsim_lcd_driver panel_dev_dsim_ddi_driver = {
	.name = "st7703",
	.id = -1,

	.power_on = panel_dev_power_on,
	.set_sequence = panel_dev_set_sequence,
	.probe = panel_dev_probe,
	.suspend = panel_dev_suspend,
	.resume = panel_dev_resume,
	.ioctl = panel_dev_ioctl,
};

static struct mipi_dsim_lcd_device panel_dev_device = {
	.name = "st7703",
	.id = 0,
};

/**
 * @panel_probe
 *
 *   1. register to ingenicfb
 *   2. register to lcd
 *   3. register to backlight if possible
 *
 * @pdev
 *
 * @return -
 */
static int panel_probe(struct platform_device *pdev)
{
	int ret = 0;

	panel = kzalloc(sizeof(struct panel_dev), GFP_KERNEL);
	if (IS_ERR_OR_NULL(panel)) {
		dev_err(&pdev->dev, "Failed to alloc memory!\n");
		return -ENOMEM;
	}
	panel->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, panel);

	ret = of_panel_parse(&pdev->dev);
	if (ret < 0)
		goto err_of_parse;

	/* register to mipi-dsi devicelist*/
	mipi_dsi_register_lcd_device(&panel_dev_device);
	mipi_dsi_register_lcd_driver(&panel_dev_dsim_ddi_driver);

	ret = ingenicfb_register_panel(&lcd_panel);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register lcd panel!\n");
		goto err_of_parse;
	}

	return 0;
err_of_parse:
	kfree(panel);
	return ret;
}

static int panel_remove(struct platform_device *dev)
{
	if (NULL != panel)
		kfree(panel);

	return 0;
}

#ifdef CONFIG_PM
static int panel_suspend(struct device *dev)
{
	struct panel_dev *panel = dev_get_drvdata(dev);

	panel_set_power(panel->lcd, FB_BLANK_POWERDOWN);
	return 0;
}

static int panel_resume(struct device *dev)
{
	struct panel_dev *panel = dev_get_drvdata(dev);

	panel_set_power(panel->lcd, FB_BLANK_UNBLANK);
	return 0;
}

static const struct dev_pm_ops panel_pm_ops = {
	.suspend = panel_suspend,
	.resume = panel_resume,
};
#endif
static const struct of_device_id panel_of_match[] = {
	{ .compatible = "ingenic,st7703", },
	{ .compatible = "st7703", },
	{},
};

static struct platform_driver panel_driver = {
	.probe = panel_probe,
	.remove = panel_remove,
	.driver = {
		.name = "st7703",
		.of_match_table = panel_of_match,
#ifdef CONFIG_PM
		.pm = &panel_pm_ops,
#endif
	},
};

module_platform_driver(panel_driver);
