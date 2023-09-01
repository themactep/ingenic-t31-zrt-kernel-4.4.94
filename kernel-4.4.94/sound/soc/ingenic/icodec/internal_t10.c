/*
 * sound/soc/ingenic/icodec/internal_t10.c
 * ALSA SoC Audio driver -- ingenic internal codec (internal_t10) driver

 * Copyright 2014 Ingenic Semiconductor Co.,Ltd
 *	cscheng <shicheng.cheng@ingenic.com>
 *
 * Note: internal_t10 is an internal codec for ingenic SOC
 *	 used for x1000 and so on
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <asm/div64.h>
#include <sound/soc-dai.h>
#include <linux/of_address.h>
#include "internal_t10.h"

static int internal_t10_debug = 0;
module_param(internal_t10_debug, int, 0644);
#define DEBUG_MSG(msg...)			\
	do {					\
		if (internal_t10_debug)		\
			printk("ICDC: " msg);	\
	} while(0)

static u8 internal_t10_reg_defcache[TS_CODEC_CCAGVR_4c] = { 0x0 };


static int internal_t10_write(struct snd_soc_codec *codec, unsigned int reg,
			unsigned int value)
{
	struct internal_t10 *internal_t10 = snd_soc_codec_get_drvdata(codec);
	int val = value;
	BUG_ON(reg > TS_CODEC_CCAGVR_4c);
	dev_dbg(internal_t10->dev,"%s reg = %x value = %x \n",__func__,reg,val);

	writel(value, internal_t10->mapped_base + reg);

	return 0;
}

static unsigned int internal_t10_read(struct snd_soc_codec *codec, unsigned int reg)
{
	struct internal_t10 *internal_t10 = snd_soc_codec_get_drvdata(codec);
	int val = 0;
	BUG_ON(reg > TS_CODEC_CCAGVR_4c);

	return readl(internal_t10->mapped_base + reg);
}


int codec_reg_set(struct snd_soc_codec *codec, unsigned int reg, int start, int end, int val)
{
	int ret = 0;
	int i = 0, mask = 0;
	unsigned int oldv = 0;
	unsigned int new = 0;
	for (i = 0;  i < (end - start + 1); i++) {
		mask += (1 << (start + i));
	}
	return snd_soc_update_bits(codec, reg, mask, val);
}


static int internal_t10_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	int playback = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK);
	int bit_width_sel = 3;
	int speed_sel = 0;
	//int aicr_reg = playback ? SCODA_REG_AICR_DAC : SCODA_REG_AICR_ADC;
	//int fcr_reg = playback ? SCODA_REG_FCR_DAC : SCODA_REG_FCR_ADC;


	int speed = params_rate(params);
	int bit_width = params_format(params);
	DEBUG_MSG("%s enter  set bus width %d , sample rate %d\n",
			__func__, bit_width, speed);
	/* bit width */
	switch (bit_width) {
	case SNDRV_PCM_FORMAT_S16_LE:
		bit_width_sel = 0;
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
		bit_width_sel = 1;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		bit_width_sel = 2;
		break;
	default:
	case SNDRV_PCM_FORMAT_S24_3LE:
		bit_width_sel = 3;
		break;
	}

	/*sample rate*/
	unsigned int mrate[8] = {8000, 12000, 16000, 24000, 32000, 44100, 48000, 96000};
	unsigned int rate_fs[8] = {7, 6, 5, 4, 3, 2, 1, 0};
	for (speed_sel = 0; speed_sel < 8; speed_sel++) {
		if (speed <= mrate[speed_sel])
			break;
	}
	if (speed_sel == 8)
		speed_sel = 0;
	codec_reg_set(codec, TS_CODEC_CSRR_44, 0, 2, rate_fs[speed_sel]);


	return 0;
}


#if (defined(CONFIG_SOC_T31) || defined(CONFIG_SOC_C100))
static int codec_set_buildin_mic(struct snd_soc_codec *codec)
{
	/* enable POP sound control, low vcm power */
	codec_reg_set(codec, TS_CODEC_CANACR_26, 0, 1, 0x2);
	msleep(10);

	/* enable hpf and set mode 0 */
	codec_reg_set(codec, TS_CODEC_CGAINLR_09, 0, 4, 0x8);
	msleep(10);

	/* ADC valid data length */
	codec_reg_set(codec, TS_CODEC_CACR_02, 5, 6, 0x0);
	msleep(10);
	/* Choose ADC I2S interface mode */
	codec_reg_set(codec, TS_CODEC_CACR_02, 3, 4, 0x2);
	msleep(10);
	/* Choose ADC chn */
	codec_reg_set(codec, TS_CODEC_CACR_02, 0, 0, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CACR_02, 7, 7, 0x1);
	msleep(10);

	/* Choose ADC I2S Master Mode */
	codec_reg_set(codec, TS_CODEC_CACR2_03, 0, 7, 0x3e);
	msleep(10);

	/* Enable ADC */
	codec_reg_set(codec, TS_CODEC_CMICCR_23, 7, 7, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAACR_22, 4, 4, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAACR_22, 3, 3, 0x1);   //micbias enable
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAACR_22, 0, 2, 0x7);   //micbias
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CMICCR_23, 5, 5, 0x1);  //ref voltage
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CMICGAINR_24, 4, 4, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CMICGAINR_24, 5, 5, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CANACR_26, 6, 6, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CANACR_26, 5, 5, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CANACR_26, 4, 4, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CANACR_26, 7, 7, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CMICCR_23, 6, 6, 0x1);
	msleep(10);

	/*if (mic_gain) {
		codec_reg_set(codec, TS_CODEC_CMICGAINR_24, 6, 7, mic_gain+1);
		msleep(10);
	}*/
	codec_reg_set(codec, TS_CODEC_CALCGR_25, 0, 4, 0x10);
	msleep(10);

	/*if(alc_mode) {
		codec_reg_set(codec, TS_CODEC_CMICCR_23, 4, 4, 1);
		msleep(10);
	}*/

	/* low power consumption */
	codec_reg_set(codec, TS_CODEC_CCR_21, 0, 6, 0x1);
	msleep(10);
	return 0;
}
#elif (defined(CONFIG_SOC_T30) || defined(CONFIG_SOC_T21))
static int codec_set_buildin_mic(struct snd_soc_codec *codec)
{
	/* ADC valid data length */
	codec_reg_set(codec, TS_CODEC_CACR_02, 5, 6, 0x0);
	msleep(10);
	/* Choose ADC I2S interface mode */
	/*codec_reg_set(codec, TS_CODEC_CACR_02, 3, 4, ADC_I2S_INTERFACE_LJ_MODE);*/
	codec_reg_set(codec, TS_CODEC_CACR_02, 3, 4, 0x2);
	msleep(10);
	/* Choose ADC chn */
	codec_reg_set(codec, TS_CODEC_CACR_02, 0, 0, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CACR_02, 7, 7, 0x1);
	msleep(10);

	/* Choose ADC I2S Master Mode */
	codec_reg_set(codec, TS_CODEC_CMCR_03, 0, 7, 0x3e);
	msleep(10);

	/* set sample rate */
	/*codec_reg_set(codec, TS_CODEC_CSRR_44, 0, 2, 0x7);*/
	msleep(10);

	/* set record volume */
	codec_reg_set(codec, TS_CODEC_CAACR_21, 7, 7, 1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAACR_21, 6, 6, 1);
	msleep(10);
	/* set MIC Bias is 1.6 * vcc */
	codec_reg_set(codec, TS_CODEC_CAACR_21, 0, 2, 7);
	msleep(10);
	/* enable ref voltage for ADC */
	codec_reg_set(codec, TS_CODEC_CAMPCR_24, 7, 7, 1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAACR_21, 4, 4, 1);
	msleep(10);
	/* enable ALC mode */
	codec_reg_set(codec, TS_CODEC_CMICCR_22, 0, 0, 1);
	msleep(10);
	/* enable ADC clk and ADC amplifier */
	codec_reg_set(codec, TS_CODEC_CAMPCR_24, 6, 6, 1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAACR_21, 3, 3, 1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAMPCR_24, 4, 4, 1);
	msleep(10);
	/* enable BST mode */
	codec_reg_set(codec, TS_CODEC_CMICCR_22, 6, 6, 1);
	msleep(10);

	/* MIC mode: 1: Signal-ended input , 0: Full Diff input */
	codec_reg_set(codec, TS_CODEC_CACR2_23, 5, 5, sign_mode?1:0);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CMICCR_22, 3, 3, 1);
	msleep(10);

	codec_reg_set(codec, TS_CODEC_CMICCR_22, 1, 2, mic_gain);
	msleep(10);

	/* record ALC gain 6dB */
	codec_reg_set(codec, TS_CODEC_CACR2_23, 0, 4, 0x10);
	msleep(10);

	if(alc_mode) {
		codec_reg_set(codec, TS_CODEC_CAACR_21, 5, 5, 1);
	}
	msleep(10);

	if(agc_mode) {
		codec_reg_set(codec, TS_CODEC_CAGCCR_42, 4, 4, 1);
		if (agc_noise) codec_reg_set(codec, TS_CODEC_CAGCCR_42, 3, 3, 1);
	}
	msleep(10);

	return 0;
}
#else
static int codec_set_buildin_mic(struct snd_soc_codec *codec)
{
	/* ADC valid data length */
	codec_reg_set(codec, TS_CODEC_CACR_02, 5, 6, 0x0);
	msleep(10);
	/* Choose ADC I2S interface mode */
	/*codec_reg_set(codec, TS_CODEC_CACR_02, 3, 4, ADC_I2S_INTERFACE_LJ_MODE);*/
	codec_reg_set(codec, TS_CODEC_CACR_02, 3, 4, 0x2);
	msleep(10);
	/* Choose ADC chn */
	codec_reg_set(codec, TS_CODEC_CACR_02, 0, 0, 0x1);
	msleep(10);

	/* Choose ADC I2S Master Mode */
	codec_reg_set(codec, TS_CODEC_CMCR_03, 0, 7, 0x3e);
	msleep(10);

	/* set sample rate */
	/*codec_reg_set(codec, TS_CODEC_CSRR_44, 0, 2, 0x7);*/
	msleep(10);

	/* set record volume */
	codec_reg_set(codec, TS_CODEC_CACR2_23, 0, 4, 0xc);
	msleep(10);

	/* MIC mode: 1: Signal-ended input , 0: Full Diff input */
	codec_reg_set(codec, TS_CODEC_CACR2_23, 5, 5, sign_mode?1:0);
	msleep(10);
	/* enable current source of ADC module, enable mic bias */
	codec_reg_set(codec, TS_CODEC_CAACR_21, 7, 7, 1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAACR_21, 6, 6, 1);
	msleep(10);
	/* set MIC Bias is 1.6 * vcc */
	codec_reg_set(codec, TS_CODEC_CAACR_21, 0, 2, 6);
	msleep(10);
	/* enable ref voltage for ADC */
	codec_reg_set(codec, TS_CODEC_CAMPCR_24, 7, 7, 1);
	msleep(10);
	/* enable BST mode */
	codec_reg_set(codec, TS_CODEC_CMICCR_22, 6, 6, 1);
	msleep(10);
	/* enable ALC mode */
	codec_reg_set(codec, TS_CODEC_CMICCR_22, 0, 0, 1);
	msleep(10);
	/* enable ADC clk and ADC amplifier */
	codec_reg_set(codec, TS_CODEC_CAMPCR_24, 6, 6, 1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAMPCR_24, 5, 5, 1);
	msleep(10);
	/* make ALC in work state */
	codec_reg_set(codec, TS_CODEC_CMICCR_22, 1, 1, 1);
	msleep(10);
	/* make BST in work state */
	codec_reg_set(codec, TS_CODEC_CMICCR_22, 4, 4, 1);
	msleep(10);
	/* enable zero-crossing detection */
	codec_reg_set(codec, TS_CODEC_CAACR_21, 5, 5, 1);
	msleep(10);
	/* record ALC gain 6dB */
	codec_reg_set(codec, TS_CODEC_CACR2_23, 0, 4, 0x13);
	msleep(10);

	if(mic_gain){
		/* record fix alg gain 20dB */
		codec_reg_set(codec, TS_CODEC_CMICCR_22, 5, 5, 1);
		msleep(10);
	}

	return 0;
}
#endif


#if (defined(CONFIG_SOC_T31) || defined(CONFIG_SOC_C100))
static int codec_set_speaker(struct snd_soc_codec *codec)
{
	int val = -1;

	/* low power consumption */
	codec_reg_set(codec, TS_CODEC_POWER_20, 0, 2, 0x7);
	codec_reg_set(codec, TS_CODEC_CMICCR_23, 0, 3, 0xf);

	/* enable current source */
	codec_reg_set(codec, TS_CODEC_CANACR_26, 3, 3, 0x1);
	msleep(10);

	/* enable reference voltage */
	codec_reg_set(codec, TS_CODEC_CANACR_26, 2, 2, 0x1);
	msleep(10);
	/* enable POP sound control */
	codec_reg_set(codec, TS_CODEC_CANACR_26, 0, 1, 0x2);
	msleep(10);

	codec_reg_set(codec, TS_CODEC_CHR_28, 5, 5, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CHR_28, 6, 6, 0x1);
	msleep(10);

	codec_reg_set(codec, TS_CODEC_CANACR2_27, 7, 7, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CANACR2_27, 6, 6, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CANACR2_27, 5, 5, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CANACR2_27, 4, 4, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CHR_28, 7, 7, 0x1);
	msleep(10);

	/* set replay volume */
	codec_reg_set(codec, TS_CODEC_CHR_28, 0, 4, 0x18);
	msleep(10);

	/* low power consumption */
	codec_reg_set(codec, TS_CODEC_CCR_21, 0, 6, 0x1);
	msleep(10);

	return 0;
}
#elif (defined(CONFIG_SOC_T30)||defined(CONFIG_SOC_T21))
static int codec_set_speaker(struct snd_soc_codec *codec)
{
	int val = -1;

	codec_reg_set(codec, TS_CODEC_CAACR_21, 7, 7, 0x1);
	msleep(10);
	/* enable current source */
	codec_reg_set(codec, TS_CODEC_CAR_25, 5, 5, 1);
	msleep(10);
	/* enable reference voltage */
	codec_reg_set(codec, TS_CODEC_CAR_25, 3, 3, 1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAR_25, 2, 2, 1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAR_25, 1, 1, 1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAR_25, 0, 0, 1);
	msleep(10);
	/* enable POP sound control */
	codec_reg_set(codec, TS_CODEC_CHCR_27, 6, 7, 2);
	msleep(10);
	/* enable HP OUT */
	codec_reg_set(codec, TS_CODEC_CHR_26, 7, 7, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CHR_26, 6, 6, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CMICCR_22, 4, 4, 0x1);
	msleep(10);

	/* set sample rate */
	/*codec_reg_set(codec, TS_CODEC_CSRR_44, 0, 2, 0x7);*/
	msleep(10);

	/* set replay volume */
	codec_reg_set(codec, TS_CODEC_CHCR_27, 0, 4, 0x18);

	if (codec_platform_data->gpio_spk_en.gpio > 0) {
		val = gpio_get_value(codec_platform_data->gpio_spk_en.gpio);
		gpio_direction_output(codec_platform_data->gpio_spk_en.gpio, codec_platform_data->gpio_spk_en.active_level);
		val = gpio_get_value(codec_platform_data->gpio_spk_en.gpio);
	}

	return 0;
}
#else
static int codec_set_speaker(struct snd_soc_codec *codec)
{
	int val = -1;

	codec_reg_set(codec, TS_CODEC_CAACR_21, 6, 7, 0x3);
	msleep(10);
	/* enable current source */
	codec_reg_set(codec, TS_CODEC_CAR_25, 6, 6, 1);
	msleep(10);
	/* enable reference voltage */
	codec_reg_set(codec, TS_CODEC_CAR_25, 5, 5, 1);
	msleep(10);
	/* enable POP sound control */
	codec_reg_set(codec, TS_CODEC_CHCR_27, 6, 7, 2);
	msleep(10);
	/* enable ADC */
	codec_reg_set(codec, TS_CODEC_CAR_25, 3, 3, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAR_25, 2, 2, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAR_25, 1, 1, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CAR_25, 0, 0, 0x1);
	msleep(10);
	/* enable HP OUT */
	codec_reg_set(codec, TS_CODEC_CHR_26, 7, 7, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CHR_26, 6, 6, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CHR_26, 5, 5, 0x1);
	msleep(10);

	/* set sample rate */
	/*codec_reg_set(codec, TS_CODEC_CSRR_44, 0, 2, 0x7);*/
	msleep(10);

	/* set replay volume */
	codec_reg_set(codec, TS_CODEC_CHCR_27, 0, 4, 0x18);

	if (codec_platform_data->gpio_spk_en.gpio > 0) {
		val = gpio_get_value(codec_platform_data->gpio_spk_en.gpio);
		gpio_direction_output(codec_platform_data->gpio_spk_en.gpio, codec_platform_data->gpio_spk_en.active_level);
		val = gpio_get_value(codec_platform_data->gpio_spk_en.gpio);
	}

	return 0;
}
#endif

static int internal_t10_trigger(struct snd_pcm_substream * stream, int cmd,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = stream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	int playback = (stream->stream == SNDRV_PCM_STREAM_PLAYBACK);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (playback) {
			return codec_set_speaker(codec);
		} else {
			return codec_set_buildin_mic(codec);
		}
	};

#ifdef DEBUG
	struct snd_soc_codec *codec = dai->codec;
	struct internal_t10 *internal_t10 = snd_soc_codec_get_drvdata(codec);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		dump_registers_hazard(internal_t10);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		dump_registers_hazard(internal_t10);
		break;
	}
#endif
	return 0;
}

#define DLV4780_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE | \
			 SNDRV_PCM_FMTBIT_S20_3LE |SNDRV_PCM_FMTBIT_S24_LE)

static int ingenic_icdc_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	int i = 0;
	char value = 0;

	codec_reg_set(codec, TS_CODEC_CGR_00, 0, 1, 0);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CGR_00, 0, 1, 0x3);
	msleep(10);

#if (!defined(CONFIG_SOC_T31) && !defined(CONFIG_SOC_C100))
	/* Choose DAC I2S Master Mode */
	codec_reg_set(codec, TS_CODEC_CMCR_03, 0, 7, 0x3e);
	codec_reg_set(codec, TS_CODEC_CDCR1_04, 0, 7, 0x10);//DAC I2S interface is I2S Mode.
	codec_reg_set(codec, TS_CODEC_CDCR2_05, 0, 7, 0xe);

	/* codec precharge */
	codec_reg_set(codec, TS_CODEC_CHCR_27, 6, 7, 1);
	msleep(10);
#else
	/* Choose DAC I2S Master Mode */
	codec_reg_set(codec, TS_CODEC_CACR2_03, 0, 7, 0x3e);
	codec_reg_set(codec, TS_CODEC_CDCR1_04, 0, 7, 0x10);//DAC I2S interface is I2S Mode.
	codec_reg_set(codec, TS_CODEC_CDCR2_05, 0, 7, 0x3e);

	codec_reg_set(codec, TS_CODEC_CANACR_26, 0, 1, 0x1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CCR_21, 0, 6, 0x1);
	msleep(10);
#endif

#ifdef CONFIG_SOC_T20
	codec_reg_set(codec, TS_CODEC_CCR_28, 7, 7, 1);
	msleep(10);
	for (i = 0; i < 6; i++) {
	     codec_reg_set(codec, TS_CODEC_CCR_28, 0, 6, 0x3f >> (6 - i));
		msleep(30);
	}
	/*msleep(10);*/
	codec_reg_set(codec, TS_CODEC_CCR_28, 0, 6, 0x3f);
#elif (defined(CONFIG_SOC_T30) || defined(CONFIG_SOC_T21))
	codec_reg_set(codec, TS_CODEC_CCR_28, 0, 6, 1);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CCR_28, 7, 7, 1);
	msleep(10);
	for (i = 0; i < 6; i++) {
		value |= value<<1 | 1;
		codec_reg_set(codec, TS_CODEC_CCR_28, 0, 6, value);
		msleep(30);
	}
	/*msleep(10);*/
	codec_reg_set(codec, TS_CODEC_CCR_28, 0, 6, 1);
#elif (defined(CONFIG_SOC_T31) || defined(CONFIG_SOC_C100))
	codec_reg_set(codec, TS_CODEC_CAACR_22, 5, 5, 1);//setup reference voltage
	msleep(10);
	for (i = 0; i <= 6; i++) {
		value |= value<<1 | 1;
		codec_reg_set(codec, TS_CODEC_CCR_21, 0, 6, value);
		msleep(30);
	}
#else	/* CONFIG_SOC_T10 */
	codec_reg_set(codec, TS_CODEC_CCR_28, 7, 7, 0);
	msleep(10);
	codec_reg_set(codec, TS_CODEC_CCR_28, 0, 6, 0x3f);
	msleep(10);
	for (i = 0; i < 6; i++) {
		codec_reg_set(codec, TS_CODEC_CCR_28, 0, 6, 0x3f >> i);
		msleep(30);
	}
	/*msleep(20);*/
	codec_reg_set(codec, TS_CODEC_CCR_28, 0, 6, 0x0);
#endif
	msleep(20);

	return 0;
}


static void ingenic_icdc_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	return;
}


int internal_t10_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_codec *codec = dai->codec;

	return 0;
}


static struct snd_soc_dai_ops internal_t10_dai_ops = {
	.hw_params	= internal_t10_hw_params,
	.mute_stream	= internal_t10_mute_stream,
	.trigger	= internal_t10_trigger,
	.shutdown	= ingenic_icdc_shutdown,
	.startup	= ingenic_icdc_startup,
};

static struct snd_soc_dai_driver  internal_t10_codec_dai = {
	.name = "icdc-d3-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = DLV4780_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = DLV4780_FORMATS,
	},
	.ops = &internal_t10_dai_ops,
};


static const struct snd_kcontrol_new internal_t10_snd_controls[] = {
	/* Volume controls */
	SOC_SINGLE("Playback Volume", TS_CODEC_CHR_28, 0, 31, 0),
	SOC_SINGLE("Record Volume", TS_CODEC_CALCGR_25, 0, 31, 0),
};

static const struct snd_soc_dapm_widget internal_t10_dapm_widgets[] = {
/* ADC */
	//SND_SOC_DAPM_ADC("ADC", "Capture" , SCODA_REG_AICR_ADC, 4, 1),
	SND_SOC_DAPM_MICBIAS("MICBIAS", TS_CODEC_CAACR_22, 3, 0),
	//SND_SOC_DAPM_PGA("AMIC", SCODA_REG_CR_MIC1, 4, 1, NULL, 0),
	//SND_SOC_DAPM_PGA("DMIC", SCODA_REG_CR_DMIC, 7, 0, NULL, 0),

/* DAC */
	//SND_SOC_DAPM_DAC("DAC", "Playback", SCODA_REG_AICR_DAC, 4, 1),

	//SND_SOC_DAPM_MUX("DAC_MERCURY VMux", SND_SOC_NOPM, 0, 0, &internal_t10_mercury_vmux_controls),
	//SND_SOC_DAPM_PGA("DAC_MERCURY", SCODA_REG_CR_DAC, 4, 1, NULL, 0),

/*	SND_SOC_DAPM_MUX("DAC_TITANIUM VMux", SND_SOC_NOPM, 0, 0, &internal_t10_titanium_vmux_controls),*/
/*	SND_SOC_DAPM_PGA("DAC_TITANIUM", SCODA_REG_CR_DAC2, 4, 1, NULL, 0),*/

/* MIXER */
	//SND_SOC_DAPM_MUX("MERCURY AIDAC MIXER Mux", SND_SOC_NOPM, 0, 0, &internal_t10_dac_input_sel_controls),
	//SND_SOC_DAPM_MUX("DAC Mode Mux", SND_SOC_NOPM, 0, 0, &internal_t10_mercury_mixer_mode_sel_controls),
	//SND_SOC_DAPM_MUX("MERCURY AIDAC Mux", SND_SOC_NOPM, 0, 0, &internal_t10_mercury_aidac_input_sel_controls),

/* ADC */
	//SND_SOC_DAPM_MUX("ADC Mode Mux", SND_SOC_NOPM, 0, 0, &internal_t10_aiadc_mixer_mode_sel_controls),
	//SND_SOC_DAPM_MUX("AIADC Mux", SND_SOC_NOPM, 0, 0, &internal_t10_aiadc_input_sel_controls),
	//SND_SOC_DAPM_MUX("AIADC Mux L", SND_SOC_NOPM, 0, 0, &internal_t10_aiadc_input_sel_controls_l),
	//SND_SOC_DAPM_MUX("AIADC Mux R", SND_SOC_NOPM, 0, 0, &internal_t10_aiadc_input_sel_controls_r),
	//SND_SOC_DAPM_MUX("ADC MIXER Mux", SND_SOC_NOPM, 0, 0, &internal_t10_adc_input_sel_controls),

/* PINS */
	SND_SOC_DAPM_INPUT("DMIC"),
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route intercon[] = {

	/*input*/
	{ "DMIC", NULL, "DMIC" },

	/*output*/
	{ "SPK"  , NULL, "SPK" },
};

#ifdef CONFIG_PM
static int internal_t10_suspend(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	snd_soc_dapm_force_bias_level(dapm, SND_SOC_BIAS_OFF);
	return 0;
}

static int internal_t10_resume(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	snd_soc_dapm_force_bias_level(dapm, SND_SOC_BIAS_STANDBY);
	return 0;
}
#endif

static int internal_t10_probe(struct snd_soc_codec *codec)
{
	struct internal_t10 *internal_t10 = snd_soc_codec_get_drvdata(codec);
	dev_info(codec->dev, "codec internal-t10 probe enter\n");

	internal_t10->codec = codec;
	return 0;
}

static int internal_t10_remove(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	dev_info(codec->dev, "codec internal_t10 remove enter\n");
	snd_soc_dapm_force_bias_level(dapm, SND_SOC_BIAS_OFF);
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_internal_t10_codec = {
	.probe = 	internal_t10_probe,
	.remove = 	internal_t10_remove,
#ifdef CONFIG_PM
	.suspend =	internal_t10_suspend,
	.resume =	internal_t10_resume,
#endif

	.read = 	internal_t10_read,
	.write = 	internal_t10_write,
	.reg_cache_default = internal_t10_reg_defcache,
	.reg_word_size = sizeof(u8),
	.reg_cache_step = 1,
	.reg_cache_size = TS_CODEC_CCAGVR_4c,

	.controls = 	internal_t10_snd_controls,
	.num_controls = ARRAY_SIZE(internal_t10_snd_controls),
	.dapm_widgets = internal_t10_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(internal_t10_dapm_widgets),
	.dapm_routes = intercon,
	.num_dapm_routes = ARRAY_SIZE(intercon),
};

/*Just for debug*/
static ssize_t hw_regs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct internal_t10 *internal_t10 = dev_get_drvdata(dev);
	struct snd_soc_codec *codec = internal_t10->codec;
	if (!codec) {
		dev_info(dev, "internal_t10 is not probe, can not use %s function\n", __func__);
		return 0;
	}
	mutex_lock(&codec->component.io_mutex);
	//dump_registers_hazard(internal_t10);
	mutex_unlock(&codec->component.io_mutex);
	return 0;
}

static ssize_t hw_regs_store(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t count)
{
	struct internal_t10 *internal_t10 = dev_get_drvdata(dev);
	struct snd_soc_codec *codec = internal_t10->codec;
	const char *start = buf;
	unsigned int reg, val;
	int ret_count = 0;

	if (!codec) {
		dev_info(dev, "internal_t10 is not probe, can not use %s function\n", __func__);
		return count;
	}

	while(!isxdigit(*start)) {
		start++;
		if (++ret_count >= count)
			return count;
	}
	reg = simple_strtoul(start, (char **)&start, 16);
	while(!isxdigit(*start)) {
		start++;
		if (++ret_count >= count)
			return count;
	}
	val = simple_strtoul(start, (char **)&start, 16);
	mutex_lock(&codec->component.io_mutex);
	internal_t10_write(codec, reg, val);
	mutex_unlock(&codec->component.io_mutex);
	return count;
}

static struct device_attribute internal_t10_sysfs_attrs =
	__ATTR_RW(hw_regs);

static int internal_t10_platform_probe(struct platform_device *pdev)
{
	struct device_node *parent = of_get_parent(pdev->dev.of_node);
	struct internal_t10 *internal_t10 = NULL;
	int ret = 0;

	internal_t10 = (struct internal_t10*)devm_kzalloc(&pdev->dev,
			sizeof(struct internal_t10), GFP_KERNEL);
	if (!internal_t10)
		return -ENOMEM;

	internal_t10->mapped_base = of_iomap(parent, 0);
	if (IS_ERR(internal_t10->mapped_base)) {
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		return PTR_ERR(internal_t10->mapped_base);
	}

	internal_t10->dev = &pdev->dev;
	internal_t10->dac_user_mute = 1;
	internal_t10->aohp_in_pwsq = 0;
	spin_lock_init(&internal_t10->io_lock);
	platform_set_drvdata(pdev, (void *)internal_t10);

	ret = snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_internal_t10_codec, &internal_t10_codec_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Faild to register codec\n");
		platform_set_drvdata(pdev, NULL);
		return ret;
	}

	ret = device_create_file(&pdev->dev, &internal_t10_sysfs_attrs);
	dev_info(&pdev->dev, "codec icdc-d3 platfrom probe success\n");
	return 0;
}

static int internal_t10_platform_remove(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "codec icdc-d3 platform remove\n");
	snd_soc_unregister_codec(&pdev->dev);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id codec_dt_match[] = {
	{ .compatible = "ingenic,t10-internal", .data = NULL },
};
MODULE_DEVICE_TABLE(of, codec_dt_match);

static struct platform_driver internal_t10_codec_driver = {
	.driver = {
		.name = "icdc-d3",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(codec_dt_match),
	},
	.probe = internal_t10_platform_probe,
	.remove = internal_t10_platform_remove,
};

static int internal_t10_modinit(void)
{
	return platform_driver_register(&internal_t10_codec_driver);
}
module_init(internal_t10_modinit);

static void internal_t10_exit(void)
{
	platform_driver_unregister(&internal_t10_codec_driver);
}
module_exit(internal_t10_exit);

MODULE_DESCRIPTION("iCdc d3 Codec Driver");
MODULE_AUTHOR("sccheng<shicheng.cheng@ingenic.com>");
MODULE_LICENSE("GPL");
