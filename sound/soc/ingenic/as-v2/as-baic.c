/*
 * ALSA Soc Audio Layer -- ingenic as(audio system) baic(Basic Audio Inter-
 * face Controller) driver
 *
 * Copyright 2017 - 2022 Ingenic Semiconductor Co.,Ltd
 *   cli <chen.li@ingenic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/types.h>
#include <linux/clk.h>
#include "as-baic.h"

#ifdef DEBUG
static int ingenic_baic_debug = 1;
#else
static int ingenic_baic_debug = 0;
#endif
module_param(ingenic_baic_debug, int, 0644);
#define BAIC_DEBUG_MSG(msg...)					\
	do {										\
		if (ingenic_baic_debug)					\
			printk(KERN_DEBUG"BAIC: " msg);				\
	} while(0)

static int ingenic_baic_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct ingenic_baic *baic = dev_get_drvdata(dai->dev);
	int id  = dai->driver->id;
	struct ingenic_baic_dai *idai = &baic->dai[id];
	u32 baic_cfg = 0;
	u16 select_mode = 0;
	bool frame_master = false;

	BAIC_DEBUG_MSG("enter [BAIC%d] %s dai fmt %x\n", id, __func__, fmt);

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_NB_IF:
		baic_cfg |= BAICCFG_ISYNC;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		baic_cfg |= BAICCFG_NEG;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		baic_cfg |= BAICCFG_ISYNC | BAICCFG_NEG;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:		/*i2s format*/
		if (!(idai->support_mode & BAIC_DAIFMT_I2S_GRP))
			return -EINVAL;
		baic_cfg |= BAICCFG_MODE_I2S;
		select_mode = BAIC_DAIFMT_I2S_GRP|BAIC_DAIFMT_I2S;
		break;
	case SND_SOC_DAIFMT_MSB:		/*msb/left_j format*/
		if (!(idai->support_mode & BAIC_DAIFMT_I2S_GRP))
			return -EINVAL;
		baic_cfg |= BAICCFG_MODE_LEFTJ;
		select_mode = BAIC_DAIFMT_I2S_GRP|BAIC_DAIFMT_LEFTJ;
		break;
	case SND_SOC_DAIFMT_LSB:		/*lsb/right_j format*/
		if (!(idai->support_mode & BAIC_DAIFMT_I2S_GRP))
			return -EINVAL;
		baic_cfg |= BAICCFG_MODE_RIGHTJ;
		select_mode = BAIC_DAIFMT_I2S_GRP|BAIC_DAIFMT_RIGHTJ;
		break;
	case SND_SOC_DAIFMT_DSP_A:		/*TDM/PCM/DSP A*/
		if (!(idai->support_mode & BAIC_DAIFMT_DSP_GRP))
			return -EINVAL;
		if (!(idai->support_mode & BAIC_DAIFMT_MODEA))
			return -EINVAL;

		if (idai->support_mode & BAIC_DAIFMT_TDM2) {
			baic_cfg |= BAICCFG_MODE_TDM2A;
			select_mode = BAIC_DAIFMT_TDM2|BAIC_DAIFMT_MODEA|BAIC_DAIFMT_DSP_GRP;
			break;
		}

		if (idai->support_mode & BAIC_DAIFMT_TDM1) {
			baic_cfg |= BAICCFG_MODE_TDM1A;
			select_mode = BAIC_DAIFMT_TDM1|BAIC_DAIFMT_MODEA|BAIC_DAIFMT_DSP_GRP;
			break;
		}

		if (idai->support_mode & BAIC_DAIFMT_DSP) {
			baic_cfg |= BAICCFG_MODE_DSPA;
			select_mode = BAIC_DAIFMT_DSP|BAIC_DAIFMT_MODEA|BAIC_DAIFMT_DSP_GRP;
			break;
		}

		if (idai->support_mode & BAIC_DAIFMT_PCM) {
			baic_cfg |= BAICCFG_MODE_PCMA;
			select_mode = BAIC_DAIFMT_PCM|BAIC_DAIFMT_MODEA|BAIC_DAIFMT_DSP_GRP;
			break;
		}
		return -EINVAL;
	case SND_SOC_DAIFMT_DSP_B:		/*TDM/PCM/DSP B*/
		if (!(idai->support_mode & BAIC_DAIFMT_DSP_GRP))
			return -EINVAL;

		if (!(idai->support_mode & BAIC_DAIFMT_MODEB))
			return -EINVAL;

		if (idai->support_mode & BAIC_DAIFMT_TDM2) {
			baic_cfg |= BAICCFG_MODE_TDM2B;
			select_mode = BAIC_DAIFMT_TDM2|BAIC_DAIFMT_MODEB|BAIC_DAIFMT_DSP_GRP;
			break;
		}

		if (idai->support_mode & BAIC_DAIFMT_TDM1) {
			baic_cfg |= BAICCFG_MODE_TDM1B;
			select_mode = BAIC_DAIFMT_TDM1|BAIC_DAIFMT_MODEB|BAIC_DAIFMT_DSP_GRP;
			break;
		}

		if (idai->support_mode & BAIC_DAIFMT_DSP) {
			baic_cfg |= BAICCFG_MODE_DSPB;
			select_mode = BAIC_DAIFMT_DSP|BAIC_DAIFMT_MODEB|BAIC_DAIFMT_DSP_GRP;
			break;
		}

		if (idai->support_mode & BAIC_DAIFMT_PCM) {
			baic_cfg |= BAICCFG_MODE_PCMB;
			select_mode = BAIC_DAIFMT_PCM|BAIC_DAIFMT_MODEB|BAIC_DAIFMT_DSP_GRP;
			break;
		}
	default:
		pr_err("%s:%d\n", __func__, __LINE__);
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	case SND_SOC_DAIFMT_CBS_CFS:	/*codec frame slave */
		baic_cfg |= BAICCFG_MASTER;
		frame_master = true;
		break;
	default:
		pr_err("%s:%d\n", __func__, __LINE__);
		return -EINVAL;
	}

	if (fmt & BAIC_DAIFMT_R) {
		idai->baic_rcfg = baic_cfg;
		idai->select_rmode = select_mode;
		idai->frame_rmaster = frame_master;
	} else if (fmt & BAIC_DAIFMT_T) {
		idai->baic_tcfg = baic_cfg;
		idai->select_tmode = select_mode;
		idai->frame_tmaster = frame_master;
	} else {
		idai->baic_rcfg = baic_cfg;
		idai->baic_tcfg = baic_cfg;
		idai->select_rmode = select_mode;
		idai->select_tmode = select_mode;
		idai->frame_tmaster = frame_master;
		idai->frame_rmaster = frame_master;
	}

	return 0;
}

static int ingenic_baic_set_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	struct ingenic_baic *baic = dev_get_drvdata(dai->dev);
	int id  = dai->driver->id;
	struct ingenic_baic_dai *idai = &baic->dai[id];
	int baic_cfg = 0, baic_msk = 0;

	if (!slots)
		return 0;
	if (!(idai->support_mode & (BAIC_DAIFMT_TDM2|BAIC_DAIFMT_TDM1|BAIC_DAIFMT_DSP)))
		goto error;
	if (!(idai->select_tmode & (BAIC_DAIFMT_TDM2|BAIC_DAIFMT_TDM1|BAIC_DAIFMT_DSP)) && tx_mask)
		goto error;
	if (!(idai->select_rmode & (BAIC_DAIFMT_TDM2|BAIC_DAIFMT_TDM1|BAIC_DAIFMT_DSP)) && rx_mask)
		goto error;

	baic_cfg = BAICCFG_SLOT(slots)|BAICCFG_SLS(slot_width);
	baic_msk = BAICCFG_SLOT_MSK|BAICCFG_SLS_MSK;

	if (tx_mask) {
		idai->baic_tcfg &= ~baic_msk;
		idai->baic_tcfg |= baic_cfg;
	}
	if (rx_mask) {
		idai->baic_rcfg &= ~baic_msk;
		idai->baic_rcfg |= baic_cfg;
	}
	return 0;
error:
	pr_err("%s:%d\n", __func__, __LINE__);
	return -EINVAL;
}

static int ingenic_baic_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
	struct ingenic_baic *baic = dev_get_drvdata(dai->dev);
	int id  = dai->driver->id;
	struct ingenic_baic_dai *idai = &baic->dai[id];
	struct ingenic_baic_div *idiv = (div_id & DIVID_DIR_T) ? &idai->tdiv.v : &idai->rdiv.v;
	u16 select_mode = (div_id & DIVID_DIR_T) ? idai->select_tmode : idai->select_rmode;
	u16 *bclk_ratio = (div_id & DIVID_DIR_T) ? &idai->bclk_tratio : &idai->bclk_rratio;
	bool frame_master = (div_id & DIVID_DIR_T) ? idai->frame_tmaster : idai->frame_rmaster;

	if (!frame_master)
		return 0;

	switch (div_id & DIVID_MSK) {
	case DIVID_SYNC_W:
		idiv->synl = (BAICDIV_SYNL(div) >> BAICDIV_SYNL_SFT);
		break;
	case DIVID_BCLK:
		idiv->bclkdiv = (BAICDIV_BCLKDIV(div) >> BAICDIV_BCLKDIV_SFT);
		break;
	case DIVID_SYNC:
		if (select_mode & BAIC_DAIFMT_I2S_GRP)
			idiv->sync_div = BAICDIV_SYNLDIV_I2S(div) >> BAICDIV_SYNLDIV_SFT;
		else
			idiv->sync_div = BAICDIV_SYNLDIV_DSP(div) >> BAICDIV_SYNLDIV_SFT;
		*bclk_ratio = div;
		break;
	default:
		pr_err("%s:%d\n", __func__, __LINE__);
		return -EINVAL;
	}

	return 0;
}

static int ingenic_baic_set_sysclk(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir)
{
	struct ingenic_baic *baic = dev_get_drvdata(dai->dev);
	int id = dai->driver->id;
	struct ingenic_baic_dai *idai = &baic->dai[id];

	BAIC_DEBUG_MSG("enter [BAIC%d] %s clk_id %d req %d clk dir %d\n", id, __func__,
			clk_id, freq, dir);
	switch (clk_id) {
	case CLKID_INNER_CODEC:
		regmap_write(baic->regmap, BAICTLCR(id), BAICTLCR_ICDC);
		break;
	case CLKID_SYSCLK:
		idai->tsysclk = freq;
		idai->rsysclk = freq;
		regmap_write(baic->regmap, BAICTLCR(id),
				idai->split_data_pin ? BAICTLCR_CLK_SPLIT_EN : 0);
		break;
	case CLKID_SYSCLK_R:
		idai->rsysclk = freq;
		regmap_write(baic->regmap, BAICTLCR(id),
				idai->split_data_pin ? BAICTLCR_CLK_SPLIT_EN : 0);
		break;
	case CLKID_SYSCLK_T:
		idai->tsysclk = freq;
		regmap_write(baic->regmap, BAICTLCR(id),
				idai->split_data_pin ? BAICTLCR_CLK_SPLIT_EN : 0);
		break;
	default:
		pr_err("%s:%d\n", __func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}

static bool ingenic_baic_fmt_check(u16 select_mode, int stream,
		int fmt_width, int channels, int pin_num)
{
	if (select_mode & BAIC_DAIFMT_I2S_GRP) {
		if (channels != 1 && ((pin_num * 2) % channels))
			return false;
		if (channels == 1 && stream == SNDRV_PCM_STREAM_CAPTURE)
			return false;
		return true;
	}
	if (!(select_mode & BAIC_DAIFMT_DSP_GRP))
		return false;


	if (select_mode & BAIC_DAIFMT_TDM1) {
		if (channels > 8 || (channels != 1 && (channels % 2)))
			return false;
		if (fmt_width % 8)
			return false;
		return true;
	}

	if (select_mode & BAIC_DAIFMT_TDM2) {
		if (channels != 8)
			return false;
		if (fmt_width % 8)
			return false;
		return true;
	}


	if (select_mode & BAIC_DAIFMT_PCM) {
		if (channels != 1)
			return false;
		switch (fmt_width) {
		case 8: case 16: case 32: return true;
		default:
			return false;
		}
	}

	if (select_mode & BAIC_DAIFMT_DSP) {
		if (channels != 1 && channels != 2)
			return false;
		switch (fmt_width) {
		case 8: case 16: case 32: return true;
		default:
			return false;
		}
	}
	return false;
}

static int ingenic_baic_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	int channels = params_channels(params);
	int rate = params_rate(params);
	int fmt_width = snd_pcm_format_width(params_format(params));
	struct ingenic_baic *baic = dev_get_drvdata(dai->dev);
	int id = dai->driver->id;
	struct ingenic_baic_dai *idai = &baic->dai[id];
	unsigned int cfgreg = BAICTCFG(id), divreg = BAICTDIV(id);
	bool is_out = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? true : false;
	struct ingenic_baic_div *idiv = is_out ? &idai->tdiv.v : &idai->rdiv.v;
	u32 *baic_cfg = is_out ? &idai->baic_tcfg : &idai->baic_rcfg;
	u16 select_mode = is_out ? idai->select_tmode : idai->select_rmode;
	u16 bclk_ratio = is_out ? idai->bclk_tratio : idai->bclk_rratio;
	u32 sysclk = is_out ? idai->rsysclk : idai->tsysclk;
	bool frame_master = is_out ? idai->frame_tmaster : idai->frame_rmaster;

	BAIC_DEBUG_MSG("enter [BAIC%d] %s, substream = %s, channels %d, rate %d, fmt_width %d\n",
			id, __func__, (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture",
			channels, rate, fmt_width);

	if (!ingenic_baic_fmt_check(select_mode, substream->stream,
				fmt_width, channels, idai->data_pin_num))
		return -EINVAL;

	if ((select_mode & BAIC_DAIFMT_I2S_GRP)) {
		(*baic_cfg) &= ~BAICCFG_CHANNEL_MSK;
		(*baic_cfg) |= BAICCFG_CHANNEL(channels);
	} else if (select_mode & BAIC_DAIFMT_TDM2) {
		(*baic_cfg) &= ~BAICCFG_CHANNEL_MSK;
		(*baic_cfg) |= BAICCFG_CHANNEL(4);
	}
	(*baic_cfg) &= ~BAICCFG_SS_MSK;
	(*baic_cfg) |= BAICCFG_SS(fmt_width);

	if (idai->split_data_pin && !is_out) {
		cfgreg = BAICRCFG(id);
		divreg = BAICRDIV(id);
	}

	regmap_write(baic->regmap, cfgreg, *baic_cfg);

	if (!frame_master)
		return 0;

	if (!idiv->bclkdiv) {
		int bclk, div;
		bclk = rate * bclk_ratio;
		div = ((sysclk + bclk - 1) / bclk) & (~0x1UL);
		idiv->bclkdiv =  BAICDIV_BCLKDIV(div) >> BAICDIV_BCLKDIV_SFT;
	}
	regmap_write(baic->regmap, divreg, is_out ? idai->tdiv.r: idai->rdiv.r);

	return 0;
}

static int ingenic_baic_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	struct ingenic_baic *baic = dev_get_drvdata(dai->dev);
	int id = dai->driver->id;

	BAIC_DEBUG_MSG("enter [BAIC%d] %s, substream = %s cmd = %d\n",  id, __func__,
			(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ? "playback" : "capture",
			cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		BAIC_DEBUG_MSG("baic start\n");
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			regmap_update_bits(baic->regmap, BAICCCR(id), BAICCCR_TEN, BAICCCR_TEN);
		else
			regmap_update_bits(baic->regmap, BAICCCR(id), BAICCCR_REN, BAICCCR_REN);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		BAIC_DEBUG_MSG("baic stop\n");
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			regmap_update_bits(baic->regmap, BAICCCR(id), BAICCCR_TEN, 0);
		else
			regmap_update_bits(baic->regmap, BAICCCR(id), BAICCCR_REN, 0);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		BAIC_DEBUG_MSG("baic pause\n");
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			regmap_update_bits(baic->regmap, BAICTCFG(id), BAICCFG_T_PAUSE, BAICCFG_T_PAUSE);
			regmap_update_bits(baic->regmap, BAICCCR(id), BAICCCR_TEN, 0);
		}
		break;
	}
	return 0;
}

static struct snd_soc_dai_ops ingenic_baic_dai_ops = {
	.trigger 	= ingenic_baic_trigger,
	.hw_params 	= ingenic_baic_hw_params,
	.set_fmt	= ingenic_baic_set_dai_fmt,
	.set_sysclk	= ingenic_baic_set_sysclk,
	.set_clkdiv	= ingenic_baic_set_clkdiv,
	.set_tdm_slot	= ingenic_baic_set_tdm_slot,
};

#define ingenic_baic_suspend	NULL
#define ingenic_baic_resume	NULL

static const struct snd_soc_component_driver ingenic_baic_component = {
	.name		= "aic-baic",
};

#define INGENIC_BAIC_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_U8 | \
		SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S16_BE | \
		SNDRV_PCM_FMTBIT_U16_LE | SNDRV_PCM_FMTBIT_U16_BE | \
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_BE | \
		SNDRV_PCM_FMTBIT_U24_LE | SNDRV_PCM_FMTBIT_U24_BE | \
		SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S32_BE | \
		SNDRV_PCM_FMTBIT_U32_LE | SNDRV_PCM_FMTBIT_U32_BE)

static char *dai_name[] = {
	"BAIC0",
	"BAIC1",
	"BAIC2",
	"BAIC3",
	"BAIC4",
};

static char *pname[] = {
	"BAIC0 playback",
	"BAIC1 playback",
	"BAIC2 playback",
	"BAIC3 playback",
	"BAIC4 playback",
};

static char *cname[] = {
	"BAIC0 capture",
	"BAIC1 capture",
	"BAIC2 capture",
	"BAIC3 capture",
	"BAIC4 capture",
};

static bool ingenic_baic_vailed_reg(struct device *dev, unsigned int reg)
{
	if (unlikely(reg%BAICOFF > BAICMAX))
		return false;
	return true;
}

static void ingenic_baic_set_func(struct ingenic_baic *baic, int i)
{
	u16 mode = baic->dai[i].support_mode;

	if (mode & BAIC_NO_REPLAY)
		baic->dai_driver[i].capture.stream_name = cname[i];
	else if (mode & BAIC_NO_RECORD)
		baic->dai_driver[i].playback.stream_name = pname[i];
	else {
		baic->dai_driver[i].capture.stream_name = cname[i];
		baic->dai_driver[i].playback.stream_name = pname[i];
	}
}

const char *clk_name[] = {
	"cgu_i2s0",
	"cgu_i2s1",
	"cgu_i2s2",
	"cgu_i2s3",
	"cgu_pcm",
};

const char *clk_gate_name[] = {
	"gate_i2s0",
	"gate_i2s1",
	"gate_i2s2",
	"gate_i2s3",
	"gate_pcm",
};

static int ingenic_baic_set_clk_rate(struct ingenic_baic *baic, int i, unsigned long rate)
{
	int ret = 0;
	/* According To CPM:
	*	0 - 3 use cgu_i2s0 - cgu_i2s3
	*	4 select one of cgu_i2s0 - cgu_i2s3 as is parent clk.
	**/
	baic->dai[i].clk = devm_clk_get(baic->dev, clk_name[i]);
	if(IS_ERR_OR_NULL(baic->dai[i].clk)) {
		dev_err(baic->dev, "Failed to get clk: %s\n", clk_name[i]);
		return -EINVAL;
	}

	/* Setup PCM's Parent clk. fix to I2S2 */
	if(i == 4) {
		struct clk *parent = devm_clk_get(baic->dev, clk_name[2]);
		clk_set_parent(baic->dai[i].clk, parent);
		devm_clk_put(baic->dev, parent);
	}

	baic->dai[i].clk_gate = clk_get(baic->dev, clk_gate_name[i]);
	if(IS_ERR_OR_NULL(baic->dai[i].clk_gate)) {
		dev_err(baic->dev, "Failed to get clk_gate: %s\n", clk_gate_name[i]);
		ret = -EINVAL;
		goto err_gate;
	}
	clk_set_rate(baic->dai[i].clk, rate);

	clk_prepare_enable(baic->dai[i].clk_gate);
	clk_prepare_enable(baic->dai[i].clk);

	return ret;
err_gate:
	devm_clk_put(baic->dev, baic->dai[i].clk);
	return ret;
}

static int ingenic_baic_platform_probe(struct platform_device *pdev)
{
	struct ingenic_baic *baic;
	struct resource *res;
	int ret, i;
	const __be32 *p;
	struct property *prop;
	u32 num_dais, tmp;
	struct regmap_config regmap_config = {
		.reg_bits = 32,
		.reg_stride = 4,
		.val_bits = 32,
		.writeable_reg = ingenic_baic_vailed_reg,
		.readable_reg = ingenic_baic_vailed_reg,
		.cache_type = REGCACHE_NONE,
	};

	if (of_property_read_u32(pdev->dev.of_node, "ingenic,num-dais", &num_dais))
		num_dais = 1;

	baic = devm_kzalloc(&pdev->dev, sizeof(*baic) +
			num_dais * sizeof(struct ingenic_baic_dai) +
			num_dais * sizeof(struct snd_soc_dai_driver), GFP_KERNEL);
	if (!baic)
		return -ENOMEM;

	baic->dai = (struct ingenic_baic_dai *)(baic + 1);
	baic->dai_driver = (struct snd_soc_dai_driver *)(baic->dai + num_dais);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	baic->io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(baic->io_base))
		return PTR_ERR(baic->io_base);
	regmap_config.max_register = resource_size(res) - 0x4;
	baic->regmap = devm_regmap_init_mmio(&pdev->dev,
			baic->io_base,
			&regmap_config);
	if (IS_ERR(baic->regmap))
		return PTR_ERR(baic->regmap);
	baic->dev = &pdev->dev;
	baic->num_dais = num_dais;

	i = 0;
	of_property_for_each_u32(pdev->dev.of_node, "ingenic,dai-mode", prop, p, tmp) {
		baic->dai[i++].support_mode = (u16)tmp;
		if (i >= num_dais)
			break;
	}

	i = 0;
	of_property_for_each_u32(pdev->dev.of_node, "ingenic,dai-pin-num", prop, p, tmp) {
		if ((u8)tmp <= 1)
			baic->dai[i].support_mode &= ~BAIC_DAIFMT_TDM2;
		baic->dai[i++].data_pin_num = (u8)tmp;
		if (i >= num_dais)
			break;
	}

	of_property_for_each_u32(pdev->dev.of_node, "ingenic,dai-pin-split", prop, p, tmp) {
		if (tmp >= num_dais)
			continue;
		pr_debug("split dai id(%d)\n", tmp);
		baic->dai[tmp].split_data_pin = true;
	}

	for (i = 0; i < num_dais; i++) {
		baic->dai_driver[i].id = i;
		baic->dai_driver[i].name = dai_name[i];
		baic->dai_driver[i].suspend = ingenic_baic_suspend;
		baic->dai_driver[i].resume = ingenic_baic_resume;
		baic->dai_driver[i].ops = &ingenic_baic_dai_ops;
		ingenic_baic_set_func(baic, i);
		baic->dai_driver[i].symmetric_rates =  baic->dai[i].split_data_pin ? 0 : 1;
		baic->dai_driver[i].symmetric_channels =  baic->dai[i].split_data_pin ? 0 : 1;
		baic->dai_driver[i].symmetric_samplebits =  baic->dai[i].split_data_pin ? 0 : 1;

		ingenic_baic_set_clk_rate(baic, i, 24000000);
	}

	baic->audio_clk = devm_clk_get(baic->dev, "gate_audio");
	if(IS_ERR_OR_NULL(baic->audio_clk)) {
		dev_err(baic->dev, "Failed to get gate_audio clk!\n");
	}
	clk_prepare_enable(baic->audio_clk);


	platform_set_drvdata(pdev, (void *)baic);

	ret = devm_snd_soc_register_component(&pdev->dev, &ingenic_baic_component,
			baic->dai_driver, num_dais);
	if (!ret)
		dev_info(&pdev->dev, "baic platform probe success\n");

	return ret;
}

static int ingenic_baic_platform_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id ingenic_baic_match_table[] = {
	{ .compatible = "ingenic,as-baic", },
	{ }
};
MODULE_DEVICE_TABLE(of, ingenic_baic_match_table);

#ifdef CONFIG_PM
static int ingenic_baic_runtime_suspend(struct device *dev)
{
	return 0;
}

static int ingenic_baic_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops ingenic_baic_pm_ops = {
	SET_RUNTIME_PM_OPS(ingenic_baic_runtime_suspend,
			ingenic_baic_runtime_resume, NULL)
};


static struct platform_driver ingenic_baic_platform_driver = {
	.driver = {
		.name = "as-baic",
		.of_match_table = ingenic_baic_match_table,
		.pm = &ingenic_baic_pm_ops,
	},
	.probe = ingenic_baic_platform_probe,
	.remove = ingenic_baic_platform_remove,
};
module_platform_driver(ingenic_baic_platform_driver);

MODULE_AUTHOR("cli <chen.li@ingenic.com>");
MODULE_DESCRIPTION("Ingenic AS BAIC SoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ingenic-as-baic");
