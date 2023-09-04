/*
 * sound/soc/ingenic/icodec/icdc_d3.h
 * ALSA SoC Audio driver -- ingenic internal codec (icdc_d3) driver

 * Copyright 2015 Ingenic Semiconductor Co.,Ltd
 *	cli <chen.li@ingenic.com>
 *
 * Note: icdc_d3 is an internal codec for ingenic SOC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __ICDC_D3_REG_H__
#define __ICDC_D3_REG_H__

#include <linux/spinlock.h>
#include <sound/soc.h>
#include "../as-v1/asoc-aic.h"

struct internal_t10 {
	struct device		*dev;		/*aic device used to access register*/
	struct snd_soc_codec	*codec;
	spinlock_t       io_lock;		/*codec hw io lock,
							  Note codec cannot opt in irq context*/
	void * __iomem mapped_base;				/*vir addr*/
	resource_size_t mapped_resstart;				/*resource phy addr*/
	resource_size_t mapped_ressize;				/*resource phy addr size*/

	int dac_user_mute;				/*dac user mute state*/
	/*aohp power on anti pop event*/
	volatile int aohp_in_pwsq;				/*aohp in power up/down seq*/
	int hpl_wished_gain;				/*keep original hpl/r gain register value*/
	int hpr_wished_gain;
	int linl_wished_gain;				/*keep original hpl/r gain register value*/
	int linr_wished_gain;
	/*codec irq*/
	int irqno;
	int irqflags;
	int codec_imr;
	struct work_struct irq_work;
	/*headphone detect*/
	struct snd_soc_jack *jack;
	int report_mask;
};

/*
 * Note: icdc_d3 codec just only support detect headphone jack
 * detected_type: detect event treat as detected_type
 *	 example: SND_JACK_HEADSET detect event treat as SND_JACK_HEADSET
 */
int icdc_d3_hp_detect(struct snd_soc_codec *codec,
		struct snd_soc_jack *jack, int detected_type);

#define TS_CODEC_CGR_00			0x00
#define TS_CODEC_CACR_02		0x08
#if (!defined(CONFIG_SOC_T31) && !defined(CONFIG_SOC_C100))
#define TS_CODEC_CMCR_03		0x0c
#define TS_CODEC_CDCR1_04		0x10
#define TS_CODEC_CDCR2_05		0x14
#define TS_CODEC_CADR_07		0x1c
#define TS_CODEC_CGAINR_0a		0x28
#define TS_CODEC_CDPR_0e		0x38
#define TS_CODEC_CDDPR2_0f		0x3c
#define TS_CODEC_CDDPR1_10		0x40
#define TS_CODEC_CDDPR0_11		0x44
#define TS_CODEC_CAACR_21		0x84
#define TS_CODEC_CMICCR_22		0x88
#define TS_CODEC_CACR2_23		0x8c
#define TS_CODEC_CAMPCR_24		0x90
#define TS_CODEC_CAR_25			0x94
#define TS_CODEC_CHR_26			0x98
#define TS_CODEC_CHCR_27		0x9c
#define TS_CODEC_CCR_28			0xa0
#else
#define TS_CODEC_CACR2_03       0x0c
#define TS_CODEC_CDCR1_04       0x10
#define TS_CODEC_CDCR2_05       0x14
#define TS_CODEC_CAVR_08        0x20
#define TS_CODEC_CGAINLR_09     0x24
#define TS_CODEC_POWER_20       0x80
#define TS_CODEC_CCR_21         0x84
#define TS_CODEC_CAACR_22       0x88
#define TS_CODEC_CMICCR_23      0x8c
#define TS_CODEC_CMICGAINR_24   0x90
#define TS_CODEC_CALCGR_25      0x94
#define TS_CODEC_CANACR_26      0x98
#define TS_CODEC_CANACR2_27     0x9c
#define TS_CODEC_CHR_28         0xA0
#endif
#define TS_CODEC_CMR_40			0x100
#define TS_CODEC_CTR_41			0x104
#define TS_CODEC_CAGCCR_42		0x108
#define TS_CODEC_CPGR_43		0x10c
#define TS_CODEC_CSRR_44		0x110
#define TS_CODEC_CALMR_45		0x114
#define TS_CODEC_CAHMR_46		0x118
#define TS_CODEC_CALMINR_47		0x11c
#define TS_CODEC_CAHMINR_48		0x120
#define TS_CODEC_CAFG_49		0x124
#define TS_CODEC_CCAGVR_4c		0x130

#define ADC_VALID_DATA_LEN_16BIT	0x0
#define ADC_VALID_DATA_LEN_20BIT	0x1
#define ADC_VALID_DATA_LEN_24BIT	0x2
#define ADC_VALID_DATA_LEN_32BIT	0x3

#define ADC_I2S_INTERFACE_RJ_MODE	0x0
#define ADC_I2S_INTERFACE_LJ_MODE	0x1
#define ADC_I2S_INTERFACE_I2S_MODE	0x2
#define ADC_I2S_INTERFACE_PCM_MODE	0x3

#define CHOOSE_ADC_CHN_STEREO		0x0
#define CHOOSE_ADC_CHN_MONO			0x1

#define SAMPLE_RATE_8K		0x7
#define SAMPLE_RATE_12K		0x6
#define SAMPLE_RATE_16K		0x5
#define SAMPLE_RATE_24K		0x4
#define SAMPLE_RATE_32K		0x3
#define SAMPLE_RATE_44_1K	0x2
#define SAMPLE_RATE_48K		0x1
#define SAMPLE_RATE_96K		0x0


#if 0
static inline void icdc_d3_mapped_reg_set(void __iomem * xreg, int xmask, int xval)
{
	int val = readl(xreg);
	val &= ~(xmask);
	val |= xval;
	writel(val, xreg);
}

static inline int icdc_d3_mapped_test_bits(void __iomem * xreg, int xmask, int xval)
{
	int val = readl(xreg);
	val &= xmask;
	return (val == xval);
}


#define icdc_d3_test_rw_inval(icdc_d3)      \
	icdc_d3_mapped_test_bits((icdc_d3->mapped_base + RGADW), SCODA_RGWR_MASK, (1 << SCODA_RGWR_BIT))
/*
 * RGDATA
 */
#define SCODA_RGDOUT_BIT		(0)
#define SCODA_RGDOUT_MASK		(0xff << SCODA_RGDOUT_BIT)
#define SCODA_IRQ_BIT		(8)
#define SCODA_IRQ_MASK		(0x1  << SCODA_IRQ_BIT)

#define icdc_d3_test_irq(icdc_d3)	\
	icdc_d3_mapped_test_bits((icdc_d3->mapped_base + RGDATA),	\
			SCODA_IRQ_MASK, (1 << SCODA_IRQ_BIT))

static inline u8 icdc_d3_hw_read_normal(struct icdc_d3 *icdc_d3, int reg)
{
	void __iomem * mapped_base = icdc_d3->mapped_base;
	int reval;
	int timeout = 0xfffff;
	unsigned long flags;

	spin_lock_irqsave(&icdc_d3->io_lock, flags);

	while(icdc_d3_test_rw_inval(icdc_d3)) {
		timeout--;
		if (!timeout) pr_err("icdc_d3 test_rw_inval timeout\n");
	}

	icdc_d3_mapped_reg_set((mapped_base + RGADW), SCODA_RGWR_MASK,(0 << SCODA_RGWR_BIT));

	icdc_d3_mapped_reg_set((mapped_base + RGADW), SCODA_RGADDR_MASK,(reg << SCODA_RGADDR_BIT));

	reval = readl((mapped_base + RGDATA));
	reval = readl((mapped_base + RGDATA));
	reval = readl((mapped_base + RGDATA));
	reval = readl((mapped_base + RGDATA));
	reval = readl((mapped_base + RGDATA));
	reval = ((reval & SCODA_RGDOUT_MASK) >> SCODA_RGDOUT_BIT);
//	printk("reg %x = %x\n", reg, reval);
	spin_unlock_irqrestore(&icdc_d3->io_lock, flags);
	return (u8) reval;
}

static inline int icdc_d3_hw_write_normal(struct icdc_d3 *icdc_d3, int reg, int data)
{
	void __iomem * mapped_base = icdc_d3->mapped_base;
	int ret = 0;
	int timeout = 0xfffff;
	unsigned long flags;

	spin_lock_irqsave(&icdc_d3->io_lock, flags);

	while(icdc_d3_test_rw_inval(icdc_d3)) {
		timeout--;
		if (!timeout) pr_err("icdc_d3 test_rw_inval timeout\n");
	}
	icdc_d3_mapped_reg_set((mapped_base + RGADW),SCODA_RGDIN_MASK|SCODA_RGADDR_MASK,
			(data << SCODA_RGDIN_BIT)|(reg << SCODA_RGADDR_BIT));
	icdc_d3_mapped_reg_set((mapped_base + RGADW), SCODA_RGWR_MASK , 1 << SCODA_RGWR_BIT);
	spin_unlock_irqrestore(&icdc_d3->io_lock, flags);
	if( reg != SCODA_REG_IFR && reg != SCODA_REG_IFR2 ){
		ret = icdc_d3_hw_read_normal(icdc_d3, reg);
		if (data != ret){
			printk("icdc write reg %x err exp %x now is %x\n",reg,data,ret);
			ret = -1;
		}
	}
	return ret;
}

static int icdc_d3_hw_write_extend(struct icdc_d3 *icdc_d3, u8 sreg, u8 sdata){
	int creg, cdata, dreg;
	switch (sreg) {
		case SCODA_MIX_0 ... SCODA_MIX_4:
			creg = SCODA_REG_CR_MIX;
			dreg = SCODA_REG_DR_MIX;
			sreg -= SCODA_MIX_0;
			break;
		case SCODA_DAC_AGC0 ... SCODA_DAC_AGC3:
			creg = SCODA_REG_CR_DAC_AGC;
			dreg = SCODA_REG_DR_DAC_AGC;
			sreg -= SCODA_DAC_AGC0;
			break;
		case SCODA_DAC2_AGC0 ... SCODA_DAC2_AGC3:
			creg = SCODA_REG_CR_DAC2_AGC;
			dreg = SCODA_REG_DR_DAC2_AGC;
			sreg -= SCODA_DAC2_AGC0;
			break;
		case SCODA_ADC_AGC0 ... SCODA_ADC_AGC4:
			creg = SCODA_REG_CR_ADC_AGC;
			dreg = SCODA_REG_DR_ADC_AGC;
			sreg -= SCODA_ADC_AGC0;
			break;
		default:
			return 0;
	}
	printk("write extend : sreg: %d [0 - 4], creg: %x sdata: %d\n", sreg, creg, sdata);

	cdata = (icdc_d3_hw_read_normal(icdc_d3,creg)&(~0x3f))|((sreg&0x3f)|0x40);

	icdc_d3_hw_write_normal(icdc_d3, creg, cdata);
	icdc_d3_hw_write_normal(icdc_d3, dreg, sdata);
	if(sdata!=icdc_d3_hw_read_normal(icdc_d3,dreg))
		return -1;
	return 0;
}


static u8 icdc_d3_hw_read_extend(struct icdc_d3 *icdc_d3, u8 sreg)
{
	int creg, cdata, dreg, ddata;
	switch (sreg) {

		case SCODA_MIX_0 ... SCODA_MIX_4:
			creg = SCODA_REG_CR_MIX;
			dreg = SCODA_REG_DR_MIX;
			sreg -= SCODA_MIX_0;
			break;
		case SCODA_DAC_AGC0 ... SCODA_DAC_AGC3:
			creg = SCODA_REG_CR_DAC_AGC;
			dreg = SCODA_REG_DR_DAC_AGC;
			sreg -= SCODA_DAC_AGC0;
			break;
		case SCODA_DAC2_AGC0 ... SCODA_DAC2_AGC3:
			creg = SCODA_REG_CR_DAC2_AGC;
			dreg = SCODA_REG_DR_DAC2_AGC;
			sreg -= SCODA_DAC2_AGC0;
			break;
		case SCODA_ADC_AGC0 ... SCODA_ADC_AGC4:
			creg = SCODA_REG_CR_ADC_AGC;
			dreg = SCODA_REG_DR_ADC_AGC;
			sreg -= SCODA_ADC_AGC0;
			break;
		default:
			return 0;
	}
	cdata = (icdc_d3_hw_read_normal(icdc_d3,creg)&(~0x7f))|(sreg&0x3f);
	icdc_d3_hw_write_normal(icdc_d3, creg, cdata);
	ddata = icdc_d3_hw_read_normal(icdc_d3, dreg);
	return (u8) ddata;
}


static inline u8 icdc_d3_hw_read(struct icdc_d3 *icdc_d3, int reg)
{
	if (reg > SCODA_REG_SR_TR_SRCDAC)
		return icdc_d3_hw_read_extend(icdc_d3, reg);
	else
		return icdc_d3_hw_read_normal(icdc_d3, reg);
}
static inline int icdc_d3_hw_write(struct icdc_d3 *icdc_d3, int reg, int data)
{
	if (reg > SCODA_REG_SR_TR_SRCDAC){
		return icdc_d3_hw_write_extend(icdc_d3, reg, data);
	} else {
		return icdc_d3_hw_write_normal(icdc_d3, reg, data);
	}
}
#endif

#endif	/* __ICDC_D3_REG_H__ */
