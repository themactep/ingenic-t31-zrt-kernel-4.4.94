/*
 * Copyright (c) 2012 Ingenic Semiconductor Co., Ltd.
 *              http://www.ingenic.com/
 *
 * Core file for Ingenic Display Controller driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/delay.h>
#include "mipi_csi.h"

void dump_csi_reg(void)
{
	printk("****>>>>> dump csi reg <<<<<******\n");
	printk("**********VERSION =%08x\n", csi_core_read(VERSION));
	printk("**********N_LANES =%08x\n", csi_core_read(N_LANES));
	printk("**********PHY_SHUTDOWNZ = %08x\n", csi_core_read(PHY_SHUTDOWNZ));
	printk("**********DPHY_RSTZ = %08x\n", csi_core_read(DPHY_RSTZ));
	printk("**********CSI2_RESETN =%08x\n", csi_core_read(CSI2_RESETN));
	printk("**********PHY_STATE = %08x\n", csi_core_read(PHY_STATE));
	printk("**********DATA_IDS_1 = %08x\n", csi_core_read(DATA_IDS_1));
	printk("**********DATA_IDS_2 = %08x\n", csi_core_read(DATA_IDS_2));
	printk("**********ERR1 = %08x\n", csi_core_read(ERR1));
	printk("**********ERR2 = %08x\n", csi_core_read(ERR2));
	printk("**********MASK1 =%08x\n", csi_core_read(MASK1));
	printk("**********MASK2 =%08x\n", csi_core_read(MASK2));
	printk("**********PHY_TST_CTRL0 = %08x\n", csi_core_read(PHY_TST_CTRL0));
	printk("**********PHY_TST_CTRL1 = %08x\n", csi_core_read(PHY_TST_CTRL1));
}

void check_csi_error(void) {

	unsigned int temp1, temp2;
	while(1){
		dump_csi_reg();
		temp1 = csi_core_read(ERR1);
		temp2 = csi_core_read(ERR2);
		if(temp1 != 0)
			printk("error-------- 1:0x%08x\n", temp1);
		if(temp2 != 0)
			printk("error-------- 2:0x%08x\n", temp2);
	}
}

static unsigned char csi_core_write_part(unsigned int address, unsigned int data, unsigned char shift, unsigned char width)
{
        unsigned int mask = (1 << width) - 1;
        unsigned int temp = csi_core_read(address);
        temp &= ~(mask << shift);
        temp |= (data & mask) << shift;
        csi_core_write(address, temp);

	return 0;
}

static unsigned char  csi_event_disable(unsigned int  mask, unsigned char err_reg_no)
{
	switch (err_reg_no)
	{
		case 1:
			csi_core_write(MASK1, mask | csi_core_read(MASK1));
			break;
		case 2:
			csi_core_write(MASK2, mask | csi_core_read(MASK2));
			break;
		default:
			return ERR_OUT_OF_BOUND;
	}

	return 0;
}

unsigned char csi_set_on_lanes(unsigned char lanes)
{
	csi_core_write_part(N_LANES, (lanes - 1), 0, 2);
	return 0;
}

int csi_phy_start(unsigned int id, unsigned int freq, unsigned int lans)
{
	csi_set_on_lanes(lans);

	/*reset phy*/
	csi_core_write_part(PHY_SHUTDOWNZ, 0, 0, 1);
	csi_core_write_part(PHY_SHUTDOWNZ, 1, 0, 1);
	csi_core_write_part(DPHY_RSTZ, 0, 0, 1);
	csi_core_write_part(DPHY_RSTZ, 1, 0, 1);

	csi_phy_write(LANE_EN, 0x7d);
	csi_phy_write(CLK_CON_MODE, 0x3f);
	csi_phy_write(SWITCH_LVDS_BANK, 0x3f);
	csi_phy_write(LVDS_LOGICAL_EN, 0x1e);
	csi_phy_write(SWITCH_LVDS_BANK, 0x1f);
	csi_phy_write(L0_CNT_TIME, 0x8b);
	csi_phy_write(L1_CNT_TIME, 0x8b);

	csi_core_write_part(CSI2_RESETN, 0, 0, 1);
	csi_core_write_part(CSI2_RESETN, 1, 0, 1);

	/* MASK all interrupts */
	csi_event_disable(0xffffffff, 1);
	csi_event_disable(0xffffffff, 2);

	return 0;
}

int csi_phy_stop(unsigned int id)
{

	printk("csi_phy_stop being called \n");
	/*reset phy*/
	csi_core_write_part(PHY_SHUTDOWNZ, 0, 0, 1);
	csi_core_write_part(DPHY_RSTZ, 0, 0, 1);
	csi_core_write_part(CSI2_RESETN, 0, 0, 1);

	return 0;
}

int csi_phy_init(void)
{
	//printk("csi_phy_init being called ....\n");
	return 0;
}

int csi_phy_release(void)
{
	csi_core_write_part(CSI2_RESETN, 0, 0, 1);
	csi_core_write_part(CSI2_RESETN, 1, 0, 1);
	return 0;
}
