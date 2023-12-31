/*
 * SFC controller for SPI protocol, use FIFO and DMA;
 *
 * Copyright (c) 2015 Ingenic
 * Author: <xiaoyang.fu@ingenic.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/page.h>
#include "sfc_flash.h"
#include "spinor.h"
#include "ingenic_sfc_common.h"

//#define DEBUG_CLONER_PARAMS

#define STATUS_SUSPND	(1<<0)

static struct spi_nor_cmd_info read_standard_table = {
	.cmd = SPINOR_OP_READ,
	.dummy_byte = 0,
	.addr_nbyte = 3,
	.transfer_mode = 0,
};
static struct spi_nor_cmd_info read_quad_table = {
	.cmd = SPINOR_OP_READ_1_1_4,
	.dummy_byte = 8,
	.addr_nbyte = 3,
	.transfer_mode = 5,
};
static struct spi_nor_cmd_info write_standard_table = {
	.cmd = SPINOR_OP_PP,
	.dummy_byte = 0,
	.addr_nbyte = 3,
	.transfer_mode = 0,
};
static struct spi_nor_cmd_info write_quad_table = {
	.cmd = SPINOR_OP_QPP,
	.dummy_byte = 0,
	.addr_nbyte = 3,
	.transfer_mode = 5,
};
static struct spi_nor_cmd_info sector_erase_table = {
	.cmd = SPINOR_OP_BE_32K,
	.dummy_byte = 0,
	.addr_nbyte = 3,
	.transfer_mode = 0,
};
static struct spi_nor_cmd_info wr_en_table = {
	.cmd = SPINOR_OP_WREN,
	.dummy_byte = 0,
	.addr_nbyte = 0,
	.transfer_mode = 0,
};
static struct spi_nor_cmd_info en4byte_table = {
	.cmd = SPINOR_OP_EN4B,
	.dummy_byte = 0,
	.addr_nbyte = 0,
	.transfer_mode = 0,
};
static struct spi_nor_st_info quad_set_table = {
	.cmd = SPINOR_OP_WRSR_1,
	.bit_shift = 1,
	.mask = 1,
	.val = 1,
	.len = 1,
	.dummy = 0,
};
static struct spi_nor_st_info quad_get_table = {
	.cmd = SPINOR_OP_RDSR_1,
	.bit_shift = 1,
	.mask = 1,
	.val = 1,
	.len = 1,
	.dummy = 0,
};
static struct spi_nor_st_info busy_table = {
	.cmd = SPINOR_OP_RDSR,
	.bit_shift = 0,
	.mask = 1,
	.val = 0,
	.len = 1,
	.dummy = 0,
};

static struct burner_params *burner_params;
struct sfc_flash *to_ingenic_spi_norflash(struct mtd_info *mtd_info)
{
	return container_of(mtd_info, struct sfc_flash, mtd);
}

int32_t sfc_nor_reset(struct sfc_flash *flash)
{
	struct sfc_cdt_xfer xfer;
	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	xfer.cmd_index = NOR_RESET_ENABLE;

	/* set addr */
	xfer.rowaddr = 0;
	xfer.columnaddr = 0;

	/* set transfer config */
	xfer.dataen = DISABLE;

	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	udelay(100);
	return 0;
}

int sfc_nor_read_id(struct sfc_flash *flash)
{
	struct sfc_cdt_xfer xfer;
	unsigned char buf[3];
	unsigned int chip_id = 0;
	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	xfer.cmd_index = NOR_READ_ID;

	/* set addr */
	xfer.rowaddr = 0;
	xfer.columnaddr = 0;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = 3;
	xfer.config.data_dir = GLB0_TRAN_DIR_READ;
	xfer.config.ops_mode = CPU_OPS;
	xfer.config.buf = buf;

	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	if(xfer.config.ops_mode == DMA_OPS)
		dma_cache_sync(NULL, (void *)xfer.config.buf, xfer.config.datalen, DMA_FROM_DEVICE);

	chip_id = ((buf[0] & 0xff) << 16) | ((buf[1] & 0xff) << 8) | (buf[2] & 0xff);

	return chip_id;
}

static unsigned int sfc_do_read(struct sfc_flash *flash, unsigned int addr, unsigned char *buf, size_t len)
{
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct sfc_cdt_xfer xfer;
	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	if (nor_info->quad_succeed) {
		xfer.cmd_index = NOR_READ_QUAD;
	} else {
		xfer.cmd_index = NOR_READ_STANDARD;
	}

	/* set addr */
	xfer.columnaddr = 0;
	xfer.rowaddr = addr;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = len;
	xfer.config.data_dir = GLB0_TRAN_DIR_READ;
	xfer.config.ops_mode = DMA_OPS;
	xfer.config.buf = buf;

	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	if(xfer.config.ops_mode == DMA_OPS)
		dma_cache_sync(NULL, (void *)buf, len, DMA_FROM_DEVICE);

	return len;
}

static unsigned  int sfc_do_write(struct sfc_flash *flash, unsigned int addr, const unsigned char *buf, size_t len)
{
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct sfc_cdt_xfer xfer;
	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	if (nor_info->quad_succeed) {
		xfer.cmd_index = NOR_WRITE_QUAD_ENABLE;
	} else {
		xfer.cmd_index = NOR_WRITE_STANDARD_ENABLE;
	}

	/* set addr */
	xfer.columnaddr = 0;
	xfer.rowaddr = addr;

	/* set transfer config */
	xfer.dataen = ENABLE;
	xfer.config.datalen = len;
	xfer.config.data_dir = GLB0_TRAN_DIR_WRITE;
	xfer.config.ops_mode = DMA_OPS;
	xfer.config.buf = (uint8_t *)buf;

	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	return len;
}

static int sfc_do_erase(struct sfc_flash *flash, uint32_t addr)
{
	struct sfc_cdt_xfer xfer;

	memset(&xfer, 0, sizeof(xfer));

	/* set Index */
	xfer.cmd_index = NOR_ERASE_WRITE_ENABLE;

	/* set addr */
	xfer.rowaddr = addr;

	/* set transfer config */
	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	return 0;
}

static int sfc_read(struct sfc_flash *flash, loff_t from, size_t len, unsigned char *buf)
{
	int32_t ret;

	//memset(flash->sfc->desc, 0, sizeof(struct sfc_desc) * DESC_MAX_NUM);  //1024 bytes space

	/* create DMA Descriptors */
	ret = create_sfc_desc(flash, buf, len);
	if(ret < 0){
		dev_err(flash->dev, "%s create descriptors error. -%d\n", __func__, ret);
		return ret;
	}

	//dump_desc(flash->sfc, ret);

	/* DMA Descriptors read */
	ret = sfc_do_read(flash, (unsigned int)from, buf, len);

	return ret;
}

static int sfc_write(struct sfc_flash *flash, loff_t to, size_t len, const unsigned char *buf)
{
	int32_t ret;

	//memset(flash->sfc->desc, 0, sizeof(struct sfc_desc) * DESC_MAX_NUM);  //1024 bytes space

	/* create DMA Descriptors */
	ret = create_sfc_desc(flash, (unsigned char *)buf, len);
	if(ret < 0){
		dev_err(flash->dev, "%s create descriptors error. -%d\n", __func__, ret);
		return ret;
	}

	//dump_desc(flash->sfc, ret);

	/* DMA Descriptors write */
	ret = sfc_do_write(flash, (unsigned int)to, buf, len);

	return ret;
}

static int ingenic_spi_norflash_read(struct mtd_info *mtd, loff_t from, size_t len,size_t *retlen, unsigned char *buf)
{
	struct sfc_flash *flash = to_ingenic_spi_norflash(mtd);

	mutex_lock(&flash->lock);
	*retlen = sfc_read(flash, from, len, buf);
	mutex_unlock(&flash->lock);

	return 0;
}

static int ingenic_spi_norflash_write(struct mtd_info *mtd, loff_t to, size_t len,
		size_t *retlen, const unsigned char *buf)
{
	u32 page_offset, actual_len;
	struct sfc_flash *flash = to_ingenic_spi_norflash(mtd);
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct spi_nor_info *spi_nor_info = nor_info->nor_flash_info;
	int ret;

	mutex_lock(&flash->lock);

	page_offset = to & (spi_nor_info->page_size - 1);
	/* do all the bytes fit onto one page? */
	if (page_offset + len <= spi_nor_info->page_size) {
		ret = sfc_write(flash, (unsigned int)to, len, buf);
		*retlen = ret;
	} else {
		u32 i;

		/* the size of data remaining on the first page */
		actual_len = spi_nor_info->page_size - page_offset;
		ret = sfc_write(flash, (unsigned int)to, actual_len, buf);
		*retlen += ret;

		/* write everything in flash->page_size chunks */
		for (i = actual_len; i < len; i += mtd->writesize) {
			actual_len = len - i;
			if (actual_len >= mtd->writesize)
				actual_len = mtd->writesize;

			ret = sfc_write(flash, (unsigned int)to + i, actual_len, buf + i);
			*retlen += ret;
		}
	}
	mutex_unlock(&flash->lock);
	return 0;
}

static int ingenic_spi_norflash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct sfc_flash *flash = to_ingenic_spi_norflash(mtd);
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct spi_nor_info *spi_nor_info = nor_info->nor_flash_info;
	uint32_t addr, end;
	int ret;

	mutex_lock(&flash->lock);

	addr = (instr->addr & (mtd->erasesize - 1));
	if (addr) {
		dev_err(flash->dev, "%s eraseaddr no align\n", __func__);
		mutex_unlock(&flash->lock);
		return -EINVAL;
	}
	end = (instr->len & (mtd->erasesize - 1));
	if (end) {
		dev_err(flash->dev,"%s erasesize no align\n", __func__);
		mutex_unlock(&flash->lock);
		return -EINVAL;
	}
	addr = (uint32_t)instr->addr;
	end = addr + (uint32_t)instr->len;

	while (addr < end) {
		ret = sfc_do_erase(flash, addr);
		if (ret) {
			dev_err(flash->dev,"erase error !\n");
			mutex_unlock(&flash->lock);
			instr->state = MTD_ERASE_FAILED;
			return ret;
		}
		addr += spi_nor_info->erase_size;
	}
	mutex_unlock(&flash->lock);
	instr->state = MTD_ERASE_DONE;

	mtd_erase_callback(instr);
	return 0;
}
#if 0
static int32_t ingenic_spi_norflash_read_params(struct sfc_flash *flash, loff_t from, size_t len, uint8_t *buf)
{
	struct sfc_cdt_xfer xfer;
	memset(&xfer, 0, sizeof(xfer));

	xfer.cmd_index = NOR_READ_STANDARD;

	xfer.columnaddr = 0;
	xfer.rowaddr = from;

	xfer.dataen =  ENABLE;
	xfer.config.datalen = len;
	xfer.config.data_dir = GLB0_TRAN_DIR_READ;
	xfer.config.ops_mode = CPU_OPS;
	xfer.config.buf = buf;

	if(sfc_sync_cdt(flash->sfc, &xfer)) {
		dev_err(flash->dev,"sfc_sync_cdt error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		return -EIO;
	}

	return 0;
}
#endif
#ifdef DEBUG_CLONER_PARAMS
static void dump_cloner_params(struct burner_params *params)
{
	struct spi_nor_info *spi_nor_info;

	spi_nor_info = &params->spi_nor_info;

	printk("name=%s\n", spi_nor_info->name);
	printk("id=0x%x\n", spi_nor_info->id);

	printk("read_standard->cmd=0x%x\n",		spi_nor_info->read_standard.cmd);
	printk("read_standard->dummy=0x%x\n",		spi_nor_info->read_standard.dummy_byte);
	printk("read_standard->addr_nbyte=0x%x\n",	spi_nor_info->read_standard.addr_nbyte);
	printk("read_standard->transfer_mode=0x%x\n",	spi_nor_info->read_standard.transfer_mode);

	printk("read_quad->cmd=0x%x\n",			spi_nor_info->read_quad.cmd);
	printk("read_quad->dummy=0x%x\n",		spi_nor_info->read_quad.dummy_byte);
	printk("read_quad->addr_nbyte=0x%x\n",		spi_nor_info->read_quad.addr_nbyte);
	printk("read_quad->transfer_mode=0x%x\n",	spi_nor_info->read_quad.transfer_mode);

	printk("write_standard->cmd=0x%x\n",		spi_nor_info->write_standard.cmd);
	printk("write_standard->dummy=0x%x\n",		spi_nor_info->write_standard.dummy_byte);
	printk("write_standard->addr_nbyte=0x%x\n",	spi_nor_info->write_standard.addr_nbyte);
	printk("write_standard->transfer_mode=0x%x\n",	spi_nor_info->write_standard.transfer_mode);

	printk("write_quad->cmd=0x%x\n",		spi_nor_info->write_quad.cmd);
	printk("write_quad->dummy=0x%x\n",		spi_nor_info->write_quad.dummy_byte);
	printk("write_quad->addr_nbyte=0x%x\n",		spi_nor_info->write_quad.addr_nbyte);
	printk("write_quad->transfer_mode=0x%x\n",	spi_nor_info->write_quad.transfer_mode);

	printk("sector_erase->cmd=0x%x\n",		spi_nor_info->sector_erase.cmd);
	printk("sector_erase->dummy=0x%x\n",		spi_nor_info->sector_erase.dummy_byte);
	printk("sector_erase->addr_nbyte=0x%x\n",	spi_nor_info->sector_erase.addr_nbyte);
	printk("sector_erase->transfer_mode=0x%x\n",	spi_nor_info->sector_erase.transfer_mode);

	printk("wr_en->cmd=0x%x\n",		spi_nor_info->wr_en.cmd);
	printk("wr_en->dummy=0x%x\n",		spi_nor_info->wr_en.dummy_byte);
	printk("wr_en->addr_nbyte=0x%x\n",	spi_nor_info->wr_en.addr_nbyte);
	printk("wr_en->transfer_mode=0x%x\n",	spi_nor_info->wr_en.transfer_mode);

	printk("en4byte->cmd=0x%x\n",		spi_nor_info->en4byte.cmd);
	printk("en4byte->dummy=0x%x\n",		spi_nor_info->en4byte.dummy_byte);
	printk("en4byte->addr_nbyte=0x%x\n",	spi_nor_info->en4byte.addr_nbyte);
	printk("en4byte->transfer_mode=0x%x\n",	spi_nor_info->en4byte.transfer_mode);

	printk("quad_set->cmd=0x%x\n",		spi_nor_info->quad_set.cmd);
	printk("quad_set->bit_shift=0x%x\n",		spi_nor_info->quad_set.bit_shift);
	printk("quad_set->mask=0x%x\n",		spi_nor_info->quad_set.mask);
	printk("quad_set->val=0x%x\n",		spi_nor_info->quad_set.val);
	printk("quad_set->len=0x%x\n",		spi_nor_info->quad_set.len);
	printk("quad_set->dummy=0x%x\n",	spi_nor_info->quad_set.dummy);

	printk("quad_get->cmd=0x%x\n",		spi_nor_info->quad_get.cmd);
	printk("quad_get->bit_shift=0x%x\n",		spi_nor_info->quad_get.bit_shift);
	printk("quad_get->mask=0x%x\n",		spi_nor_info->quad_get.mask);
	printk("quad_get->val=0x%x\n",		spi_nor_info->quad_get.val);
	printk("quad_get->len=0x%x\n",		spi_nor_info->quad_get.len);
	printk("quad_get->dummy=0x%x\n",	spi_nor_info->quad_get.dummy);

	printk("busy->cmd=0x%x\n",		spi_nor_info->busy.cmd);
	printk("busy->bit_shift=0x%x\n",		spi_nor_info->busy.bit_shift);
	printk("busy->mask=0x%x\n",		spi_nor_info->busy.mask);
	printk("busy->val=0x%x\n",		spi_nor_info->busy.val);
	printk("busy->len=0x%x\n",		spi_nor_info->busy.len);
	printk("busy->dummy=0x%x\n",		spi_nor_info->busy.dummy);

	printk("quad_ops_mode=%d\n",	spi_nor_info->quad_ops_mode);
	printk("addr_ops_mode=%d\n",	spi_nor_info->addr_ops_mode);

	printk("tCHSH=%d\n",	spi_nor_info->tCHSH);
	printk("tSLCH=%d\n",	spi_nor_info->tSLCH);
	printk("tSHSL_RD=%d\n", spi_nor_info->tSHSL_RD);
	printk("tSHSL_WR=%d\n", spi_nor_info->tSHSL_WR);

	printk("chip_size=%d\n",	spi_nor_info->chip_size);
	printk("page_size=%d\n",	spi_nor_info->page_size);
	printk("erase_size=%d\n",	spi_nor_info->erase_size);

	printk("chip_erase_cmd=0x%x\n",	spi_nor_info->chip_erase_cmd);
}
#endif

static int ingenic_spi_norflash_get_params(struct sfc_flash *flash, struct ingenic_sfc_info *pdata)
{
	int i = 0;
	int32_t err = 0;
	unsigned int chip_id;
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct ingenic_sfc_info *ingenic_sfc_info = pdata;

	burner_params = kzalloc(sizeof(struct burner_params), GFP_KERNEL);
	if (!burner_params) {
		dev_err(flash->dev, "Failed to alloc mem for params\n");
		err = -ENOMEM;
		goto err_params;
	}

	chip_id = sfc_nor_read_id(flash);
	burner_params->spi_nor_info_size = ARRAY_SIZE(spi_nor_info_table);

	for(i = 0; i < burner_params->spi_nor_info_size; i++)
	{
		if(chip_id == spi_nor_info_table[i].id)
		{
			if(spi_nor_info_table[i].addr_len == 4)
			{
				read_standard_table.addr_nbyte = 4;
				read_quad_table.addr_nbyte = 4;
				write_standard_table.addr_nbyte = 4;
				write_quad_table.addr_nbyte = 4;
				sector_erase_table.addr_nbyte =4;
			}
			quad_set_table.cmd = spi_nor_info_table[i].quad_set.cmd;
			quad_set_table.bit_shift = spi_nor_info_table[i].quad_set.bit_shift;
			quad_get_table.cmd = spi_nor_info_table[i].quad_get.cmd;
			quad_get_table.bit_shift = spi_nor_info_table[i].quad_get.bit_shift;
			spi_nor_info_table[i].read_standard		= read_standard_table;
			spi_nor_info_table[i].read_quad			= read_quad_table;
			spi_nor_info_table[i].write_standard	= write_standard_table;
			spi_nor_info_table[i].write_quad		= write_quad_table;
			spi_nor_info_table[i].sector_erase		= sector_erase_table;
			spi_nor_info_table[i].wr_en				= wr_en_table;
			spi_nor_info_table[i].en4byte			= en4byte_table;
			spi_nor_info_table[i].quad_set			= quad_set_table;
			spi_nor_info_table[i].quad_get			= quad_get_table;
			spi_nor_info_table[i].busy				= busy_table;
			burner_params->spi_nor_info				= spi_nor_info_table[i];
			printk("the id code = %x, the flash name is %s\n", chip_id, spi_nor_info_table[i].name);
			break;
		}
	}

	if(i == burner_params->spi_nor_info_size)
	{
		if((chip_id != 0)&&(chip_id != 0xff))
		{
			printk("ingenic: Unsupported ID %x\n", chip_id);
			return EINVAL;
		}else{
			printk("read or write error! Unsupported ID %x\n", chip_id);
			return EINVAL;
		}
	}

#if 0
	ret = jz_spi_norflash_read_params(flash, SPIFLASH_PARAMER_OFFSET, sizeof(struct burner_params), (uint8_t *)burner_params);
	if (ret) {
		dev_err(flash->dev, "Failed to read params (burned by Burner)\n");
		err = -EINVAL;
		goto err_read_params;
	}
#endif

	//add crc check for params
	nor_info->nor_flash_info = NULL;
	nor_info->nor_flash_info = &burner_params->spi_nor_info;
#if 0
	nor_info->norflash_partitions = &burner_params->norflash_partitions;
	nor_info->nor_pri_data = &burner_params->nor_pri_data;
#endif
	if (ingenic_sfc_info && ingenic_sfc_info->use_board_info) {
		printk("use_board_info is set!!!\n");
		if (ingenic_sfc_info->flash_param)
			nor_info->nor_flash_info = ingenic_sfc_info->flash_param;
		if (ingenic_sfc_info->flash_partition) {
			nor_info->norflash_partitions->num_partition_info = ingenic_sfc_info->num_partition;
			memcpy(nor_info->norflash_partitions->nor_partition, ingenic_sfc_info->flash_partition, sizeof(struct nor_partition) * ingenic_sfc_info->num_partition);
		}
	}

	if ((!nor_info->nor_flash_info)) {
		printk("WARNING : cannot get nor flash params !!!\n");
		err = -EINVAL;
		goto err_read_params;
	}

#ifdef DEBUG_CLONER_PARAMS
	dump_cloner_params(burner_params);
#endif
	return 0;
err_read_params:
	kfree(burner_params);
err_params:
	return err;
}
static ssize_t sfc_nor_partition_offset_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf,"0x%x\n", SPIFLASH_PARAMER_OFFSET + sizeof(int) * 2 + sizeof(struct spi_nor_info));
}

static DEVICE_ATTR(sfc_nor_partition_offset, S_IRUGO | S_IWUSR,
		sfc_nor_partition_offset_show,
		NULL);

static ssize_t sfc_nor_params_offset_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf,"0x%x\n",SPIFLASH_PARAMER_OFFSET);
}

static DEVICE_ATTR(sfc_nor_params_offset, S_IRUGO | S_IWUSR,
		sfc_nor_params_offset_show,
		NULL);

/*add your attr in here*/
static struct attribute *sfc_norflash_info_attributes[] = {
	&dev_attr_sfc_nor_partition_offset.attr,
	&dev_attr_sfc_nor_params_offset.attr,
	NULL
};

static const struct attribute_group sfc_norflash_info_attr_group = {
	.attrs = sfc_norflash_info_attributes
};

/*
 *MK_CMD(cdt, cmd, LINK, ADDRMODE, DATA_EN)
 *MK_ST(cdt, st, LINK, ADDRMODE, ADDR_WIDTH, POLL_EN, DATA_EN, TRAN_MODE)
 */
static void params_to_cdt(struct spi_nor_info *params, struct sfc_cdt *cdt)
{
	/* 4.nor singleRead */
	MK_CMD(cdt[NOR_READ_STANDARD], params->read_standard, 0, ROW_ADDR, ENABLE);

	/* 5.nor quadRead */
	MK_CMD(cdt[NOR_READ_QUAD], params->read_quad, 0, ROW_ADDR, ENABLE);

	/* 6. nor writeStandard */
	MK_CMD(cdt[NOR_WRITE_STANDARD_ENABLE], params->wr_en, 1, DEFAULT_ADDRMODE, DISABLE);
	MK_CMD(cdt[NOR_WRITE_STANDARD], params->write_standard, 1, ROW_ADDR, ENABLE);
	MK_ST(cdt[NOR_WRITE_STANDARD_FINISH], params->busy, 0, DEFAULT_ADDRMODE, 0, ENABLE, DISABLE, TM_STD_SPI);

	/* 7. nor writeQuad */
	MK_CMD(cdt[NOR_WRITE_QUAD_ENABLE], params->wr_en, 1, DEFAULT_ADDRMODE, DISABLE);
	MK_CMD(cdt[NOR_WRITE_QUAD], params->write_quad, 1, ROW_ADDR, ENABLE);
	MK_ST(cdt[NOR_WRITE_QUAD_FINISH], params->busy, 0, DEFAULT_ADDRMODE, 0, ENABLE, DISABLE, TM_STD_SPI);

	/* 8. nor erase */
	MK_CMD(cdt[NOR_ERASE_WRITE_ENABLE], params->wr_en, 1, DEFAULT_ADDRMODE, DISABLE);
	MK_CMD(cdt[NOR_ERASE], params->sector_erase, 1, ROW_ADDR, DISABLE);
	MK_ST(cdt[NOR_ERASE_FINISH], params->busy, 0, DEFAULT_ADDRMODE, 0, ENABLE, DISABLE, TM_STD_SPI);

	/* 9. quad mode */
	if(params->quad_ops_mode){
		MK_CMD(cdt[NOR_QUAD_SET_ENABLE], params->wr_en, 1, DEFAULT_ADDRMODE, DISABLE);
		MK_ST(cdt[NOR_QUAD_SET], params->quad_set, 1, DEFAULT_ADDRMODE, 0, DISABLE, ENABLE, TM_STD_SPI);  //disable poll, enable data

		MK_ST(cdt[NOR_QUAD_FINISH], params->busy, 1, DEFAULT_ADDRMODE, 0, ENABLE, DISABLE, TM_STD_SPI);

		MK_ST(cdt[NOR_QUAD_GET], params->quad_get, 0, DEFAULT_ADDRMODE, 0, ENABLE, DISABLE, TM_STD_SPI);
	}

	/* 10. nor write ENABLE */
	MK_CMD(cdt[NOR_WRITE_ENABLE], params->wr_en, 0, DEFAULT_ADDRMODE, DISABLE);

	/* 11. entry 4byte mode */
	MK_CMD(cdt[NOR_EN_4BYTE], params->en4byte, 0, DEFAULT_ADDRMODE, DISABLE);

}

static void create_cdt_table(struct sfc_flash *flash, uint32_t flag)
{
	struct spinor_flashinfo *nor_info = flash->flash_info;
	struct spi_nor_info *nor_flash_info;
	struct sfc_cdt sfc_cdt[INDEX_MAX_NUM];

	memset(sfc_cdt, 0, sizeof(sfc_cdt));

	/* 1.nor reset */
	sfc_cdt[NOR_RESET_ENABLE].link = CMD_LINK(1, DEFAULT_ADDRMODE, TM_STD_SPI);
	sfc_cdt[NOR_RESET_ENABLE].xfer = CMD_XFER(0, DISABLE, 0, DISABLE, SPINOR_OP_RSTEN);
	sfc_cdt[NOR_RESET_ENABLE].staExp = 0;
	sfc_cdt[NOR_RESET_ENABLE].staMsk = 0;

	sfc_cdt[NOR_RESET].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
	sfc_cdt[NOR_RESET].xfer = CMD_XFER(0, DISABLE, 0, DISABLE, SPINOR_OP_RST);
	sfc_cdt[NOR_RESET].staExp = 0;
	sfc_cdt[NOR_RESET].staMsk = 0;


	/* 2.nor read id */
	sfc_cdt[NOR_READ_ID].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
	sfc_cdt[NOR_READ_ID].xfer = CMD_XFER(0, DISABLE, 0, ENABLE, SPINOR_OP_RDID);
	sfc_cdt[NOR_READ_ID].staExp = 0;
	sfc_cdt[NOR_READ_ID].staMsk = 0;


	/* 3. nor get status */
	sfc_cdt[NOR_GET_STATUS].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
	sfc_cdt[NOR_GET_STATUS].xfer = CMD_XFER(0, DISABLE, 0, ENABLE, SPINOR_OP_RDSR);
	sfc_cdt[NOR_GET_STATUS].staExp = 0;
	sfc_cdt[NOR_GET_STATUS].staMsk = 0;

	sfc_cdt[NOR_GET_STATUS_1].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
	sfc_cdt[NOR_GET_STATUS_1].xfer = CMD_XFER(0, DISABLE, 0, ENABLE, SPINOR_OP_RDSR_1);
	sfc_cdt[NOR_GET_STATUS_1].staExp = 0;
	sfc_cdt[NOR_GET_STATUS_1].staMsk = 0;

	sfc_cdt[NOR_GET_STATUS_2].link = CMD_LINK(0, DEFAULT_ADDRMODE, TM_STD_SPI);
	sfc_cdt[NOR_GET_STATUS_2].xfer = CMD_XFER(0, DISABLE, 0, ENABLE, SPINOR_OP_RDSR_2);
	sfc_cdt[NOR_GET_STATUS_2].staExp = 0;
	sfc_cdt[NOR_GET_STATUS_2].staMsk = 0;

	if(flag == DEFAULT_CDT){
		/* 4.nor singleRead */
		sfc_cdt[NOR_READ_STANDARD].link = CMD_LINK(0, ROW_ADDR, TM_STD_SPI);
		sfc_cdt[NOR_READ_STANDARD].xfer = CMD_XFER(DEFAULT_ADDRSIZE, DISABLE, 0, ENABLE, SPINOR_OP_READ);
		sfc_cdt[NOR_READ_STANDARD].staExp = 0;
		sfc_cdt[NOR_READ_STANDARD].staMsk = 0;

		/* first create cdt table */
		write_cdt(flash->sfc, sfc_cdt, NOR_RESET_ENABLE, NOR_READ_STANDARD);
	}


	if(flag == UPDATE_CDT){
		nor_flash_info = nor_info->nor_flash_info;
		params_to_cdt(nor_flash_info, sfc_cdt);

		/* second create cdt table */
		write_cdt(flash->sfc, sfc_cdt, NOR_READ_STANDARD, NOR_EN_4BYTE);
	}
	//dump_cdt(flash->sfc);
}

static int request_sfc_desc(struct sfc_flash *flash)
{
	struct sfc *sfc = flash->sfc;
	sfc->desc = (struct sfc_desc *)dma_alloc_coherent(flash->dev, sizeof(struct sfc_desc) * DESC_MAX_NUM, &sfc->desc_pyaddr, GFP_KERNEL);
	if(flash->sfc->desc == NULL){
		return -ENOMEM;
	}
	sfc->desc_max_num = DESC_MAX_NUM;

	return 0;
}

static int __init ingenic_sfc_probe(struct platform_device *pdev)
{
	struct sfc_flash *flash;
	int err = 0,ret = 0;
	struct spinor_flashinfo *nor_info;
	struct ingenic_sfc_info *pdata_params;
	int tchsh;
	int tslch;
	int tshsl_rd;
	int tshsl_wr;

	flash = kzalloc(sizeof(struct sfc_flash), GFP_KERNEL);
	if (flash == NULL) {
		dev_err(&pdev->dev, "Failed to alloc mem for flash\n");
		return -ENOMEM;
	}

	nor_info = kzalloc(sizeof(*nor_info), GFP_KERNEL);
	if(!nor_info) {
		dev_err(&pdev->dev, "alloc nor_info failed!\n");
		kfree(flash);
		return -ENOMEM;
	}
	flash->flash_info = nor_info;

	flash->dev = &pdev->dev;

	flash->sfc = sfc_res_init(pdev);
	if(IS_ERR_OR_NULL(flash->sfc)) {
		ret = -ENOMEM;
		goto err_sfc_res_init;
	}

	platform_set_drvdata(pdev, flash);
	mutex_init(&flash->lock);

	set_flash_timing(flash->sfc, DEF_TCHSH, DEF_TSLCH, DEF_TSHSL_R, DEF_TSHSL_W);

	/* request DMA Descriptor space */
	ret = request_sfc_desc(flash);
	if(ret){
		dev_err(flash->dev, "Failure to request DMA descriptor space!\n");
		ret = -ENOMEM;
		goto err_sfc_desc_request;
	}

	/* try creating default CDT table */
	create_cdt_table(flash, DEFAULT_CDT);

	ret = sfc_nor_reset(flash);
	if(ret) {
		dev_warn(flash->dev, "Failed to reset nor flash, Try to go on\n");
	}

	pdata_params = pdev->dev.platform_data;
	ret = ingenic_spi_norflash_get_params(flash, pdata_params);
	if (ret) {
		ret = -ENODEV;
		dev_err(flash->dev, "Failed to match correct nor flash device!\n");
		goto err_match_device;
	}

	/* Update to private CDT table */
	create_cdt_table(flash, UPDATE_CDT);

	flash->mtd.name     = "jz_sfc";
	flash->mtd.owner    = THIS_MODULE;
	flash->mtd.type     = MTD_NORFLASH;
	flash->mtd.flags    = MTD_CAP_NORFLASH;
	flash->mtd.erasesize    = nor_info->nor_flash_info->erase_size;
	flash->mtd.writesize    = nor_info->nor_flash_info->page_size;
	flash->mtd.size     = nor_info->nor_flash_info->chip_size;
	flash->mtd._erase   = ingenic_spi_norflash_erase;
	flash->mtd._read    = ingenic_spi_norflash_read;
	flash->mtd._write   = ingenic_spi_norflash_write;

	tchsh = nor_info->nor_flash_info->tCHSH;
	tslch = nor_info->nor_flash_info->tSLCH;
	tshsl_rd = nor_info->nor_flash_info->tSHSL_RD;
	tshsl_wr = nor_info->nor_flash_info->tSHSL_WR;
	set_flash_timing(flash->sfc, tchsh, tslch, tshsl_rd, tshsl_wr);

	sfc_nor_get_special_ops(flash);

#ifdef CONFIG_SPI_STANDARD_MODE
	nor_info->quad_succeed = 0;
	dev_info(&pdev->dev, "nor flash now use standard mode!\n");
#else
	ret = nor_info->nor_flash_ops->set_quad_mode(flash);
	if (ret < 0) {
		nor_info->quad_succeed = 0;
		dev_info(&pdev->dev, "set quad mode error !\n");
	} else {
		nor_info->quad_succeed = 1;
		dev_info(&pdev->dev, "nor flash quad mode is set, now use quad mode!\n");
	}
#endif

	/* if nor flash size is greater than 16M, use 4byte mode */
	if(flash->mtd.size > NOR_SIZE_16M) {
		if (nor_info->nor_flash_ops->set_4byte_mode) {
			nor_info->nor_flash_ops->set_4byte_mode(flash);
		}
	}

	ret = mtd_device_parse_register(&flash->mtd, NULL, NULL, NULL, 0);
	if (ret) {
		ret = -ENODEV;
		dev_err(flash->dev, "Failed to parse register!\n");
		goto err_parse_register;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &sfc_norflash_info_attr_group);
	if (err) {
		dev_err(&pdev->dev, "failed to register sysfs\n");
		ret = -EIO;
		goto err_create_group;
	}

	dev_info(&pdev->dev,"SPI NOR MTD LOAD OK\n");
	return 0;

err_create_group:
	mtd_device_unregister(&flash->mtd);
err_parse_register:
	kfree(burner_params);
err_match_device:
err_sfc_desc_request:
	dma_free_coherent(flash->dev, sizeof(struct sfc_desc) * DESC_MAX_NUM, flash->sfc->desc, flash->sfc->desc_pyaddr);
err_sfc_res_init:
	kfree(nor_info);
	kfree(flash);
	return ret;

}

static int __exit ingenic_sfc_remove(struct platform_device *pdev)
{
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;

	clk_disable_unprepare(sfc->clk_gate);
	clk_put(sfc->clk_gate);

	clk_disable_unprepare(sfc->clk);
	clk_put(sfc->clk);

	free_irq(sfc->irq, flash);
	iounmap(sfc->iomem);
	release_mem_region(sfc->ioarea->start, resource_size(sfc->ioarea));

	platform_set_drvdata(pdev, NULL);
	sysfs_remove_group(&pdev->dev.kobj, &sfc_norflash_info_attr_group);
	return 0;
}

static int ingenic_sfc_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;
#ifdef CONFIG_ENABLE_DEEP_POWER_DOWN
	{
		struct sfc_transfer transfer;
		int ret;

		memset(&transfer, 0, sizeof(transfer));
		sfc_list_init(&transfer);

		transfer.cmd_info.cmd = CMD_DP;
		transfer.cmd_info.dataen = DISABLE;
		ret = sfc_sync(sfc, &transfer);
		if(ret) {
			dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		}
		udelay(10);
	}
#endif

	disable_irq(sfc->irq);
	clk_disable_unprepare(sfc->clk_gate);
	clk_disable_unprepare(sfc->clk);

	return 0;
}

static int ingenic_sfc_resume(struct platform_device *pdev)
{
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;

	clk_prepare_enable(sfc->clk);
	clk_prepare_enable(sfc->clk_gate);

	enable_irq(sfc->irq);
#ifdef CONFIG_ENABLE_DEEP_POWER_DOWN
	{
		struct sfc_transfer transfer;
		int ret;

		memset(&transfer, 0, sizeof(transfer));
		sfc_list_init(&transfer);

		transfer.cmd_info.cmd = CMD_RDP;
		transfer.cmd_info.dataen = DISABLE;
		ret = sfc_sync(sfc, &transfer);
		if(ret) {
			dev_err(flash->dev,"sfc_sync error ! %s %s %d\n",__FILE__,__func__,__LINE__);
		}
		udelay(60);
	}
#endif
	return 0;
}

void ingenic_sfc_shutdown(struct platform_device *pdev)
{
	struct sfc_flash *flash = platform_get_drvdata(pdev);
	struct sfc *sfc = flash->sfc;

	disable_irq(sfc->irq);
	clk_disable_unprepare(sfc->clk_gate);
	clk_disable_unprepare(sfc->clk);
	return ;
}

static const struct of_device_id ingenicsfc_match[] = {
	{ .compatible = "ingenic,sfc", },
	{},
};
MODULE_DEVICE_TABLE(of, ingenicsfc_match);

static struct platform_driver ingenic_sfcdrv = {
	.driver		= {
		.name	= "ingenic-sfc",
		.owner	= THIS_MODULE,
		.of_match_table = ingenicsfc_match,
	},
	.remove		= __exit_p(ingenic_sfc_remove),
	.suspend	= ingenic_sfc_suspend,
	.resume		= ingenic_sfc_resume,
//	.shutdown	= ingenic_sfc_shutdown,
};
module_platform_driver_probe(ingenic_sfcdrv, ingenic_sfc_probe);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("INGENIC SFC Driver");
