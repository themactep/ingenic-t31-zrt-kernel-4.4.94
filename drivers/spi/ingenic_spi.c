/* linux/drivers/spi/ingenic_spi.c
 *
 * SSI controller for SPI protocol,use FIFO and DMA;
 * base-to: linux/drivers/spi/spi_bitbang.c
 *
 * Copyright (c) 2010 Ingenic
 * Author:Shumb <sbhuang@ingenic.cn>
 *
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
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>

#include "ingenic_spi.h"

/* #define SSI_DEGUG */
#ifdef SSI_DEGUG
#define  print_dbg(format,arg...)				\
	printk(format,## arg)
#else
#define  print_dbg(format,arg...)
#endif

#define INGENIC_SPI_RX_BUF(type)							\
	u32 ingenic_spi_rx_buf_##type(struct ingenic_spi *ingspi)	\
	{														\
		u32 data  = spi_readl(ingspi, SSI_DR);					\
		type * rx = (type *)ingspi->rx;							\
		*rx++ = (type)(data);								\
		ingspi->rx = (u8 *)rx;									\
		return (u32)data;									\
	}

#define INGENIC_SPI_TX_BUF(type)							\
	u32 ingenic_spi_tx_buf_##type(struct ingenic_spi *ingspi)	\
	{														\
		u32 data;											\
		const type * tx = (type *)ingspi->tx;					\
		data = *tx++;										\
		ingspi->tx = (u8 *)tx;									\
		transmit_data(ingspi, data);							\
		return (u32)data;									\
	}

INGENIC_SPI_RX_BUF(u8)
INGENIC_SPI_TX_BUF(u8)

INGENIC_SPI_RX_BUF(u16)
INGENIC_SPI_TX_BUF(u16)

INGENIC_SPI_RX_BUF(u32)
INGENIC_SPI_TX_BUF(u32)

/* the spi->mode bits understood by this driver: */
#define MODEBITS (SPI_3WIRE | SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST | SPI_LOOP)
#define SPI_BITS_SUPPORT  (SPI_BITS_8 | SPI_BITS_16 | SPI_BITS_32)

static void ingenic_spi_cs(struct ingenic_spi_info *spi, u8 cs, unsigned int pol)
{
#ifdef CONFIG_INGENIC_SPI_PIO_CE
	u32 pin_value = *(spi->chipselect + cs);
	gpio_direction_output(pin_value, !pol ? 0 : 1);
#endif
}

static void ingenic_spi_chipsel(struct spi_device *spi, int value)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	unsigned int cspol = spi->mode & SPI_CS_HIGH ? 1 : 0;

    /*printk("%s[%d]: value = %d\n",__func__,__LINE__, value);*/
	switch (value) {
	case BITBANG_CS_INACTIVE:
		/* chip disable selected */
		if (ingspi->set_cs && ingspi->pdata)
			ingspi->set_cs(ingspi->pdata, spi->chip_select, cspol^1);
		break;
	case BITBANG_CS_ACTIVE:
		if (spi->mode & SPI_CPHA)
			set_spi_clock_phase(ingspi, 1);
		else
			set_spi_clock_phase(ingspi, 0);

		if (spi->mode & SPI_CPOL)
			set_spi_clock_polarity(ingspi, 1);
		else
			set_spi_clock_polarity(ingspi, 0);

		if (!(spi->mode & SPI_LSB_FIRST)) {
			set_tx_msb(ingspi);
			set_rx_msb(ingspi);
		} else {
			set_tx_lsb(ingspi);
			set_rx_lsb(ingspi);
		}

		if (spi->mode & SPI_LOOP)
			enable_loopback(ingspi);
		else
			disable_loopback(ingspi);

		/* chip enable selected */
		if (ingspi->set_cs && ingspi->pdata)
			ingspi->set_cs(ingspi->pdata, spi->chip_select, cspol);
		break;
	default:
		break;
	}
}

static void ingenic_spi_clk_enable(struct ingenic_spi *ingspi) {
	if(ingspi->clk_flag == 0)
		return;

	clk_set_rate(ingspi->clk_cgu, ingspi->max_clk);
	clk_prepare_enable(ingspi->clk_cgu);
	clk_prepare_enable(ingspi->clk_gate);
	ingspi->clk_flag = 0;
}

static void ingenic_spi_clk_disable(struct ingenic_spi *ingspi) {
	if(ingspi->clk_flag)
		return;

	clk_disable_unprepare(ingspi->clk_cgu);
	clk_disable_unprepare(ingspi->clk_gate);
	ingspi->clk_flag = 1;
}

static unsigned long ingenic_spi_clk_get_rate(struct spi_device *spi)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	unsigned long rate;
	u16 cgv;

	spin_lock(&ingspi->lock);
	rate = clk_get_rate(ingspi->clk_cgu);
	cgv = spi_readl(ingspi, SSI_GR);
	spin_unlock(&ingspi->lock);
	return (rate / (2 * (cgv + 1)));
}
static int ingenic_spi_clk_set_rate(struct spi_device *spi, unsigned long rate)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	unsigned long cur_rate;
	unsigned long src_rate;
	int cgv;

	cur_rate = ingenic_spi_clk_get_rate(spi);
	if(cur_rate == rate)
		return 0;
	spin_lock(&ingspi->lock);
	src_rate = clk_get_rate(ingspi->clk_cgu);
	cgv = (src_rate / (rate * 2)) - 1;
	if(cgv < 0) {
		printk("spi clk set %ld not support, src_rate = %ld\n", rate, src_rate);
		return -1;
	}
    /*printk("%s[%d]: clk_cgu = %ld, cur_rate = %d, rate = %ld, cgv = %d\n",__func__,__LINE__,src_rate, cur_rate, rate, cgv);*/
	spi_writel(ingspi, SSI_GR, cgv);
	spin_unlock(&ingspi->lock);
	return 0;
}

static void dma_tx_callback(void *data)
{
	struct ingenic_spi *ingspi = data;

	dma_unmap_sg(ingspi->txchan->device->dev, ingspi->sg_tx, 1, DMA_TO_DEVICE);
	complete(&ingspi->done_tx_dma);
}

static void dma_rx_callback(void *data)
{
	struct ingenic_spi *ingspi = data;

	dma_unmap_sg(ingspi->txchan->device->dev, ingspi->sg_tx, 1, DMA_TO_DEVICE);
	dma_unmap_sg(ingspi->rxchan->device->dev, ingspi->sg_rx, 1, DMA_FROM_DEVICE);
	complete(&ingspi->done_rx_dma);
}

/*extern void jzdma_dump(struct dma_chan *chan);*/
static int ingenic_spi_dma_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	int ret;
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	struct dma_slave_config rx_config, tx_config;
	struct dma_async_tx_descriptor *rxdesc;
	struct dma_async_tx_descriptor *txdesc;
	struct dma_chan *rxchan = ingspi->rxchan;
	struct dma_chan *txchan = ingspi->txchan;
	struct ingenic_intr_cnt *g_ingenic_intr;
	int dma_ds[] = {64, 32, 16, 4, 2, 1};
	int i;

//    printk("%s[%d]: \n",__func__,__LINE__);
	/* Check that the channels are available */
	if (!txchan || !rxchan) {
		dev_err(&spi->dev, "no dma channel\n");
		return -ENODEV;
	}

	if (t->len % ingspi->transfer_unit_size) {
		pr_err("The length of tranfer data is error\n");
		return -EFAULT;
	}

	ingspi->rw_mode = 0;
	if(t->tx_buf)
		ingspi->rw_mode |= W_MODE;
	if(t->rx_buf)
		ingspi->rw_mode |= R_MODE;

	/* all transfer starts with tx, ends with rx. */
	if (ingspi->rw_mode & W_MODE)
		ingspi->tx = t->tx_buf;
	else
		ingspi->tx = ingspi->buffer;

	if (ingspi->rw_mode & R_MODE)
		ingspi->rx = t->rx_buf;
	else
		ingspi->rx = ingspi->buffer;

	memset(ingspi->buffer, 0, BUFFER_SIZE);

	switch (ingspi->transfer_unit_size) {
	case SPI_8BITS:
		tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		tx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		rx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		rx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		tx_config.dst_maxburst = 1;
		tx_config.src_maxburst = 1;
		rx_config.src_maxburst = 1;
		rx_config.dst_maxburst = 1;
		break;
	case SPI_16BITS:
		tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		tx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		rx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		rx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		tx_config.dst_maxburst = 2;
		tx_config.src_maxburst = 2;
		rx_config.src_maxburst = 2;
		rx_config.dst_maxburst = 2;
		break;
	case SPI_32BITS:
		tx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		tx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		rx_config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		rx_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		tx_config.dst_maxburst = 4;
		tx_config.src_maxburst = 4;
		rx_config.src_maxburst = 4;
		rx_config.dst_maxburst = 4;
		break;
	}

	tx_config.dst_addr = (dma_addr_t)(ingspi->phys + SSI_DR);
	rx_config.src_addr = (dma_addr_t)(ingspi->phys + SSI_DR);

	tx_config.direction = DMA_MEM_TO_DEV;
	rx_config.direction = DMA_DEV_TO_MEM;

	dmaengine_slave_config(txchan, &tx_config);
	dmaengine_slave_config(rxchan, &rx_config);

	/* set tx dma trigger */
	for (i = 0; i < ARRAY_SIZE(dma_ds); i++) {
		if (t->len / dma_ds[i])
			break;
	}

	if (i < ARRAY_SIZE(dma_ds)) {
		ingspi->dma_tx_unit = dma_ds[i];
	} else {
		print_dbg("DMA tx block_size force to defaut set!!!");
		ingspi->dma_tx_unit = INGENIC_SSI_DMA_BURST_LENGTH;
	}

	ingspi->tx_trigger = ingspi->dma_tx_unit / (ingspi->txfifo_width >> 3);
	//set_tx_trigger(ingspi, ingspi->tx_trigger);
	set_tx_trigger(ingspi, 8); //The transfer is steady if the trigger number is used
	print_dbg("t->len: %d, tx fifo width: %d, set tx trigger value to %d\n", t->len, ingspi->txfifo_width, ingspi->tx_trigger);

	sg_init_one(ingspi->sg_tx, ingspi->tx, t->len);
	if (dma_map_sg(ingspi->txchan->device->dev,
		       ingspi->sg_tx, 1, DMA_TO_DEVICE) != 1) {
		dev_err(&spi->dev, "dma_map_sg tx error\n");
		printk("%s LINE %d: %s\n", __func__, __LINE__, __FILE__);
		goto err_tx_sgmap;
	}

	txdesc = txchan->device->device_prep_slave_sg(txchan,
				      ingspi->sg_tx,
				      1,
				      DMA_TO_DEVICE,
				      DMA_PREP_INTERRUPT | DMA_CTRL_ACK,NULL);
	if (!txdesc) {
		dev_err(&spi->dev, "device_prep_slave_sg error\n");
		printk("%s LINE %d: %s\n", __func__, __LINE__, __FILE__);
		goto err_txdesc;
	}

	// config controller
	disable_tx_intr(ingspi);
	disable_rx_intr(ingspi);

	//revisit
	disable_tx_error_intr(ingspi);
	disable_rx_error_intr(ingspi);

	start_transmit(ingspi);
	//finish_transmit(ingspi);

	flush_fifo(ingspi);

//	enable_receive(ingspi);
	clear_errors(ingspi);

	g_ingenic_intr = ingspi->g_ingenic_intr;
	memset(g_ingenic_intr, 0, sizeof *g_ingenic_intr);

	if (!(ingspi->rw_mode & R_MODE)) {
		txdesc->callback = dma_tx_callback;
		txdesc->callback_param = ingspi;
		enable_tx_error_intr(ingspi);

		dmaengine_submit(txdesc);
		dma_async_issue_pending(txchan);

		enable_receive(ingspi);
		ret = wait_for_completion_interruptible_timeout(&ingspi->done_tx_dma, 60 * HZ);
		if (ret <= 0) {
			printk("The tx_dma umap wait timeout\n");
			goto err_txdesc;
		}
		ret = wait_for_completion_interruptible_timeout(&ingspi->done, 60 * HZ);
		if (ret <= 0) {
			printk("The spi transfer wait timeout\n");
			goto err_txdesc;
		}

		if(t->cs_change)
			finish_transmit(ingspi);
		flush_rxfifo(ingspi);
		clear_errors(ingspi);

		return t->len;
	}
	/*  prepare spi dma rx */
	for (i = 0; i < ARRAY_SIZE(dma_ds); i++) {
		if (!(t->len % dma_ds[i]))
			break;
	}

	if (i < ARRAY_SIZE(dma_ds)) {
		ingspi->dma_rx_unit = dma_ds[i];
	} else {
		print_dbg("DMA rx block_size force to defaut set!!!");
		ingspi->dma_rx_unit = INGENIC_SSI_DMA_BURST_LENGTH;
	}

	ingspi->rx_trigger = ingspi->dma_rx_unit/(ingspi->rxfifo_width >> 3);
	//set_rx_trigger(ingspi, ingspi->rx_trigger);
	set_rx_trigger(ingspi, 1); //the rx trigger is steady for tranfer
	print_dbg("t->len: %d, rx fifo width: %d, set rx trigger value to %d\n", t->len, ingspi->rxfifo_width, ingspi->rx_trigger);

	sg_init_one(ingspi->sg_rx, ingspi->rx, t->len);

	if (dma_map_sg(ingspi->rxchan->device->dev,
		       ingspi->sg_rx, 1, DMA_FROM_DEVICE) != 1) {
		dev_err(&spi->dev, "dma_map_sg rx error\n");
		goto err_rx_sgmap;
	}

	rxdesc = rxchan->device->device_prep_slave_sg(rxchan,
				      ingspi->sg_rx,
				      1,
				      DMA_FROM_DEVICE,
				      DMA_PREP_INTERRUPT | DMA_CTRL_ACK,NULL);
	if (!rxdesc) {
		dev_err(&spi->dev, "device_prep_slave_sg error\n");
		goto err_rxdesc;
	}

	txdesc->callback = NULL;
	txdesc->callback_param = NULL;

	rxdesc->callback = dma_rx_callback;
	rxdesc->callback_param = ingspi;
	enable_rx_error_intr(ingspi);
	enable_tx_error_intr(ingspi);

	dmaengine_submit(txdesc);
	dmaengine_submit(rxdesc);

	dma_async_issue_pending(rxchan);
	dma_async_issue_pending(txchan);

	enable_receive(ingspi);
//	dump_spi_reg(ingspi);
	ret = wait_for_completion_interruptible_timeout(&ingspi->done_rx, 60 * HZ);
	if (ret <= 0) {
		dump_spi_reg(ingspi);
		printk("The spi receiver wait timeout\n");
		goto err_rxdesc;
	}
	/*jzdma_dump(rxchan);*/
    /*printk("%s[%d]: wait dma\n",__func__,__LINE__);*/
	ret = wait_for_completion_interruptible_timeout(&ingspi->done_rx_dma, 60 * HZ);
	if (ret <= 0) {
		dump_spi_reg(ingspi);
		printk("The spi dam_callback wait timeout\n");
		goto err_rxdesc;
	}
	finish_transmit(ingspi);
	//flush_rxfifo(ingspi);
	clear_errors(ingspi);

	return t->len;

err_rxdesc:
	dma_unmap_sg(rxchan->device->dev, ingspi->sg_rx, 1, DMA_FROM_DEVICE);
err_rx_sgmap:
err_txdesc:
	dma_unmap_sg(txchan->device->dev, ingspi->sg_tx, 1, DMA_TO_DEVICE);
err_tx_sgmap:
	printk("<< dma_txrx error. out of memory >>\n");
	return -ENOMEM;
}

static irqreturn_t ingenic_spi_dma_irq_callback(struct ingenic_spi *ingspi)
{
	struct ingenic_intr_cnt *g_ingenic_intr = ingspi->g_ingenic_intr;
	print_dbg("%s: status register: %08x\n", __func__, spi_readl(ingspi, SSI_SR));

	if (ssi_underrun(ingspi) && tx_error_intr(ingspi)) {
		print_dbg("UNDR:\n");

		g_ingenic_intr->ssi_eti++;
		disable_tx_error_intr(ingspi);

		clear_errors(ingspi);
		complete(&ingspi->done);
		complete(&ingspi->done_rx);

		goto irq_done;
	}

	if (ssi_overrun(ingspi) && rx_error_intr(ingspi)) {
			print_dbg(" overrun:\n");
			g_ingenic_intr->ssi_eri++;

			clear_errors(ingspi);
			complete(&ingspi->done);
			complete(&ingspi->done_rx);
	}

irq_done:
	return IRQ_HANDLED;
}

static inline u32 cpu_read_rxfifo(struct ingenic_spi *ingspi)
{
	u8 unit_size = ingspi->transfer_unit_size;
	u32 cnt, dat;
	int dummy_read = 0;

	print_dbg("The count of RxFIFO is %d \n", get_rxfifo_count(ingspi));
	if (get_rxfifo_count(ingspi) < 1)
		return 0;

	cnt = ingspi->rlen;
	if ((ingspi->rw_mode & RW_MODE) == W_MODE) {
		print_dbg("W_MODE\n");
		dummy_read = 1;
	}

	spin_lock(&ingspi->lock);

	while (!rxfifo_empty(ingspi)) {
		ingspi->rlen += unit_size;
		if (dummy_read)
			dat = spi_readl(ingspi, SSI_DR);
		else
			dat = ingspi->get_rx(ingspi);
	}

	spin_unlock(&ingspi->lock);

	return (ingspi->rlen - cnt);
}

static inline u32 cpu_write_txfifo(struct ingenic_spi *ingspi, u32 entries)
{
	u8 unit_size = ingspi->transfer_unit_size;
	u32 i, cnt, count;
	u32 dat;

	if ((!entries ) || (!(ingspi->rw_mode & RW_MODE)))
		return 0;

	cnt = entries;
	count = cnt * unit_size;

	spin_lock(&ingspi->lock);
	if (ingspi->rw_mode & W_MODE) {
		for (i = 0; i < cnt; i++) {
			ingspi->count += unit_size;
			dat = (u32)(ingspi->get_tx(ingspi));
		}
	} else {		 /* read, fill txfifo with 0 */
		for (i = 0; i < cnt; i++) {
			ingspi->count += unit_size;
			transmit_data(ingspi, 0);
		}
	}
	spin_unlock(&ingspi->lock);

	print_dbg("ingspi->count:%d. %s LINE %d: %s\n", ingspi->count, __func__, __LINE__, __FILE__);
	return count;
}

static int ingenic_spi_cpu_transfer(struct ingenic_spi *ingspi, long length)
{
	unsigned char int_flag = 0, last_flag = 0;
	u32 entries = 0, send_entries = 0;
	u32 unit_size, trigger;
	long leave_len_bytes;
	u32 retlen;

	print_dbg("%s LINE %d: %s\n", __func__, __LINE__, __FILE__);

	/* calculate the left entries */
	leave_len_bytes = ingspi->len - ingspi->count;

	if (ingspi->len < ingspi->count) {
		dev_err(ingspi->dev,
			"Fill data len error!!!(len : count > %d : %d)\n",
			ingspi->len, ingspi->count);
		return -1;
	}

	if (leave_len_bytes == 0) {
		print_dbg("leave_len_bytes = 0\n");
		printk("leave_len_bytes = 0\n");
		return 0;
	}

	if (ingspi->len % ingspi->transfer_unit_size) {
		pr_err("The length of tranfer data is error\n");
		return -EFAULT;
	}

	unit_size = ingspi->transfer_unit_size;
	if (unit_size == SPI_8BITS)
		entries = leave_len_bytes;
	else if (unit_size == SPI_16BITS )
		entries = leave_len_bytes >> 1;
	else if (unit_size == SPI_32BITS )
		entries = leave_len_bytes >> 2;
	else {
		dev_err(ingspi->dev,"transfer_unit_size error!\n");
		return -1;
	}
	print_dbg("%s unit_size:%d, entries:%d\n", __func__, unit_size, entries);

	/* calculate the entries which will be sent currently
	 * distinguish between the first and interrupt */
	if (ingspi->is_first) {
		/* CPU Mode should reset SSI triggers at first */
		ingspi->tx_trigger = SSI_TX_FIFO_THRESHOLD * 8;
		ingspi->rx_trigger = (SSI_RX_FIFO_THRESHOLD - SSI_SAFE_THRESHOLD) * 8;

		set_tx_trigger(ingspi, ingspi->tx_trigger);
		set_rx_trigger(ingspi, ingspi->rx_trigger);

		if(entries <= INGENIC_SSI_MAX_FIFO_ENTRIES)	{
			send_entries = entries;
		} else {
			/* need enable half_intr, left entries will be sent
			   in SSI interrupt and receive the datas */
			send_entries = INGENIC_SSI_MAX_FIFO_ENTRIES;
			int_flag = 1;
		}
		start_transmit(ingspi);

		ingspi->is_first = 0;
	} else { /* happen in interrupts */
		trigger = INGENIC_SSI_MAX_FIFO_ENTRIES - ingspi->tx_trigger;
		if (entries <= trigger) {
			send_entries = entries;
			/* the last part of data shouldn't disable RXI_intr
			   at once !!! */
			last_flag = 1;
		} else {
			/* need enable half_intr, left entries will be sent
			   in SSI interrupt and receive the datas */
			send_entries = CPU_ONCE_BLOCK_ENTRIES;
			int_flag = 1;
		}
	}

	if (length > 0) {
		length = length/ingspi->transfer_unit_size;
		if (length < send_entries)
			send_entries = length;
	}

	/* fill the txfifo with CPU Mode */
	retlen = cpu_write_txfifo(ingspi, send_entries);
	if (!retlen) {
		dev_info(ingspi->dev,"cpu_write_txfifo error!\n");
		return -1;
	}
	print_dbg("+:(%d)\n", retlen);

	enable_tx_error_intr(ingspi);
	enable_rx_error_intr(ingspi);

	/* every time should control the SSI half_intrs */
	if (int_flag) {
		enable_txfifo_half_empty_intr(ingspi);
		enable_rxfifo_half_full_intr(ingspi);
	} else {
		disable_txfifo_half_empty_intr(ingspi);
		disable_rxfifo_half_full_intr(ingspi);
	}

	/* to avoid RxFIFO overflow when CPU Mode at last time to fill */
	if (last_flag) {
		last_flag = 0;
		enable_rxfifo_half_full_intr(ingspi);
	}

#ifdef SSI_DEGUG
	dump_spi_reg(ingspi);
#endif

	return 0;
}

static int ingenic_spi_pio_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	struct ingenic_intr_cnt *g_ingenic_intr = ingspi->g_ingenic_intr;
	u32 entries;
	int status;
	unsigned long flags;

	ingspi->tx = t->tx_buf;
	ingspi->rx = t->rx_buf;
	ingspi->len = t->len;
	ingspi->count = 0;
	ingspi->rlen = 0;
	ingspi->dma_flag &= ~SPI_DMA_ACK;

	ingspi->rw_mode = 0;
	if(ingspi->tx)
		ingspi->rw_mode |= W_MODE;
	if(ingspi->rx)
		ingspi->rw_mode |= R_MODE;

	disable_tx_intr(ingspi);
	disable_rx_intr(ingspi);

	start_transmit(ingspi);
	flush_fifo(ingspi);

	enable_receive(ingspi);
	clear_errors(ingspi);

	memset(g_ingenic_intr, 0, sizeof(struct ingenic_intr_cnt));
	/* Calculate Max IRQ numbers for SSI error out */
	entries = ingspi->len * 8 / ingspi->bits_per_word;
	g_ingenic_intr->max_ssi_intr = (entries + INGENIC_SSI_MAX_FIFO_ENTRIES - 1) /
				  INGENIC_SSI_MAX_FIFO_ENTRIES * 2 + 2;

#ifdef SSI_DEGUG
	dump_spi_reg(ingspi);
#endif

	/* This start SSI transfer, write data or 0 to txFIFO.
	 * irq is locked to protect SSI config registers */
	spin_lock_irqsave(&ingspi->txrx_lock, flags);
	ingspi->is_first = 1;
	status = ingenic_spi_cpu_transfer(ingspi, 0);
	if (status < 0) {
		dev_err(ingspi->dev,"ERROR:spi_transfer error(%d)!\n", status);
		disable_tx_intr(ingspi);
		disable_rx_intr(ingspi);
		spin_unlock_irqrestore(&ingspi->txrx_lock, flags);

		return status;
	}
	spin_unlock_irqrestore(&ingspi->txrx_lock, flags);

	/* wait the interrupt finish the transfer( one spi_transfer be sent ) */
	wait_for_completion_interruptible(&ingspi->done);

	if(t->cs_change)
		finish_transmit(ingspi);
	clear_errors(ingspi);

	if (ingspi->rlen != t->len) {
		dev_info(ingspi->dev, "Length error:ingspi->rlen=%d  t->len=%d\n", ingspi->rlen,t->len);

		if(ingspi->rlen > ingspi->len)
			ingspi->rlen = ingspi->len;
	}

	return ingspi->rlen;
}

static irqreturn_t ingenic_spi_pio_irq_callback(struct ingenic_spi *ingspi)
{
	struct ingenic_intr_cnt *g_ingenic_intr = ingspi->g_ingenic_intr;
	long left_count = ingspi->len - ingspi->count;
	u8 flag = 0;
	u32 cnt;
	int status;

	g_ingenic_intr->ssi_intr_cnt++;
	/* to avoid die in interrupt if some error occur */
	if (g_ingenic_intr->ssi_intr_cnt > g_ingenic_intr->max_ssi_intr) {
		disable_tx_intr(ingspi);
		disable_rx_intr(ingspi);
		dev_err(ingspi->dev,"ssi interrupts too many count(%d)!\n",
			g_ingenic_intr->ssi_intr_cnt);

		complete(&ingspi->done);
		goto irq_done;
	}

	if ( ssi_underrun(ingspi) && tx_error_intr(ingspi) ) {
		print_dbg("UNDR:");
		g_ingenic_intr->ssi_eti++;
		disable_tx_error_intr(ingspi);

		if(left_count == 0){
			cnt = cpu_read_rxfifo(ingspi);
			print_dbg("-:(%d)\n",cnt);

			disable_tx_intr(ingspi);
			disable_rx_intr(ingspi);

			complete(&ingspi->done);
		} else {
			clear_errors(ingspi);
			enable_tx_error_intr(ingspi);
		}

		flag++;
	}

	if ( ssi_overrun(ingspi) && rx_error_intr(ingspi) ) {
		print_dbg(" overrun:");
		g_ingenic_intr->ssi_eri++;

		cnt = cpu_read_rxfifo(ingspi);
		print_dbg("-:(%d)\n",cnt);

		flag++;
	}

	if ( rxfifo_half_full(ingspi) &&
		rxfifo_half_full_intr(ingspi)) {

		print_dbg("RXI:");
		g_ingenic_intr->ssi_rxi++;

		cnt = cpu_read_rxfifo(ingspi);
		print_dbg("-:(%d)\n",cnt);

		flag++;
	}

	if ( txfifo_half_empty_intr(ingspi) &&
		txfifo_half_empty(ingspi)) {

		print_dbg("TXI:");
		g_ingenic_intr->ssi_txi++;

		status = ingenic_spi_cpu_transfer(ingspi, 0);
		if (status < 0) {
			dev_err(ingspi->dev,"ingenic_spi_cpu_transfer error!!!!!\n");
			disable_tx_intr(ingspi);
			disable_rx_intr(ingspi);
			complete(&ingspi->done);

			goto irq_done;
		}
		flag++;
	}

	if (!flag) {
		dev_info(ingspi->dev, "\nERROR:SSI interrupt Type error\n");
		complete(&ingspi->done);
	}

irq_done:
	clear_errors(ingspi);
	return IRQ_HANDLED;
}

/* every spi_transfer could call this routine to setup itself */
static int ingenic_spi_setupxfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	u8  bpw, fifo_width;
	u32 hz;
	int ret;

    /*printk("%s[%d]: \n",__func__,__LINE__);*/
	bpw = spi->bits_per_word;
	hz  = spi->max_speed_hz;

	if (t) {
		if(t->bits_per_word)
			bpw = t->bits_per_word;
		if(t->speed_hz)
			hz = t->speed_hz;
	}

	if (bpw < 2 || bpw > 32) {
		dev_err(&spi->dev, "invalid bits-per-word (%d)\n", bpw);
		return -EINVAL;
	}

	if (ingspi->use_dma) {
		ingspi->txrx_bufs = &ingenic_spi_dma_txrx;
		ingspi->irq_callback = &ingenic_spi_dma_irq_callback;
	} else {
		ingspi->txrx_bufs = &ingenic_spi_pio_txrx;
		ingspi->irq_callback = &ingenic_spi_pio_irq_callback;
	}

	ingspi->bits_per_word = bpw;
	if (bpw <= 8) {
		ingspi->transfer_unit_size = SPI_8BITS;
		ingspi->get_rx = ingenic_spi_rx_buf_u8;
		ingspi->get_tx = ingenic_spi_tx_buf_u8;
		fifo_width = FIFO_W8;
	} else if (bpw <= 16) {
		ingspi->transfer_unit_size = SPI_16BITS;
		ingspi->get_rx = ingenic_spi_rx_buf_u16;
		ingspi->get_tx = ingenic_spi_tx_buf_u16;
		fifo_width = FIFO_W16;
	} else {
		ingspi->transfer_unit_size = SPI_32BITS;
		ingspi->get_rx = ingenic_spi_rx_buf_u32;
		ingspi->get_tx = ingenic_spi_tx_buf_u32;
		fifo_width = FIFO_W32;
	}

	ingspi->txfifo_width = fifo_width;
	ingspi->rxfifo_width = fifo_width;
	set_frame_length(ingspi, fifo_width);

	if (spi->mode & SPI_LSB_FIRST) {
		set_tx_lsb(ingspi);
		set_rx_lsb(ingspi);
	} else {
		set_tx_msb(ingspi);
		set_rx_msb(ingspi);
	}

	if((ret = ingenic_spi_clk_set_rate(spi, hz)))
		return ret;

	dev_dbg(&spi->dev, "The real SPI CLK is %ld Hz\n", ingenic_spi_clk_get_rate(spi));

	mutex_lock(&ingspi->bitbang.lock);
	if (!ingspi->bitbang.busy) {
		ingspi->bitbang.chipselect(spi, BITBANG_CS_INACTIVE);
		/* need to ndelay for 0.5 clocktick ? */
	}
	mutex_unlock(&ingspi->bitbang.lock);

	return 0;
}

static int ingenic_spi_setup(struct spi_device *spi)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	unsigned long flags;
	unsigned int frmhl = 0;

	spin_lock_irqsave(&ingspi->lock, flags);
	if (ingspi->state & SUSPND) {
		spin_unlock_irqrestore(&ingspi->lock, flags);
		dev_err(&spi->dev,
			"setup: SPI-%d not active!\n", spi->master->bus_num);
		return -ESHUTDOWN;
	}
	spin_unlock_irqrestore(&ingspi->lock, flags);

	if (spi->chip_select >= spi->master->num_chipselect) {
		dev_err(&spi->dev, "cs%d >= max %d\n",
			spi->chip_select,
			spi->master->num_chipselect);
		return -EINVAL;
	}

	if (spi->chip_select == 0) {
		select_ce(ingspi);
		frmhl = spi_readl(ingspi, SSI_CR1);
		frmhl &= ~(1<<30);
		frmhl |= (spi->mode & SPI_CS_HIGH ? 1 : 0) << 30;
		spi_writel(ingspi, SSI_CR1, frmhl);
	} else if (spi->chip_select == 1) {
		select_ce2(ingspi);
		frmhl = spi_readl(ingspi, SSI_CR1);
		frmhl &= ~(1<<31);
		frmhl |= (spi->mode & SPI_CS_HIGH ? 1 : 0) << 31;
		spi_writel(ingspi, SSI_CR1, frmhl);
	} else
		return -EINVAL;

	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if (spi->mode & ~MODEBITS) {
		dev_info(&spi->dev, "Warning: unsupported mode bits %x\n",
			spi->mode & ~MODEBITS);
		return -EINVAL;
	}
	ingspi->spi_mode = spi->mode;

	if (spi->mode & SPI_LSB_FIRST) {
		set_tx_lsb(ingspi);
		set_rx_lsb(ingspi);
	} else {
		set_tx_msb(ingspi);
		set_rx_msb(ingspi);
	}

	if (spi->bits_per_word & ~SPI_BITS_SUPPORT) {
		dev_info(&spi->dev, "Warning: unsupported bits_per_word: %d\n",
			spi->bits_per_word);
		return -EINVAL;
	}

	if (!spi->max_speed_hz) {
		return -EINVAL;
	}

    /*printk("%s[%d]: ingspi->max_clk = %ld, spi->max_speed_hz = %ld\n",__func__,__LINE__,ingspi->max_clk, spi->max_speed_hz);*/
	if (ingspi->max_clk < spi->max_speed_hz) {
		dev_info(&spi->dev, "Warning:invalid clock(%d Hz) be set to source clk(%d Hz)!\n",
				 spi->max_speed_hz,(uint)ingspi->max_clk);
		spi->max_speed_hz = ingspi->max_clk;
	}

	mutex_lock(&ingspi->bitbang.lock);
	if (!ingspi->bitbang.busy) {
		ingspi->bitbang.chipselect(spi, BITBANG_CS_INACTIVE);
		/* need to ndelay for 0.5 clocktick ? */
	}
	mutex_unlock(&ingspi->bitbang.lock);

	return 0;
}

/**
 * ingenic_spi_txrx - functions which will handle transfer data
 * @spi: spi device on which data transfer to be done
 * @t: spi transfer in which transfer info is filled
 *
 * This function will put data to be transferred into data register
 * of SPI controller and then wait until the completion will be marked
 * by the IRQ Handler.
 */
static int ingenic_spi_txrx(struct spi_device * spi, struct spi_transfer *t)
{
	struct ingenic_spi *ingspi = spi_master_get_devdata(spi->master);
	unsigned int ret;
	unsigned long flags;

	spin_lock_irqsave(&ingspi->lock, flags);
	if (ingspi->state & SUSPND) {
		ingspi->state &= ~SPIBUSY;
		spin_unlock_irqrestore(&ingspi->lock, flags);
		printk("Now enter suspend, so cann't tranfer data\n");
		return -ESHUTDOWN;
	}
	ingspi->state |= SPIBUSY;
	spin_unlock_irqrestore(&ingspi->lock, flags);

	ret = ingspi->txrx_bufs(spi, t);

	spin_lock_irqsave(&ingspi->lock, flags);
	ingspi->state &= ~SPIBUSY;
	spin_unlock_irqrestore(&ingspi->lock, flags);
	return ret;
}

static irqreturn_t ingenic_spi_irq(int irq, void *dev)
{
	struct ingenic_spi *ingspi = dev;

	return ingspi->irq_callback(ingspi);
}

static int ingenic_spi_init_setup(struct ingenic_spi *ingspi)
{
	ingspi->clk_flag = 1;
	ingenic_spi_clk_enable(ingspi);

	/* disable the SSI controller */
	ssi_disable(ingspi);

	/* set default half_intr trigger */
	ingspi->tx_trigger = SSI_TX_FIFO_THRESHOLD * 8;
	ingspi->rx_trigger = SSI_RX_FIFO_THRESHOLD * 8;
	set_tx_trigger(ingspi, ingspi->tx_trigger);
	set_rx_trigger(ingspi, ingspi->rx_trigger);

	/* First,mask the interrupt, while verify the status ? */
	disable_tx_intr(ingspi);
	disable_rx_intr(ingspi);

	disable_receive(ingspi);

	set_spi_clock_phase(ingspi, 0);
	set_spi_clock_polarity(ingspi, 0);
	set_tx_msb(ingspi);
	set_rx_msb(ingspi);

	set_spi_format(ingspi);
	set_frame_length(ingspi, 8);
	disable_loopback(ingspi);
	flush_fifo(ingspi);

	underrun_auto_clear(ingspi);
	clear_errors(ingspi);
	ssi_enable(ingspi);

	return 0;
}

static struct spi_board_info board_info [] = {
	{	.modalias	= "bcm53xxspiflash",},
	/*{	.modalias	= "spidev", .mode = SPI_3WIRE | SPI_MODE_0},*/
};

#ifdef CONFIG_OF
static struct ingenic_spi_info *ingenic_spi_parse_dt(struct ingenic_spi *ingspi)
{
	struct ingenic_spi_info *isi;
	struct device *dev = ingspi->dev;
	unsigned int value;
	int i;
	isi = devm_kzalloc(dev, sizeof(*isi), GFP_KERNEL);
	if (!isi)
		return ERR_PTR(-ENOMEM);

	if(of_property_read_u32(dev->of_node, "spi-max-frequency", &value)) {
		dev_warn(dev, "spi-max-frequency not specified\n");
		isi->max_clk = 0;
	} else {
		isi->max_clk = value;
	}

    /*printk("%s[%d]: isi->max_clk = %ld\n",__func__,__LINE__, isi->max_clk);*/
	if(of_property_read_u32(dev->of_node, "ingenic,has_dma_support", &value)) {
		dev_warn(dev, "spi-max-frequency not specified\n");
		ingspi->use_dma = 0;
	} else {
		ingspi->use_dma = value;
	}

	if (of_property_read_u32(dev->of_node, "ingenic,chnl", &value)) {
		dev_warn(dev, "ingenic,channel not specified\n");
		isi->chnl = 0;
	} else {
		isi->chnl = value;
	}

    /*printk("%s[%d]: isi->chnl = %d\n",__func__,__LINE__, isi->chnl);*/
	if (of_property_read_u32(dev->of_node, "num-cs", &value)) {
		dev_warn(dev, "num_cs not specified\n");
		isi->num_chipselect = 0;
	} else {
		isi->num_chipselect = value;
	}

    /*printk("%s[%d]: isi->num_chipselect = %d\n",__func__,__LINE__, isi->num_chipselect);*/
	if (of_property_read_u32(dev->of_node, "ingenic,allow_cs_same", &value)) {
		dev_warn(dev, "ingenic,allow_cs_same not specified\n");
		isi->allow_cs_same = 0;
	} else {
		isi->allow_cs_same = value;
	}

    /*printk("%s[%d]: isi->allow_cs_same = %d\n",__func__,__LINE__, isi->allow_cs_same);*/
	if (of_property_read_u32(dev->of_node, "ingenic,bus_num", &value)) {
		dev_warn(dev, "ingenic,bus_num not specified\n");
		isi->bus_num = 0;
	} else {
		isi->bus_num = value;
	}

    /*printk("%s[%d]: isi->bus_num = %d\n",__func__,__LINE__, isi->bus_num);*/
	for (i = 0; i < isi->num_chipselect; i++) {
		int cs_gpio = of_get_named_gpio(dev->of_node, "cs-gpios", i);
		if (cs_gpio == -EPROBE_DEFER) {
			break;
		}
		isi->chipselects[i] = cs_gpio;
		/*printk("%s[%d]: cs%d is gpio = %d\n",__func__,__LINE__, i ,cs_gpio);*/
		if (gpio_is_valid(cs_gpio)) {
			if (devm_gpio_request(dev, cs_gpio, "INGENIC_SPI_CS")) {
				if(!isi->allow_cs_same)
					dev_err(dev, "could not request %d gpio\n", cs_gpio);
			} else if (gpio_direction_output(cs_gpio, 1))
				dev_err(dev, "could not set gpio %d as output\n", cs_gpio);
		}
	}

	return isi;
}
static int ingenic_spi_board_parse_dt(struct device *dev, struct ingenic_spi *ingspi)
{
	struct device_node *np = dev->of_node;
	struct device_node *child;
	struct spi_board_info *bi;
	int board_num, i;

	board_num = ARRAY_SIZE(board_info);
	for(i = 0; i < board_num; i++) {
		bi = NULL;
		for_each_child_of_node(np, child) {
			if (of_device_is_compatible(child, board_info[i].modalias)) {
				bi = &board_info[i];
				break;
			}
		}
		if(bi) {
			unsigned int tmp;

			if(of_property_read_u32(child, "spi-max-frequency", &tmp)) {
				bi->max_speed_hz = 0;
			} else {
				bi->max_speed_hz = tmp;
			}

			if (of_property_read_u32(child, "chip_select", &tmp)) {
				bi->chip_select = 0;
			} else {
				bi->chip_select = tmp;
			}
			bi->controller_data = ingspi;
			spi_new_device(ingspi->master, bi);
		}
	}

	return 0;
}

#else
static struct ingenic_spi_info *ingenic_spi_parse_dt(struct device *dev)
{
	return dev_get_platdata(dev);
}
static int ingenic_spi_board_parse_dt(struct device *dev, struct ingenic_spi *ingspi)
{
	return 0;
}
#endif

static int ingenic_spi_clk_init(struct platform_device *pdev, struct ingenic_spi *ingspi)
{
	struct clk *clk;
	char clkname[16];
	int err = 0;

	pdev->id = of_alias_get_id(pdev->dev.of_node, "spi");
	sprintf(clkname, "gate_ssi%d", pdev->id);
	ingspi->clk_gate = devm_clk_get(&pdev->dev, clkname);
	ingspi->clk_cgu = devm_clk_get(&pdev->dev, "div_ssi");

	if (IS_ERR(ingspi->clk_cgu) || IS_ERR(ingspi->clk_gate)) {
		dev_err(&pdev->dev, "Cannot get spi clock\n");
		err = PTR_ERR(ingspi->clk_cgu);
		return err;
	}
	return 0;
}

static int ingenic_spi_configure_dma(struct ingenic_spi *ingspi)
{
	struct device *dev = ingspi->dev;
	dma_cap_mask_t mask;
	int err = 0;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	ingspi->txchan = dma_request_slave_channel_reason(dev, "tx");
	if (IS_ERR(ingspi->txchan)) {
		err = PTR_ERR(ingspi->txchan);
		if (err == -EPROBE_DEFER) {
			dev_warn(dev, "no DMA channel available at the moment\n");
			return err;
		}
		dev_err(dev, "DMA TX channel not available, SPI unable to use DMA\n");
		err = -EBUSY;
		goto error;
	}

	/*
	 * No reason to check EPROBE_DEFER here since we have already requested
	 * tx channel. If it fails here, it's for another reason.
	 */
	ingspi->rxchan = dma_request_slave_channel(dev, "rx");

	if (!ingspi->rxchan) {
		dev_err(dev, "DMA RX channel not available, SPI unable to use DMA\n");
		err = -EBUSY;
		goto error;
	}

	//alloc temp buffer for dma
	ingspi->buffer = dma_alloc_coherent(dev, BUFFER_SIZE,
										&ingspi->buffer_dma, GFP_KERNEL);
	if (!ingspi->buffer) {
		dev_err(dev, "SPI request temp dma buffer failed");
		goto error;
	}

#if 0
	ingspi->buffer = kmalloc(BUFFER_SIZE, GFP_KERNEL);
	if (!ingspi->buffer) {
		dev_err(dev, "SPI request temp dma buffer failed");
		goto error;
	}
	print_dbg("<< ingspi->buffer addr:%p >>\n", ingspi->buffer);
#endif

	ingspi->sg_tx = devm_kmalloc(dev, sizeof(struct scatterlist), GFP_KERNEL);
	if (!ingspi->sg_tx) {
		dev_err(dev, "Failed to alloc tx scatterlist\n");
		goto error;
	}

	ingspi->sg_rx = devm_kmalloc(dev, sizeof(struct scatterlist), GFP_KERNEL);
	if(!ingspi->sg_rx) {
		dev_err(dev, "Failed to alloc rx scatterlist\n");
		goto error;
	}


	dev_info(dev, "Using %s (tx) and %s (rx) for DMA transfers\n",
			 dma_chan_name(ingspi->txchan),
			 dma_chan_name(ingspi->rxchan));
	return 0;
error:
	if (ingspi->rxchan)
		dma_release_channel(ingspi->rxchan);
	if (!IS_ERR(ingspi->txchan))
		dma_release_channel(ingspi->txchan);
	return err;
}

static int ingenic_spi_probe(struct platform_device *pdev)
{
	struct ingenic_spi *ingspi;
	struct spi_master *master;
	struct device_node *np = pdev->dev.of_node;
	struct ingenic_spi_info *pdata = dev_get_platdata(&pdev->dev);
	struct resource	*res;
	int err = 0;

	master = spi_alloc_master(&pdev->dev, sizeof(struct ingenic_spi));
	if (!master) {
		dev_err(&pdev->dev, "Unable to allocate SPI Master\n");
		return -ENOMEM;
	}

	/* the spi->mode bits understood by this drivers: */
	master->mode_bits = MODEBITS;

	ingspi = spi_master_get_devdata(master);
	ingspi->g_ingenic_intr = devm_kzalloc(&pdev->dev,
										  sizeof(struct ingenic_intr_cnt),GFP_KERNEL);
	if(!ingspi->g_ingenic_intr) {
		dev_err(&pdev->dev, "No memory for ingenic_intr_cnt\n");
		return -ENOMEM;
	}

	ingspi->master = spi_master_get(master);
	ingspi->dev = &pdev->dev;

	if (!pdata && np) {
		pdata = ingenic_spi_parse_dt(ingspi);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	if (!pdata) {
		dev_err(&pdev->dev, "platform_data missing!\n");
		return -ENODEV;
	}


	ingspi->pdata = pdata;
	ingspi->chnl= ingspi->pdata->chnl;
	master->bus_num = (s16)ingspi->pdata->bus_num;
	if(master->bus_num != 0 && master->bus_num != 1){
		dev_err(&pdev->dev, "No this channel, bus_num= %d.\n", master->bus_num);
		err = -ENOENT;
		goto err_no_pdata;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Unable to get SPI MEM resource\n");
		return -ENXIO;
	}
	ingspi->phys = res->start;
	ingspi->iomem = devm_ioremap_resource(&pdev->dev, res);
	if (!ingspi->iomem) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		err = -ENXIO;
		goto err_no_iomap;
	}

	ingspi->irq = platform_get_irq(pdev, 0);
	if (ingspi->irq <= 0) {
		dev_err(&pdev->dev, "No IRQ specified\n");
		err = -ENOENT;
		goto err_no_irq;
	}

	ingenic_spi_clk_init(pdev, ingspi);

	ingspi->spi_clk = ingspi->pdata->src_clk;
	ingspi->max_clk = ingspi->pdata->max_clk;

	platform_set_drvdata(pdev, ingspi);
	init_completion(&ingspi->done);
	init_completion(&ingspi->done_rx);
	init_completion(&ingspi->done_tx_dma);
	init_completion(&ingspi->done_rx_dma);
	spin_lock_init(&ingspi->lock);
	spin_lock_init(&ingspi->txrx_lock);

	master->bus_num = ingspi->pdata->bus_num;
	master->num_chipselect = ingspi->pdata->num_chipselect;

	/* setup the state for the bitbang driver */
	ingspi->bitbang.master         = ingspi->master;
	ingspi->bitbang.setup_transfer = ingenic_spi_setupxfer;
	ingspi->bitbang.chipselect     = ingenic_spi_chipsel;
	ingspi->bitbang.txrx_bufs      = ingenic_spi_txrx;
	ingspi->bitbang.master->setup  = ingenic_spi_setup;
	ingspi->fifodepth = INGENIC_SSI_MAX_FIFO_ENTRIES;
	ingspi->set_cs = &ingenic_spi_cs;

	ingenic_spi_init_setup(ingspi);

	if (ingspi->use_dma) {
		ingenic_spi_configure_dma(ingspi);
	}

	/* request SSI irq */
	err = devm_request_irq(&pdev->dev, ingspi->irq, ingenic_spi_irq, 0, pdev->name, ingspi);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto err_register;
	}

	dev_dbg(ingspi->dev, "bitbang at %p\n", &ingspi->bitbang);
	err = spi_bitbang_start(&ingspi->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master ERR_NO:%d\n",err);
		goto err_register;
	}

	err = ingenic_spi_board_parse_dt(&pdev->dev, ingspi);
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to register board info ERR_NO:%d\n",err);
		goto err_register;
	}

#ifdef CONFIG_INGENIC_SPI_BOARD_INFO_REGISTER
	/* register all the devices associated */
	bi = &ingspi->pdata->board_info[0];
	if(bi){
		for (i = 0; i < ingspi->pdata->board_size; i++, bi++) {
			dev_info(ingspi->dev, "registering %s\n", bi->modalias);

			bi->controller_data = ingspi;
			spi_new_device(master, bi);
		}
	}
#endif

	printk(KERN_INFO "INGENIC SSI Controller for SPI channel %d driver register\n",ingspi->chnl);
	return 0;

err_register:
	free_irq(ingspi->irq, ingspi);
err_no_irq:
	if(ingspi->clk_gate)
		clk_put(ingspi->clk_gate);
	if(ingspi->clk_cgu)
		clk_put(ingspi->clk_cgu);
	iounmap(ingspi->iomem);
err_no_iomap:
	release_resource(ingspi->ioarea);
	kfree(ingspi->ioarea);
#ifdef CONFIG_INGENIC_SPI_PIO_CE
err_cs_gpio:
	for (i = 0; i < num_cs_got; i++)
		gpio_free(ingspi->pdata->chipselect[i]);
#endif
err_no_pdata:
	spi_master_put(ingspi->master);

	return err;
}

static int ingenic_spi_remove(struct platform_device *dev)
{
	struct ingenic_spi *ingspi = platform_get_drvdata(dev);

	spi_master_put(ingspi->master);
	spi_bitbang_stop(&ingspi->bitbang);

	platform_set_drvdata(dev, NULL);

	free_irq(ingspi->irq, ingspi);
	iounmap(ingspi->iomem);

	ingenic_spi_clk_disable(ingspi);
	clk_put(ingspi->clk_gate);
	clk_put(ingspi->clk_cgu);

	release_resource(ingspi->ioarea);
	kfree(ingspi->ioarea);

	/* release DMA channel */
	if (ingspi->rxchan) {
		dma_release_channel(ingspi->rxchan);
	}
	if (ingspi->txchan) {
		dma_release_channel(ingspi->txchan);
	}

#ifdef CONFIG_INGENIC_SPI_PIO_CE
	/* release chipselect gpio */
	{
		int i;
		for (i = 0; i < ingspi->pdata->num_chipselect; i++)
			gpio_free(ingspi->pdata->chipselect[i]);
	}
#endif

	kfree(ingspi->g_ingenic_intr);
	kfree(ingspi);
	printk(KERN_INFO "INGENIC SSI Controller for SPI channel %d driver removed\n",ingspi->chnl);

	return 0;
}

#ifdef CONFIG_PM
static int ingenic_spi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct ingenic_spi *ingspi = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&ingspi->lock, flags);
	ingspi->state |= SUSPND;
	spin_unlock_irqrestore(&ingspi->lock, flags);

	while (ingspi->state & SPIBUSY)
		printk("Now spi is busy, waitting!\n");

	ingenic_spi_clk_disable(ingspi);

	return 0;
}

static int ingenic_spi_resume(struct platform_device *pdev)
{
	struct ingenic_spi *ingspi = platform_get_drvdata(pdev);
	unsigned long	flags;

	ingenic_spi_clk_enable(ingspi);

	spin_lock_irqsave(&ingspi->lock, flags);
	ingspi->state &= ~SUSPND;
	spin_unlock_irqrestore(&ingspi->lock, flags);

	return 0;
}

#else
#define ingenic_spi_suspend NULL
#define ingenic_spi_resume  NULL
#endif

static const struct of_device_id ingenic_spi_match[] = {
	{ .compatible = "ingenic,spi", },
	{}
};
MODULE_DEVICE_TABLE(of, ingenic_spi_match);

static struct platform_driver ingenic_spidrv = {
	.probe	    = ingenic_spi_probe,
	.remove		= ingenic_spi_remove,
	.suspend	= ingenic_spi_suspend,
	.resume		= ingenic_spi_resume,
	.driver		= {
		.name	= "ingenic-spi",
		.of_match_table	= ingenic_spi_match,
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(ingenic_spidrv);

MODULE_ALIAS("ingenic_spi");
MODULE_AUTHOR("Bo Liu <bo.liu@ingenic.com>");
MODULE_DESCRIPTION("INGENIC SPI controller driver");
MODULE_LICENSE("GPL");
