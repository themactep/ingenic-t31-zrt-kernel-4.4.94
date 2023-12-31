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

#ifndef __INGENIC_CAMERA_H__
#define __INGENIC_CAMERA_H__

#include <linux/videodev2.h>
#include <media/soc_camera.h>
#include <linux/clk.h>
#include <linux/proc_fs.h>

#define VERSION_CODE		 KERNEL_VERSION(0, 0, 1)
#define DRIVER_NAME		 "jz-cim"
#define CAMERA_GSENSOR_VCC	"vcc_gsensor"
#define CIM_BUS_FLAGS	\
	(SOCAM_MASTER | SOCAM_VSYNC_ACTIVE_HIGH | \
	 SOCAM_VSYNC_ACTIVE_LOW | SOCAM_HSYNC_ACTIVE_HIGH | \
	 SOCAM_HSYNC_ACTIVE_LOW | SOCAM_PCLK_SAMPLE_RISING | \
	 SOCAM_PCLK_SAMPLE_FALLING | SOCAM_DATAWIDTH_8)

#define MAX_BUFFER_NUM		 (3)
#define CIM_HIST_SIZE		 (256 * 4)
#define MAX_VIDEO_MEM		 (16 * 2046 * 2046 + CIM_HIST_SIZE)
#define INGENIC_CAMERA_CSI2_2_LANE	 1
#define INGENIC_CAMERA_HSYNC_HIGH     2
#define INGENIC_CAMERA_PCLK_RISING    4
#define INGENIC_CAMERA_VSYNC_HIGH     6

/* ingenic cim ctrl id */
#define INGENIC_CID_CUSTOM_BASE		(V4L2_CID_USER_BASE | 0xf000)
#define INGENIC_CID_SCALE_SHARP		(INGENIC_CID_CUSTOM_BASE + 1)
#define INGENIC_CID_HIST_EN		(INGENIC_CID_CUSTOM_BASE + 2)
#define INGENIC_CID_HIST_GAIN_ADD	(INGENIC_CID_CUSTOM_BASE + 3)
#define INGENIC_CID_HIST_GAIN_MUL	(INGENIC_CID_CUSTOM_BASE + 4)
#define INGENIC_CID_CROP_WAY		(INGENIC_CID_CUSTOM_BASE + 5)

typedef union DES_INTC{
	unsigned int d32;
	struct{
		unsigned reserved0:1;
		unsigned EOF_MSK:1;
		unsigned SOF_MSK:1;
		unsigned reserved3_31:29;
	}data;

}DES_INTC_t;

typedef union DES_CFG{
	unsigned int d32;
	struct{
		unsigned DES_END:1;
		unsigned reserved1_15:15;
		unsigned WRBK_FMT:3;
		unsigned reserved19_25:7;
		unsigned ID:6;
	}data;

}DES_CFG_t;

typedef union DES_HIST_CFG {
	unsigned int d32;
	struct{
		unsigned int GAIN_MUL:8;
		unsigned int GAIN_ADD:8;
		unsigned int reserved16_30:15;
		unsigned int HIST_EN:1;
	};
}DES_HIST_CFG_t;

struct ingenic_camera_desc{
	dma_addr_t    next;
	unsigned int  WRBK_ADDR;
	unsigned int  WRBK_STRD;
	DES_INTC_t    DES_INTC_t;
	DES_CFG_t     DES_CFG_t;
	DES_HIST_CFG_t DES_HIST_CFG_t;
	unsigned int HIST_WRBK_ADDR;
	unsigned int SF_WRBK_ADDR;
}__attribute__ ((aligned (8)));

struct sensor_pdata {
	unsigned int gpio_rst;
	unsigned int gpio_power;
	unsigned int gpio_en;
	enum v4l2_mbus_type dat_if;
	/* senser support snapshot function */
	unsigned short snapshot;
	/* snapshot exposure pulse time, unit pixclk cycle */
	unsigned short exp_pulse_w;
	/* the time it takes to end a frame
	 * to the next frame start*/
	unsigned int delay_t;
};

struct ingenic_camera_pdata {
	unsigned long mclk_10khz;
	unsigned long flags;
	struct sensor_pdata sensor_pdata;
};

/* buffer for one video frame */
struct ingenic_buffer {
	/* common v4l buffer stuff -- must be first */
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

enum {
	IMG_NO_RESIZE,
	IMG_RESIZE_CROP,
	IMG_RESIZE_SCALE,
};

enum {
	CROP_SPECIFY_ZONE,
	CROP_BASE_MAX_PIC,
};

struct ingenic_image_sz{
	unsigned int rsz_way;
	unsigned int src_w;	//camera input size
	unsigned int src_h;

	unsigned int scl_w;	//sclae size
	unsigned int scl_h;
	unsigned int sharp;

	unsigned int crop_way;
	unsigned int c_left;	//crop size
	unsigned int c_top;
	unsigned int c_w;
	unsigned int c_h;

	struct v4l2_rect *rect_s; //enum sensor output size
	unsigned short fs_cnt;
	unsigned short fs_use;
};

struct ingenic_camera_dev {
	struct soc_camera_host soc_host;
	struct soc_camera_device *icd;
	struct ingenic_camera_pdata *pdata;

	int	sequence;
	int	start_streaming_called;
	unsigned int buf_cnt;

	struct vb2_alloc_ctx *alloc_ctx;
	struct ingenic_buffer *active;
	struct list_head video_buffer_list;

	struct resource *res;
	struct clk *clk;
	struct clk *mclk;
	struct clk *mipi_clk;
	void __iomem *base;
        struct device *dev;


	unsigned int irq;
	unsigned long mclk_freq;
	spinlock_t lock;

	void *desc_vaddr;
	struct ingenic_camera_desc *desc_paddr;
	struct ingenic_camera_desc *desc_head;
	struct ingenic_camera_desc *desc_tail;

	/* for debug */
	long long debug_ms_start;

	unsigned int interlace;
	unsigned int field;

	unsigned int hist_en;
	unsigned int hist_gain_add;
	unsigned int hist_gain_mul;

	enum v4l2_mbus_type dat_if;
};

#define GENMASK(h, l) \
	(((~0UL) << (l)) & (~0UL >> (BITS_PER_LONG - 1 - (h))))

#define	GLB_CFG					(0x0000)
#define FRM_SIZE				(0x0004)
#define CROP_SITE				(0x0008)
#define RESIZE_CFG				(0x000c)
#define RESIZE_COEF_X				(0x0010)
#define RESIZE_COEF_Y				(0x0014)
#define SCAN_CFG				(0x0018)
#define DLY_CFG					(0x001C)
#define QOS_CTRL				(0x0020)
#define QOS_CFG					(0x0024)
#define DES_ADDR				(0x1000)
#define CIM_CTRL				(0x2000)
#define CIM_ST					(0x2004)
#define CIM_CLR_ST				(0x2008)
#define CIM_INTC				(0x200c)
#define INT_FLAG				(0x2010)
#define FRAME_ID				(0x2014)
#define ACT_SIZE				(0x2018)
#define DBG_DES					(0x3000)
#define DBG_DMA					(0x3004)
#define DBG_CGC					(0x3008)

/** GLOBAL register*/
#define GLB_CFG_C_ORDER_HBIT			23
#define GLB_CFG_C_ORDER_LBIT			20
#define GLB_CFG_C_ORDER_MASK	\
	GENMASK(GLB_CFG_C_ORDER_HBIT, GLB_CFG_C_ORDER_LBIT)
#define GLB_CFG_C_ORDER_RGB			(0x0 << GLB_CFG_C_ORDER_LBIT)
#define GLB_CFG_C_ORDER_RBG			(0x1 << GLB_CFG_C_ORDER_LBIT)
#define GLB_CFG_C_ORDER_GRB			(0x2 << GLB_CFG_C_ORDER_LBIT)
#define GLB_CFG_C_ORDER_GBR			(0x3 << GLB_CFG_C_ORDER_LBIT)
#define GLB_CFG_C_ORDER_BRG			(0x4 << GLB_CFG_C_ORDER_LBIT)
#define GLB_CFG_C_ORDER_BGR			(0x5 << GLB_CFG_C_ORDER_LBIT)
#define GLB_CFG_C_ORDER_YUYV			(0x8 << GLB_CFG_C_ORDER_LBIT)
#define GLB_CFG_C_ORDER_YVYU			(0x9 << GLB_CFG_C_ORDER_LBIT)
#define GLB_CFG_C_ORDER_UYVY			(0xa << GLB_CFG_C_ORDER_LBIT)
#define GLB_CFG_C_ORDER_VYUY			(0xb << GLB_CFG_C_ORDER_LBIT)

#define GLB_CFG_ORG_FMT_HBIT			18
#define GLB_CFG_ORG_FMT_LBIT			16
#define GLB_CFG_ORG_FMT_MASK	\
	GENMASK(GLB_CFG_ORG_FMT_HBIT, GLB_CFG_ORG_FMT_LBIT)
#define GLB_CFG_ORG_FMT_RGB565			(0x0 << GLB_CFG_ORG_FMT_LBIT)
#define GLB_CFG_ORG_FMT_RGB888			(0x1 << GLB_CFG_ORG_FMT_LBIT)
#define GLB_CFG_ORG_FMT_YUYV422			(0x2 << GLB_CFG_ORG_FMT_LBIT)
#define GLB_CFG_ORG_FMT_ITU656			(0x3 << GLB_CFG_ORG_FMT_LBIT)
#define GLB_CFG_ORG_FMT_MONO			(0x4 << GLB_CFG_ORG_FMT_LBIT)

#define GLB_CFG_DE_PCLK				BIT(15)
#define GLB_CFG_DL_VSYNC			BIT(14)
#define GLB_CFG_DL_HSYNC			BIT(13)

#define GLB_CFG_EXPO_WIDTH_HBIT			10
#define GLB_CFG_EXPO_WIDTH_LBIT			8
#define	GLB_CFG_EXPO_WIDTH_MASK		\
	GENMASK(GLB_CFG_EXPO_WIDTH_HBIT, GLB_CFG_EXPO_WIDTH_LBIT)
#define GLB_CFG_SIZE_CHK			BIT(5)
#define GLB_CFG_DAT_IF_SEL			BIT(4)
#define GLB_CFG_DAT_MODE			BIT(3)

#define GLB_CFG_BURST_LEN_HBIT			2
#define GLB_CFG_BURST_LEN_LBIT			1
#define	GLB_CFG_BURST_LEN_MASK		\
	GENMASK(GLB_CFG_BURST_LEN_HBIT, GLB_CFG_BURST_LEN_LBIT)
#define	GLB_CFG_BURST_LEN_4			(0x0 << GLB_CFG_BURST_LEN_LBIT)
#define	GLB_CFG_BURST_LEN_8			(0x1 << GLB_CFG_BURST_LEN_LBIT)
#define	GLB_CFG_BURST_LEN_16			(0x2 << GLB_CFG_BURST_LEN_LBIT)
#define	GLB_CFG_BURST_LEN_32			(0x3 << GLB_CFG_BURST_LEN_LBIT)

#define GLB_CFG_AUTO_RECOVERY			BIT(0)


/**Frame size register*/
#define CIM_CROP_WIDTH_HBIT			10
#define CIM_CROP_WIDTH_LBIT			0
#define	CIM_CROP_WIDTH_MASK	\
	GENMASK(CIM_CROP_WIDTH_HBIT, CIM_CROP_WIDTH_LBIT)

#define CIM_CROP_HEIGHT_HBIT			26
#define CIM_CROP_HEIGHT_LBIT			16
#define	CIM_CROP_HEIGHT_MASK	\
	GENMASK(CIM_CROP_HEIGHT_HBIT, CIM_CROP_HEIGHT_LBIT)

/**Crop site register*/

#define CIM_CROP_X_HBIT				10
#define CIM_CROP_X_LBIT				0
#define CIM_CROP_X_MASK		\
	GENMASK(CIM_CROP_X_HBIT, CIM_CROP_X_LBIT)

#define CIM_CROP_Y_HBIT				26
#define CIM_CROP_Y_LBIT				16
#define CIM_CROP_Y_MASK		\
	GENMASK(CIM_CROP_Y_HBIT, CIM_CROP_Y_LBIT)

/** Scale size register*/
#define CIM_TAR_WIDTH_HBIT			10
#define CIM_TAR_WIDTH_LBIT			0
#define CIM_TAR_WIDTH_MASK	\
	GENMASK(CIM_TAR_WIDTH_HBIT, CIM_TAR_WIDTH_LBIT)

#define CIM_TAR_HEIGHT_HBIT			26
#define CIM_TAR_HEIGHT_LBIT			16
#define CIM_TAR_HEIGHT_MASK	\
	GENMASK(CIM_TAR_HEIGHT_HBIT, CIM_TAR_HEIGHT_LBIT)

#define CIM_SHARPL_HBIT				15
#define CIM_SHARPL_LBIT				14
#define CIM_SHARPL_MASK		\
	GENMASK(CIM_SHARPL_HBIT, CIM_SHARPL_LBIT)
#define CIM_LOWST_SHARPL			(0x0 << CIM_SHARPL_LBIT)
#define CIM_HIGST_SHARPL			(0x3 << CIM_SHARPL_LBIT)

#define CIM_SCALE_EN				BIT(31)

/**Resize Coef X register*/
#define CIM_RESIZE_COEF_X_HBIT			19
#define CIM_RESIZE_COEF_X_LBIT			0
#define CIM_RESIZE_COEF_X_MASK		\
	GENMASK(CIM_RESIZE_COEF_X_HBIT, CIM_RESIZE_COEF_X_LBIT)

/**Resize Coef Y register*/

#define CIM_RESIZE_COEF_Y_HBIT			19
#define CIM_RESIZE_COEF_Y_LBIT			0
#define CIM_RESIZE_COEF_Y_MASK		\
	GENMASK(CIM_RESIZE_COEF_Y_HBIT, CIM_RESIZE_COEF_Y_LBIT)

/**Second Field Height register*/
#define CIM_SF_HEIGHT_HBIT			11
#define CIM_SF_HEIGHT_LBIT			0
#define CIM_SF_HEIGHT_MASK	\
	GENMASK(CIM_SF_HEIGHT_HBIT, CIM_SF_HEIGHT_LBIT)

#define CIM_F_ORDER				BIT(30)
#define CIM_SCAN_MD				BIT(31)

/**Delay counter register*/
#define CIM_DLY_NUM_HBIT			23
#define CIM_DLY_NUM_LBIT			0
#define CIM_DLY_NUM_MASK	\
	GENMASK(CIM_DLY_NUM_HBIT, CIM_DLY_NUM_LBIT)

#define CIM_DLY_MD				BIT(30)
#define CIM_DLY_EN				BIT(31)

#define CIM_QOS_CTRL				BIT(0)
#define CIM_QOS_VAL_HBIT			2
#define CIM_QOS_VAL_LBIT			1
#define CIM_QOS_VAL_MASK	\
	GENMASK(CIM_QOS_VAL_HBIT, CIM_QOS_VAL_LBIT)

#define CIM_QOS_CFG_TH2_LBIT			(26)
#define CIM_QOS_CFG_TH2_HBIT			(18)
#define CIM_QOS_CFG_TH2_MASK	\
	GENMASK(CIM_QOS_CFG_TH2_HBIT, CIM_QOS_CFG_TH2_LBIT)
#define CIM_QOS_CFG_TH1_LBIT			(17)
#define CIM_QOS_CFG_TH1_HBIT			(9)
#define CIM_QOS_CFG_TH1_MASK	\
	GENMASK(CIM_QOS_CFG_TH1_HBIT, CIM_QOS_CFG_TH1_LBIT)
#define CIM_QOS_CFG_TH0_LBIT			(0)
#define CIM_QOS_CFG_TH0_HBIT			(8)
#define CIM_QOS_CFG_TH0_MASK	\
	GENMASK(CIM_QOS_CFG_TH0_HBIT, CIM_QOS_CFG_TH0_LBIT)

/**Descriptor address register*/
#define CIM_DES_ADDR_HBIT			31
#define CIM_DES_ADDR_LBIT			0
#define CIM_DES_ADDR_MASK	\
	GENMASK(CIM_DES_ADDR_HBIT, CIM_DES_ADDR_LBIT)

/**Control register*/
#define CIM_CTRL_START				BIT(0)
#define CIM_CTRL_QCK_STOP			BIT(1)
#define CIM_CTRL_GEN_STOP			BIT(2)
#define CIM_CTRL_SOFT_RST			BIT(3)
#define CIM_CTRL_DBG_DES_RST			BIT(4)

/**status register*/
#define CIM_ST_WORKING				BIT(0)
#define CIM_ST_EOF				BIT(1)
#define CIM_ST_SOF				BIT(2)
#define CIM_ST_GSA				BIT(3)
#define CIM_ST_OVER				BIT(4)
#define CIM_ST_EOW				BIT(5)
#define CIM_ST_SZ_ERR				BIT(6)
#define CIM_ST_SRST				BIT(7)

/**Clear status register*/
#define CIM_CLR_FRM_END				BIT(1)
#define CIM_CLR_FRM_START			BIT(2)
#define CIM_CLR_GSA				BIT(3)
#define CIM_CLR_OVER				BIT(4)
#define CIM_CLR_EOW				BIT(5)
#define CIM_CLR_SZ_ERR				BIT(6)
#define CIM_CLR_SRST				BIT(7)
#define CIM_CLR_ALL	\
	(CIM_CLR_FRM_END | CIM_CLR_FRM_START | \
	 CIM_CLR_GSA | CIM_CLR_OVER | \
	 CIM_CLR_EOW | CIM_CLR_SZ_ERR)

/**CIM INTC register*/
#define CIM_INTC_MSK_EOF			BIT(1)
#define CIM_INTC_MSK_SOF			BIT(2)
#define CIM_INTC_MSK_GSA			BIT(3)
#define CIM_INTC_MSK_OVER			BIT(4)
#define CIM_INTC_MSK_EOW			BIT(5)
#define CIM_INTC_MSK_SZ_ERR			BIT(6)

/**CIM_INT_FLAG*/
#define CIM_INT_FLAG_EOF			BIT(1)
#define CIM_INT_FLAG_SOF			BIT(2)
#define CIM_INT_FLAG_GSA			BIT(3)
#define CIM_INT_FLAG_OVER			BIT(4)
#define CIM_INT_FLAG_EOW			BIT(5)
#define CIM_INT_FLAG_SZ_ERR			BIT(6)

#define CIM_CGC_FLOW				BIT(0)
#define CIM_CGC_DVP				BIT(1)
#define CIM_CGC_SCALE				BIT(2)
#define CIM_CGC_DES				BIT(3)
#define CIM_CGC_WRBK				BIT(4)
#define CIM_CGC_REG				BIT(5)

/**dma Write back format*/
#define DES_CFG_WRBK_FMT_HBIT			18
#define DES_CFG_WRBK_FMT_LBIT			16
#define DES_CFG_WRBK_FMT_MASK	\
	GENMASK(DES_CFG_WRBK_FMT_HBIT, DES_CFG_WRBK_FMT_LBIT)

#define WRBK_FMT_RGB888				(6)
#define WRBK_FMT_MONO				(4)
#define WRBK_FMT_YUV422				(3)
#define WRBK_FMT_RGB565				(1)


#define CIM_HSIT_EN				BIT(31)
#define CIM_GAIN_ADD_HBIT			15
#define CIM_GAIN_ADD_LBIT			8
#define CIM_GAIN_ADD_MASK	\
	GENMASK(CIM_GAIN_ADD_HBIT, CIM_GAIN_ADD_LBIT)
#define CIM_GAIN_MUL_HBIT			7
#define CIM_GAIN_MUL_LBIT			0
#define CIM_GAIN_MUL_MASK	\
	GENMASK(CIM_GAIN_MUL_HBIT, CIM_GAIN_MUL_LBIT)

static inline void cim_writel(struct ingenic_camera_dev *pcdev,
			unsigned int val, unsigned int off)
{
	writel(val, pcdev->base + off);
}

static inline unsigned int cim_readl(struct ingenic_camera_dev *pcdev,
			unsigned int off)
{
	return readl(pcdev->base + off);
}

#endif
