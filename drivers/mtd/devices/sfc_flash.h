#ifndef __SFC_FLASH_H
#define __SFC_FLASH_H
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/mtd/mtd.h>
#include "sfc.h"

struct sfc_flash {

	struct sfc *sfc;
	void *flash_info;
	struct device* dev;
	struct mtd_info mtd;
	struct mutex	lock;

};

struct ingenic_sfc_info {

	uint8_t num_partition;		//flash partiton len
	uint8_t use_board_info;		//use board flag
	void *flash_param;		//flash param
	void *flash_partition;		//flash partiton
	void *other_args;		//other args
};

#endif
