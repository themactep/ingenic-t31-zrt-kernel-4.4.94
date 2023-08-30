SRC := $(shell pwd)

ifeq ($(CONFIG_SOC_T31),y)
SOC := t31
else ifeq ($(CONFIG_SOC_T41),y)
SOC := t41
else
error "No SOC"
endif

obj-$(CONFIG_INGENIC_SENSOR_GC2053)        += $(SOC)/gc2053/gc2053.o

ccflags-y := -DRELEASE -DUSER_BIT_32 -DKERNEL_BIT_32 -Wno-date-time -D_GNU_SOURCE

# TODO: this is ugly, I know
ccflags-y += -I$(PWD)/drivers/video/ingenic_isp/$(SOC)/include

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) O=$(O) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean
