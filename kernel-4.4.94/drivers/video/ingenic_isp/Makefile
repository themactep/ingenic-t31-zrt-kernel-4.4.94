SRC := $(shell pwd)

ifeq ($(CONFIG_SOC_T31),y)
SOC := t31
else ifeq ($(CONFIG_SOC_T41),y)
SOC := t41
endif

obj-$(CONFIG_INGENIC_ISP)      += $(SOC)/

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) O=$(O) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) clean
