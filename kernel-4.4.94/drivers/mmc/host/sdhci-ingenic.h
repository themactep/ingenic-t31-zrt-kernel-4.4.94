#ifndef __SDHCI_INGENIC_H__
#define __SDHCI_INGENIC_H__

enum cd_types {
	SDHCI_INGENIC_CD_INTERNAL,	/* use mmc internal CD line */
	SDHCI_INGENIC_CD_GPIO,		/* use external gpio pin for CD line */
	SDHCI_INGENIC_CD_NONE,		/* no CD line, use polling to detect card */
	SDHCI_INGENIC_CD_PERMANENT,	/* no CD line, card permanently wired to host */
};


#define LOW_ENABLE			0
#define HIGH_ENABLE			1
struct ingenic_mmc_pin {
	short				num;
	short 				enable_level;
};

struct card_gpio {
	struct ingenic_mmc_pin 		wp;
	struct ingenic_mmc_pin 		cd;
	struct ingenic_mmc_pin 		pwr;
	struct ingenic_mmc_pin 		rst;
};

/**
 * struct sdhci_ingenic_platdata() - Platform device data for ingenic SDHCI
 * @max_width: The maximum number of data bits supported.
 * @host_caps: Standard MMC host capabilities bit field.
 * @host_caps2: The second standard MMC host capabilities bit field.
 * @cd_type: Type of Card Detection method (see cd_types enum above)
 * @ext_cd_gpio: gpio pin used for external CD line, valid only if
 *       cd_type == SDHCI_INGENIC_CD_GPIO
 * @ext_cd_gpio_invert: invert values for external CD gpio line
 * @cfg_gpio: Configure the GPIO for a specific card bit-width
 *
 * Initialisation data specific to either the machine or the platform
 * for the device driver to use or call-back when configuring gpio or
 * card speed information.
 */
struct sdhci_ingenic_pdata {
	unsigned int    host_caps;
	unsigned int    host_caps2;
	unsigned int    pm_caps;
	enum cd_types   cd_type;

	unsigned int	pio_mode;
	unsigned int	enable_autocmd12;

	struct card_gpio *gpio;
	int     ext_cd_gpio;
	bool        ext_cd_gpio_invert;

	void    (*cfg_gpio)(struct platform_device *dev, int width);
	int     (*private_init)(void);
};

/**
 * struct sdhci_ingenic - INGENIC SDHCI instance
 * @host: The SDHCI host created
 * @pdev: The platform device we where created from.
 * @ioarea: The resource created when we claimed the IO area.
 * @pdata: The platform data for this controller.
 */
struct sdhci_ingenic {
	struct sdhci_host				*host;
	struct platform_device			*pdev;
	struct device_node *node;
	struct sdhci_ingenic_pdata	*pdata;
	struct clk						*clk_cgu;
	struct clk						*clk_gate;
	int								 cur_clk;
	int								 ext_cd_irq;
	int								 ext_cd_gpio;
	unsigned long					 clk_rates;
};
#endif	/* __SDHCI_INGENIC_H__ */
