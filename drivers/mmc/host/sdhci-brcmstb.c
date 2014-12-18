/*
 * Copyright (C) 2009 Al Cooper <acooper@broadcom.com>
 * Copyright (C) 2015 Jaedon Shin <jaedon.shin@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include "sdhci-pltfm.h"

#define SDIO_CFG_CTRL1		0x00
#define SDIO_CFG_CTRL2		0x04
#define SDIO_CFG_CAP0		0x0c
#define SDIO_CFG_CAP1		0x10
#define SDIO_CFG_SCRATCH	0xfc

static u32 sdhci_brcmstb_readl(struct sdhci_host *host, int reg)
{
	return __raw_readl(host->ioaddr + reg);
}

static void sdhci_brcmstb_writel(struct sdhci_host *host, u32 val, int reg)
{
	__raw_writel(val, host->ioaddr + reg);
}

static u16 sdhci_brcmstb_readw(struct sdhci_host *host, int reg)
{
	return __raw_readw(host->ioaddr + reg);
}

static void sdhci_brcmstb_writew(struct sdhci_host *host, u16 val, int reg)
{
	__raw_writew(val, host->ioaddr + reg);
}

static const struct sdhci_ops sdhci_brcmstb_ops = {
	.read_w		= sdhci_brcmstb_readw,
	.write_w	= sdhci_brcmstb_writew,
	.read_l		= sdhci_brcmstb_readl,
	.write_l	= sdhci_brcmstb_writel,
	.reset		= sdhci_reset,
	.set_clock	= sdhci_set_clock,
	.set_bus_width	= sdhci_set_bus_width,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static const struct sdhci_pltfm_data sdhci_brcmstb_pdata = {
	.ops		= &sdhci_brcmstb_ops
};

static int sdhci_brcmstb_probe(struct platform_device *pdev)
{
	void __iomem *ctrl_regs;
	u32 reg_val;

	ctrl_regs = devm_ioremap_resource(&pdev->dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 1));
	if (IS_ERR(ctrl_regs))
		return PTR_ERR(ctrl_regs);

	if (__raw_readl(ctrl_regs + SDIO_CFG_SCRATCH) & 0x1)
		return -ENODEV;

	reg_val = __raw_readl(ctrl_regs + SDIO_CFG_CTRL1);
	__raw_writel(reg_val & ~0xf000, ctrl_regs + SDIO_CFG_CTRL1);

	reg_val = __raw_readl(ctrl_regs + SDIO_CFG_CTRL2);
	__raw_writel(reg_val & ~0x00ff, ctrl_regs + SDIO_CFG_CTRL2);

	reg_val = __raw_readl(ctrl_regs + SDIO_CFG_CTRL1);
	__raw_writel(reg_val & ~0x0400, ctrl_regs + SDIO_CFG_CTRL1);

	if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN)) {
		reg_val = __raw_readl(ctrl_regs + SDIO_CFG_CTRL1);
		__raw_writel(reg_val | 0xe000, ctrl_regs + SDIO_CFG_CTRL1);

		reg_val = __raw_readl(ctrl_regs + SDIO_CFG_CTRL2);
		__raw_writel(reg_val | 0x0050, ctrl_regs + SDIO_CFG_CTRL2);
	} else {
		reg_val = __raw_readl(ctrl_regs + SDIO_CFG_CTRL1);
		__raw_writel(reg_val | 0x3000, ctrl_regs + SDIO_CFG_CTRL1);
	}

	reg_val = __raw_readl(ctrl_regs + SDIO_CFG_CAP0);
	__raw_writel(reg_val & ~0x02000000, ctrl_regs + SDIO_CFG_CAP0);

	reg_val = __raw_readl(ctrl_regs + SDIO_CFG_CAP1);
	__raw_writel(reg_val | 0x80000000, ctrl_regs + SDIO_CFG_CAP1);

	return sdhci_pltfm_register(pdev, &sdhci_brcmstb_pdata, 0);
}

static int sdhci_brcmstb_remove(struct platform_device *pdev)
{
	return sdhci_pltfm_unregister(pdev);
}

static const struct of_device_id of_device_match[] = {
	{ .compatible = "brcm,brcmstb-sdhci", },
	{ },
};

MODULE_DEVICE_TABLE(of, of_device_match);

static struct platform_driver sdhci_brcmstb_driver = {
	.probe	= sdhci_brcmstb_probe,
	.remove	= sdhci_brcmstb_remove,
	.driver	= {
		.name	= "sdhci-brcmstb",
		.of_match_table = of_device_match,
	},
};

module_platform_driver(sdhci_brcmstb_driver);
