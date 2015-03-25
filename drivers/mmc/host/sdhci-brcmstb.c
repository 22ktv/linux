/*
 * sdhci-brcmstb.c Support for SDHCI on Broadcom SoC's
 *
 * Copyright (C) 2013 Broadcom Corporation
 *
 * Author: Al Cooper <acooper@broadcom.com>
 * Based on sdhci-dove.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/io.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>

#include "sdhci-pltfm.h"

#define SDIO_CFG_CTRL1		0x00
#define SDIO_CFG_CTRL2		0x04
#define SDIO_CFG_CAP0		0x0c
#define SDIO_CFG_CAP1		0x10
#define SDIO_CFG_SCRATCH	0xfc

static struct sdhci_pltfm_data sdhci_brcmstb_pdata = {
};

static int sdhci_brcmstb_set_cfg(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cfg");
	if (res) {
		void __iomem *cfg;
		u32 reg;

		cfg = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(cfg))
			return PTR_ERR(cfg);

		if (readl(cfg + SDIO_CFG_SCRATCH) & 0x1)
			return -ENODEV;

		reg = readl(cfg + SDIO_CFG_CTRL1);
		reg &= ~0xf000;
		writel(reg, cfg + SDIO_CFG_CTRL1);

		reg = readl(cfg + SDIO_CFG_CTRL2);
		reg &= ~0x00ff;
		writel(reg, cfg + SDIO_CFG_CTRL2);

		reg = readl(cfg + SDIO_CFG_CTRL1);
		reg &= ~0x0400;
		writel(reg, cfg + SDIO_CFG_CTRL1);

		if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN)) {
			reg = readl(cfg + SDIO_CFG_CTRL1);
			reg |= 0xe000;
			writel(reg, cfg + SDIO_CFG_CTRL1);

			reg = readl(cfg + SDIO_CFG_CTRL2);
			reg |= 0x0050;
			writel(reg, cfg + SDIO_CFG_CTRL2);
		} else {
			reg = readl(cfg + SDIO_CFG_CTRL1);
			reg |= 0x3000;
			writel(reg, cfg + SDIO_CFG_CTRL1);
		}

		reg = readl(cfg + SDIO_CFG_CAP0);
		reg &= ~0x02000000;
		writel(reg, cfg + SDIO_CFG_CAP0);

		reg = readl(cfg + SDIO_CFG_CAP1);
		reg |= 0x80000000;
		writel(reg, cfg + SDIO_CFG_CAP1);

		iounmap(cfg);
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int sdhci_brcmstb_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int res;

	res = sdhci_suspend_host(host);
	if (res)
		return res;
	clk_disable(pltfm_host->clk);
	return res;
}

static int sdhci_brcmstb_resume(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int err;

	err = clk_enable(pltfm_host->clk);
	if (err)
		return err;
	return sdhci_resume_host(host);
}

#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(sdhci_brcmstb_pmops, sdhci_brcmstb_suspend,
			sdhci_brcmstb_resume);

static int sdhci_brcmstb_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	int res;

	if (sdhci_brcmstb_set_cfg(pdev))
		return -ENODEV;

	res = sdhci_pltfm_register(pdev, &sdhci_brcmstb_pdata, 0);
	if (res)
		return res;
	host = platform_get_drvdata(pdev);
	pltfm_host = sdhci_priv(host);

	pltfm_host->clk = of_clk_get_by_name(dn, "sw_sdio");
	if (IS_ERR(pltfm_host->clk)) {
		dev_err(&pdev->dev, "Clock not found in Device Tree\n");
		pltfm_host->clk = NULL;
	}
	res = clk_prepare_enable(pltfm_host->clk);
	if (res)
		sdhci_pltfm_unregister(pdev);
	return res;
}

static int sdhci_brcmstb_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int res;
	res = sdhci_pltfm_unregister(pdev);
	clk_disable_unprepare(pltfm_host->clk);
	return res;
}


static const struct of_device_id sdhci_brcm_of_match[] = {
	{ .compatible = "brcm,sdhci-brcmstb" },
	{},
};

static struct platform_driver sdhci_brcmstb_driver = {
	.driver		= {
		.name	= "sdhci-brcmstb",
		.owner	= THIS_MODULE,
		.pm	= &sdhci_brcmstb_pmops,
		.of_match_table = of_match_ptr(sdhci_brcm_of_match),
	},
	.probe		= sdhci_brcmstb_probe,
	.remove		= sdhci_brcmstb_remove,
};

module_platform_driver(sdhci_brcmstb_driver);

MODULE_DESCRIPTION("SDHCI driver for Broadcom");
MODULE_AUTHOR("Al Cooper <acooper@broadcom.com>");
MODULE_LICENSE("GPL v2");
