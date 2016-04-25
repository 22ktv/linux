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

#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/of.h>
#include "sdhci-pltfm.h"

static struct sdhci_pltfm_data brcmstb_sdhci_pdata = {
};

#ifdef CONFIG_PM_SLEEP
static int sdhci_brcmstb_suspend(struct device *dev)
{
	struct sdhci_host *host = dev_get_drvdata(dev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	int ret;

	ret = sdhci_suspend_host(host);
	if (ret)
		return ret;
	clk_disable(pltfm_host->clk);
	return ret;
}

static int sdhci_brcmstb_resume(struct device *dev)
{
        struct sdhci_host *host = dev_get_drvdata(dev);
        struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
        int ret;

        ret = clk_enable(pltfm_host->clk);
        if (ret)
                return ret;
        return sdhci_resume_host(host);
}

static SIMPLE_DEV_PM_OPS(sdhci_brcmstb_pmops, sdhci_brcmstb_suspend,
			 sdhci_brcmstb_resume);

#define SDHCI_BRCMSTB_PMOPS (&sdhci_brcmstb_pmops)
#else
#define SDHCI_BRCMSTB_PMOPS NULL
#endif

static int brcmstb_sdhci_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	int ret;

	host = sdhci_pltfm_init(pdev, &brcmstb_sdhci_pdata, 0);
	if (IS_ERR(host))
		return PTR_ERR(host);

	pltfm_host = sdhci_priv(host);
	pltfm_host->clk = devm_clk_get(&pdev->dev, NULL);

	if (!IS_ERR(pltfm_host->clk)) {
		ret = clk_prepare_enable(pltfm_host->clk);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable host clk\n");
			goto err;
		}
	}

        mmc_of_parse(host->mmc);
        sdhci_get_of_property(pdev);

	if (of_get_property(np, "broken-64-bit-dma", NULL))
		host->quirks2 |= SDHCI_QUIRK2_BROKEN_64_BIT_DMA;

	ret = sdhci_add_host(host);
	if (ret)
		goto err_clk;

	return 0;

err_clk:
	clk_disable_unprepare(pltfm_host->clk);
err:
	sdhci_pltfm_free(pdev);
	return ret;
}

static const struct of_device_id brcmstb_sdhci_of_match[] = {
	{ .compatible = "brcm,sdhci-brcmstb" },
	{ },
};
MODULE_DEVICE_TABLE(of, brcmstb_sdhci_of_match);

static struct platform_driver brcmstb_sdhci_driver = {
	.driver = {
		.name = "sdhci-brcmstb",
		.of_match_table = of_match_ptr(brcmstb_sdhci_of_match),
		.pm = SDHCI_BRCMSTB_PMOPS,
	},
	.probe = brcmstb_sdhci_probe,
	.remove = sdhci_pltfm_unregister,
};
module_platform_driver(brcmstb_sdhci_driver);

MODULE_DESCRIPTION("SDHCI driver for Broadcom");
MODULE_AUTHOR("Al Cooper <acooper@broadcom.com>");
MODULE_LICENSE("GPL v2");
