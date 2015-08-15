/*
 * Copyright (C) 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/mmc/host.h>
#include <linux/of.h>
#include "sdhci-pltfm.h"

static struct sdhci_pltfm_data brcmstb_sdhci_pdata = {
};

static const struct of_device_id brcmstb_sdhci_of_match[] = {
	{ .compatible = "brcm,bcm7346-sdhci" },
	{ .compatible = "brcm,bcm7360-sdhci" },
	{ .compatible = "brcm,bcm7362-sdhci" },
	{ .compatible = "brcm,bcm7425-sdhci" },
	{ .compatible = "brcm,bcm7445-sdhci" },
	{ },
};
MODULE_DEVICE_TABLE(of, brcmstb_sdhci_of_match);

static int brcmstb_sdhci_probe(struct platform_device *pdev)
{
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

static struct platform_driver brcmstb_sdhci_driver = {
	.driver = {
		.name = "sdhci-brcmstb",
		.of_match_table = of_match_ptr(brcmstb_sdhci_of_match),
		.pm = SDHCI_PLTFM_PMOPS,
	},
	.probe = brcmstb_sdhci_probe,
	.remove = sdhci_pltfm_unregister,
};
module_platform_driver(brcmstb_sdhci_driver);

MODULE_DESCRIPTION("Broadcom BCM7XXX SDHCI driver");
MODULE_AUTHOR("Broadcom");
MODULE_LICENSE("GPL v2");
