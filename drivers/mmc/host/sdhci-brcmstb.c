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

static int (*brcmstb_sdhci_fixup)(void __iomem *base);

enum sdhci_regs_40nm {
	SDIO_CFG_CTRL1 = 0x00,
	SDIO_CFG_CTRL2 = 0x04,
	SDIO_CFG_CAP0 = 0x0c,
	SDIO_CFG_CAP1 = 0x10,
	SDIO_CFG_SCRATCH = 0xfc
};

enum sdhci_cap0 {
	CAP_DDR50_SUPPORT = BIT(31),
	CAP_SD104_SUPPORT = BIT(30),
	CAP_SDR50 = BIT(29),
	CAP_SLOT_TYPE = BIT(27),
	CAP_ASYNCH_INT_SUPPORT = BIT(26),
	CAP_64B_SYS_BUS = BIT(25),
	CAP_1_8V_SUPPORT = BIT(24),
	CAP_3_0V_SUPPORT = BIT(23),
	CAP_3_3V_SUPPORT = BIT(22),
	CAP_SUSP_RES = BIT(21),
	CAP_SDMA_SUPPORT = BIT(20),
	CAP_HIGH_SPEED_SUPPORT = BIT(19),
	CAP_ADMA2_SUPPORT = BIT(18),
	CAP_EXTENDED_MEDIA_SUPPORT = BIT(17)

};

enum sdhci_cap1 {
	CAP_REG_OVERRIDE = BIT(31)
};

static void brcmstb_sdhci_unmask(void __iomem *base, u32 mask)
{
	u32 val;

	val = readl(base);
	val &= ~mask;
	writel(val, base);
}

static void brcmstb_sdhci_mask(void __iomem *base, u32 mask)
{
	u32 val;

	val = readl(base);
	val |= mask;
	writel(val, base);
}

static int brcmstb_sdhci_fixup_40nm(void __iomem *base)
{
	if (readl(base + SDIO_CFG_SCRATCH) & 0x1)
		return -ENODEV;

	brcmstb_sdhci_unmask(base + SDIO_CFG_CTRL1, 0xf000);
	brcmstb_sdhci_unmask(base + SDIO_CFG_CTRL2, 0x00ff);
	brcmstb_sdhci_unmask(base + SDIO_CFG_CTRL1, 0x0400);

	if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN)) {
		brcmstb_sdhci_mask(base + SDIO_CFG_CTRL1, 0xe000);
		brcmstb_sdhci_mask(base + SDIO_CFG_CTRL2, 0x0050);
	} else
		brcmstb_sdhci_mask(base + SDIO_CFG_CTRL1, 0x3000);

	return 0;
}

static int bcm7346_fixup(void __iomem *base)
{
	if (brcmstb_sdhci_fixup_40nm(base))
		return -ENODEV;

	brcmstb_sdhci_mask(base + SDIO_CFG_CAP1, CAP_REG_OVERRIDE);

	return 0;
}

static int bcm7362_fixup(void __iomem *base)
{
	if (brcmstb_sdhci_fixup_40nm(base))
		return -ENODEV;

	brcmstb_sdhci_unmask(base + SDIO_CFG_CAP0, CAP_64B_SYS_BUS);
	brcmstb_sdhci_mask(base + SDIO_CFG_CAP1, CAP_REG_OVERRIDE);

	return 0;
}

static int bcm7425_fixup(void __iomem *base)
{
	if (brcmstb_sdhci_fixup_40nm(base))
		return -ENODEV;

	brcmstb_sdhci_mask(base + SDIO_CFG_CAP0, CAP_SDR50);
	brcmstb_sdhci_mask(base + SDIO_CFG_CAP0, CAP_1_8V_SUPPORT);
	brcmstb_sdhci_mask(base + SDIO_CFG_CAP0, (100 << 7));
	brcmstb_sdhci_mask(base + SDIO_CFG_CAP0, 50);
	brcmstb_sdhci_mask(base + SDIO_CFG_CAP1, CAP_REG_OVERRIDE);

	return 0;
}

static int bcm7445_fixup(void __iomem *base)
{
	brcmstb_sdhci_unmask(base + SDIO_CFG_CAP0, CAP_SDR50);
	brcmstb_sdhci_mask(base + SDIO_CFG_CAP1, CAP_REG_OVERRIDE);

	return 0;
}

static struct sdhci_pltfm_data brcmstb_sdhci_pdata = {
};

static const struct of_device_id brcmstb_sdhci_of_match[] = {
	{ .compatible = "brcm,bcm7346-sdhci", .data = bcm7346_fixup },
	{ .compatible = "brcm,bcm7360-sdhci", .data = bcm7362_fixup },
	{ .compatible = "brcm,bcm7362-sdhci", .data = bcm7362_fixup },
	{ .compatible = "brcm,bcm7425-sdhci", .data = bcm7425_fixup },
	{ .compatible = "brcm,bcm7445-sdhci", .data = bcm7445_fixup },
	{ },
};
MODULE_DEVICE_TABLE(of, brcmstb_sdhci_of_match);

static int brcmstb_sdhci_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	const struct of_device_id *of_id = NULL;
	void __iomem *cfg_base;
	struct resource *res;
	int ret;

	of_id = of_match_node(brcmstb_sdhci_of_match, pdev->dev.of_node);
	if (!of_id)
		return -EINVAL;

	brcmstb_sdhci_fixup = of_id->data;
	if (brcmstb_sdhci_fixup) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cfg");
		if (res) {
			cfg_base = devm_ioremap_resource(&pdev->dev, res);
			if (IS_ERR(cfg_base))
				return PTR_ERR(cfg_base);

			if (brcmstb_sdhci_fixup(cfg_base))
				return -ENODEV;

			iounmap(cfg_base);
		}
	}

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
