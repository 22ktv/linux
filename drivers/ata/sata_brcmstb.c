/*
 * Broadcom SATA3 AHCI Controller Driver
 *
 * Copyright © 2009-2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/ahci_platform.h>
#include <linux/compiler.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/libata.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>

#include "ahci.h"

#define DRV_NAME			"brcm-ahci"

#define SATA_TOP_CTRL_VERSION		0x0
#define SATA_TOP_CTRL_BUS_CTRL		0x4

#ifdef __BIG_ENDIAN
#define DATA_ENDIAN			 2 /* AHCI->DDR inbound accesses */
#define MMIO_ENDIAN			 2 /* CPU->AHCI outbound accesses */
#else
#define DATA_ENDIAN			 0
#define MMIO_ENDIAN			 0
#endif

struct brcm_ahci_priv {
	struct device *dev;
	struct ahci_host_priv *hpriv;

	void __iomem *top_ctrl;
};

static const struct ata_port_info ahci_brcm_port_info = {
	.flags		= AHCI_FLAG_COMMON,
	.pio_mask	= ATA_PIO4,
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &ahci_platform_ops,
};

static void brcm_sata3_init_config(struct brcm_ahci_priv *priv)
{
	/* Configure endianness */
	writel((DATA_ENDIAN << 4) | (DATA_ENDIAN << 2) | (MMIO_ENDIAN << 0),
		priv->top_ctrl + SATA_TOP_CTRL_BUS_CTRL);
}

static int brcm_ahci_suspend(struct device *dev)
{
	return ahci_platform_suspend(dev);
}

static int brcm_ahci_resume(struct device *dev)
{
	struct brcm_ahci_priv *priv = dev_get_drvdata(dev);

	brcm_sata3_init_config(priv);
	return ahci_platform_resume(dev);
}

static struct scsi_host_template ahci_platform_sht = {
	AHCI_SHT(DRV_NAME),
};

static int brcm_ahci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct brcm_ahci_priv *priv;
	struct ahci_host_priv *hpriv;
	struct resource *res;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	dev_set_drvdata(dev, priv);
	priv->dev = dev;

	hpriv = ahci_platform_get_resources(pdev);
	if (IS_ERR(hpriv))
		return PTR_ERR(hpriv);

	priv->hpriv = hpriv;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "top-ctrl");
	priv->top_ctrl = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->top_ctrl))
		return PTR_ERR(priv->top_ctrl);

	brcm_sata3_init_config(priv);

	ret = ahci_platform_enable_resources(hpriv);
	if (ret)
		return ret;

	hpriv->plat_data = priv;

	ret = ahci_platform_init_host(pdev, hpriv, &ahci_brcm_port_info,
				      &ahci_platform_sht);
	if (ret)
		return ret;

	dev_info(dev, "Broadcom AHCI SATA3 registered\n");

	return 0;
}

static const struct of_device_id ahci_of_match[] = {
	{.compatible = "brcm,sata3-ahci"},
	{},
};
MODULE_DEVICE_TABLE(of, ahci_of_match);

static SIMPLE_DEV_PM_OPS(ahci_brcm_pm_ops, brcm_ahci_suspend, brcm_ahci_resume);

static struct platform_driver brcm_ahci_driver = {
	.probe = brcm_ahci_probe,
	.remove = ata_platform_remove_one,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = ahci_of_match,
		.pm = &ahci_brcm_pm_ops,
	},
};
module_platform_driver(brcm_ahci_driver);

MODULE_DESCRIPTION("Broadcom SATA3 AHCI Controller Driver");
MODULE_AUTHOR("Brian Norris");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sata-brcmstb");
