/*
 * Copyright (C) 2014 Marvell Technology Group Ltd.
 * Copyright (C) 2009 - 2012 Broadcom Corporation
 * Copyright (C) 2015 Jaedon Shin <jaedon.shin@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME	": " fmt

#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#define SATA_TOP_CTRL_BUS_CTRL		0x24

#define MDIO_TXPMD_0_REG_BANK		0x1a0
#define MDIO_TXPMD_1_REG_BANK		0x1b0

#define MDIO_TXPMD_CTRL1		0x81
#define MDIO_TXPMD_TX_FREQ_CTRL1	0x82
#define MDIO_TXPMD_TX_FREQ_CTRL2	0x83
#define MDIO_TXPMD_TX_FREQ_CTRL3	0x84

struct phy_brcmstb_desc {
	struct phy *phy;
	int idx;
	bool ssc_enable;
};

struct phy_brcmstb_priv {
	void __iomem *regs;
	struct phy_brcmstb_desc **phys;
	u32 nphys;
};

static inline void brcmstb_mdio_write(void __iomem *base, u32 addr,
		u32 data, u32 mask, u32 val)
{
	u32 reg_val;

	__raw_writel(addr, base + 0x8f * 4);

	reg_val = __raw_readl(base + data * 4);
	reg_val = (reg_val & mask) | val;
	__raw_writel(reg_val, base + data * 4);

}

static int brcmstb_phy_power_on(struct phy *phy)
{
	struct phy_brcmstb_desc *desc = phy_get_drvdata(phy);
	struct phy_brcmstb_priv *priv = dev_get_drvdata(phy->dev.parent);
	void __iomem *base = priv->regs + 0x100;
	u32 addr;

	addr = (desc->idx == 0) ?
			MDIO_TXPMD_0_REG_BANK : MDIO_TXPMD_1_REG_BANK;

	/* Enable SSC force */
	brcmstb_mdio_write(base, addr, MDIO_TXPMD_CTRL1,
			0xfffffffc, 0x00000003);

	/* Set fixed minimum frequency */
	brcmstb_mdio_write(base, addr, MDIO_TXPMD_TX_FREQ_CTRL2,
			0xfffffc00, 0x000003df);

	/* Set fixed maximum frequency */
	if (desc->ssc_enable)
		brcmstb_mdio_write(base, addr, MDIO_TXPMD_TX_FREQ_CTRL3,
				0xfffffc00, 0x00000083);
	else
		brcmstb_mdio_write(base, addr, MDIO_TXPMD_TX_FREQ_CTRL3,
				0xfffffc00, 0x000003df);

	return 0;
}

static int brcmstb_phy_power_off(struct phy *phy)
{
	return 0;
}

static struct phy *phy_brcmstb_sata_phy_xlate(struct device *dev,
		struct of_phandle_args *args)
{
	struct phy_brcmstb_priv *priv = dev_get_drvdata(dev);
	int i;

	if (WARN_ON(args->args[0] >= priv->nphys))
		return ERR_PTR(-ENODEV);

	for (i = 0; i < priv->nphys; i++) {
		if (priv->phys[i]->idx == args->args[0])
			break;
	}

	if (i == priv->nphys)
		return ERR_PTR(-ENODEV);

	return priv->phys[i]->phy;
}

static struct phy_ops ops = {
	.power_on	= brcmstb_phy_power_on,
	.power_off	= brcmstb_phy_power_off,
	.owner		= THIS_MODULE,
};

static int brcmstb_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *child;
	struct phy *phy;
	struct phy_provider *phy_provider;
	struct phy_brcmstb_priv *priv;
	u32 phy_id;
	bool ssc_enable;
	int i = 0;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (IS_ERR(priv))
		return PTR_ERR(priv);

	priv->regs = devm_ioremap_resource(dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
		__raw_writel(0x2a, priv->regs + SATA_TOP_CTRL_BUS_CTRL);
	else
		__raw_writel(0, priv->regs + SATA_TOP_CTRL_BUS_CTRL);

	priv->nphys = of_get_child_count(dev->of_node);
	if (priv->nphys == 0)
		return -ENODEV;

	priv->phys = devm_kzalloc(dev, priv->nphys * sizeof(*priv->phys),
				  GFP_KERNEL);
	if (IS_ERR(priv->phys))
		return PTR_ERR(priv->phys);

	dev_set_drvdata(dev, priv);

	for_each_available_child_of_node(dev->of_node, child) {
		struct phy_brcmstb_desc *phy_desc;

		if (of_property_read_u32(child, "reg", &phy_id))
			return -EINVAL;

		if (of_get_property(child, "ssc-enable", NULL))
			ssc_enable = false;
		else
			ssc_enable = true;

		phy_desc = devm_kzalloc(dev, sizeof(*phy_desc), GFP_KERNEL);
		if (IS_ERR(phy_desc))
			return PTR_ERR(phy_desc);

		phy = devm_phy_create(dev, NULL, &ops);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		phy_desc->phy = phy;
		phy_desc->idx = phy_id;
		phy_desc->ssc_enable = ssc_enable;
		phy_set_drvdata(phy, phy_desc);

		priv->phys[i++] = phy_desc;
	}

	phy_provider = devm_of_phy_provider_register(dev,
			phy_brcmstb_sata_phy_xlate);
	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id of_device_match[] = {
	{ .compatible = "brcm,brcmstb-phy" },
	{ },
};

MODULE_DEVICE_TABLE(of, of_device_match);

static struct platform_driver phy_brcmstb_sata_driver = {
	.probe	= brcmstb_phy_probe,
	.driver	= {
		.name	= "phy-brcmstb-sata",
		.of_match_table = of_device_match,
	},
};

module_platform_driver(phy_brcmstb_sata_driver);
