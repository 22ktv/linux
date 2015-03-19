/*
 * Broadcom SATA3 AHCI Controller PHY Driver
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

#include <linux/device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#define SATA_MDIO_BANK_OFFSET				0x23c
#define SATA_MDIO_REG_OFFSET(ofs)			((ofs) * 4)
#define SATA_MDIO_REG_SPACE_SIZE			0x1000
#define SATA_MDIO_REG_LENGTH				0x1f00

#define SATA_TOP_CTRL_PHY_CTRL_1			0x0
 #define SATA_TOP_CTRL_1_PHY_DEFAULT_POWER_STATE	BIT(14)

#define SATA_TOP_CTRL_PHY_CTRL_2			0x4
 #define SATA_TOP_CTRL_2_SW_RST_MDIOREG			BIT(0)
 #define SATA_TOP_CTRL_2_SW_RST_OOB			BIT(1)
 #define SATA_TOP_CTRL_2_SW_RST_RX			BIT(2)
 #define SATA_TOP_CTRL_2_SW_RST_TX			BIT(3)
 #define SATA_TOP_CTRL_2_PHY_GLOBAL_RESET		BIT(14)

#define MAX_PORTS					2
/* Register offset between PHYs in port-ctrl */
#define SATA_TOP_CTRL_PHY_CTRL_LEN			0x8
/* Register offset between PHYs in PCB space */
#define SATA_MDIO_REG_SPACE_SIZE			0x1000

struct brcm_sata_port {
	int portnum;
	struct phy *phy;
	struct brcm_sata_phy *phy_priv;
	bool ssc_en;
};

struct brcm_sata_phy {
	struct device *dev;
	void __iomem *port_ctrl;
	void __iomem *phy_base;

	struct brcm_sata_port phys[MAX_PORTS];
};

enum sata_mdio_phy_regs_28nm {
	PLL_REG_BANK_0				= 0x50,
	PLL_REG_BANK_0_PLLCONTROL_0		= 0x81,

	TXPMD_REG_BANK				= 0x1a0,
	TXPMD_CONTROL1				= 0x81,
	TXPMD_CONTROL1_TX_SSC_EN_FRC		= BIT(0),
	TXPMD_CONTROL1_TX_SSC_EN_FRC_VAL	= BIT(1),
	TXPMD_TX_FREQ_CTRL_CONTROL1		= 0x82,
	TXPMD_TX_FREQ_CTRL_CONTROL2		= 0x83,
	TXPMD_TX_FREQ_CTRL_CONTROL2_FMIN_MASK	= 0x3ff,
	TXPMD_TX_FREQ_CTRL_CONTROL3		= 0x84,
	TXPMD_TX_FREQ_CTRL_CONTROL3_FMAX_MASK	= 0x3ff,
};

static inline void __iomem *sata_phy_get_port_ctrl(struct brcm_sata_port *port)
{
	struct brcm_sata_phy *priv = port->phy_priv;

	return priv->port_ctrl + (port->portnum * SATA_TOP_CTRL_PHY_CTRL_LEN);
}
static inline void __iomem *sata_phy_get_phy_base(struct brcm_sata_port *port)
{
	struct brcm_sata_phy *priv = port->phy_priv;

	return priv->phy_base + (port->portnum * SATA_MDIO_REG_SPACE_SIZE);
}

static void brcm_sata_mdio_wr(void __iomem *addr, u32 bank, u32 ofs,
			      u32 msk, u32 value)
{
	u32 tmp;

	writel(bank, addr + SATA_MDIO_BANK_OFFSET);
	tmp = readl(addr + SATA_MDIO_REG_OFFSET(ofs));
	tmp = (tmp & msk) | value;
	writel(tmp, addr + SATA_MDIO_REG_OFFSET(ofs));
}

/* These defaults were characterized by H/W group */
#define FMIN_VAL_DEFAULT	0x3df
#define FMAX_VAL_DEFAULT	0x3df
#define FMAX_VAL_SSC		0x83

static void cfg_ssc_28nm(struct brcm_sata_port *port)
{
	void __iomem *base = sata_phy_get_phy_base(port);
	struct brcm_sata_phy *priv = port->phy_priv;
	u32 tmp;

	/* override the TX spread spectrum setting */
	tmp = TXPMD_CONTROL1_TX_SSC_EN_FRC_VAL | TXPMD_CONTROL1_TX_SSC_EN_FRC;
	brcm_sata_mdio_wr(base, TXPMD_REG_BANK, TXPMD_CONTROL1, ~tmp, tmp);

	/* set fixed min freq */
	brcm_sata_mdio_wr(base, TXPMD_REG_BANK, TXPMD_TX_FREQ_CTRL_CONTROL2,
			  ~TXPMD_TX_FREQ_CTRL_CONTROL2_FMIN_MASK,
			  FMIN_VAL_DEFAULT);

	/* set fixed max freq depending on SSC config */
	if (port->ssc_en) {
		dev_info(priv->dev, "enabling SSC on port %d\n", port->portnum);
		tmp = FMAX_VAL_SSC;
	} else {
		tmp = FMAX_VAL_DEFAULT;
	}

	brcm_sata_mdio_wr(base, TXPMD_REG_BANK, TXPMD_TX_FREQ_CTRL_CONTROL3,
			  ~TXPMD_TX_FREQ_CTRL_CONTROL3_FMAX_MASK, tmp);
}

static void brcm_sata_phy_enable(struct brcm_sata_port *port)
{
	void __iomem *port_ctrl = sata_phy_get_port_ctrl(port);
	void __iomem *p;
	u32 reg;

	/* clear PHY_DEFAULT_POWER_STATE */
	p = port_ctrl + SATA_TOP_CTRL_PHY_CTRL_1;
	reg = readl(p);
	reg &= ~SATA_TOP_CTRL_1_PHY_DEFAULT_POWER_STATE;
	writel(reg, p);

	/* reset the PHY digital logic */
	p = port_ctrl + SATA_TOP_CTRL_PHY_CTRL_2;
	reg = readl(p);
	reg &= ~(SATA_TOP_CTRL_2_SW_RST_MDIOREG | SATA_TOP_CTRL_2_SW_RST_OOB |
		 SATA_TOP_CTRL_2_SW_RST_RX);
	reg |= SATA_TOP_CTRL_2_SW_RST_TX;
	writel(reg, p);
	reg = readl(p);
	reg |= SATA_TOP_CTRL_2_PHY_GLOBAL_RESET;
	writel(reg, p);
	reg = readl(p);
	reg &= ~SATA_TOP_CTRL_2_PHY_GLOBAL_RESET;
	writel(reg, p);
	(void)readl(p);
}

static void brcm_sata_phy_disable(struct brcm_sata_port *port)
{
	void __iomem *port_ctrl = sata_phy_get_port_ctrl(port);
	void __iomem *p;
	u32 reg;

	/* power-off the PHY digital logic */
	p = port_ctrl + SATA_TOP_CTRL_PHY_CTRL_2;
	reg = readl(p);
	reg |= (SATA_TOP_CTRL_2_SW_RST_MDIOREG | SATA_TOP_CTRL_2_SW_RST_OOB |
		SATA_TOP_CTRL_2_SW_RST_RX | SATA_TOP_CTRL_2_SW_RST_TX |
		SATA_TOP_CTRL_2_PHY_GLOBAL_RESET);
	writel(reg, p);

	/* set PHY_DEFAULT_POWER_STATE */
	p = port_ctrl + SATA_TOP_CTRL_PHY_CTRL_1;
	reg = readl(p);
	reg |= SATA_TOP_CTRL_1_PHY_DEFAULT_POWER_STATE;
	writel(reg, p);
}

static int brcmstb_sata_phy_power_on(struct phy *phy)
{
	struct brcm_sata_port *port = phy_get_drvdata(phy);

	dev_info(port->phy_priv->dev, "powering on port %d\n", port->portnum);

	brcm_sata_phy_enable(port);
	cfg_ssc_28nm(port);

	return 0;
}

static int brcmstb_sata_phy_power_off(struct phy *phy)
{
	struct brcm_sata_port *port = phy_get_drvdata(phy);

	dev_info(port->phy_priv->dev, "powering off port %d\n", port->portnum);

	brcm_sata_phy_disable(port);

	return 0;
}

static struct phy_ops phy_ops_28nm = {
	.power_on	= brcmstb_sata_phy_power_on,
	.power_off	= brcmstb_sata_phy_power_off,
	.owner		= THIS_MODULE,
};

static struct phy *brcm_sata_phy_xlate(struct device *dev,
				       struct of_phandle_args *args)
{
	struct brcm_sata_phy *priv = dev_get_drvdata(dev);
	int i = args->args[0];

	if (i >= MAX_PORTS || !priv->phys[i].phy) {
		dev_err(dev, "invalid phy: %d\n", i);
		return ERR_PTR(-ENODEV);
	}

	return priv->phys[i].phy;
}

static const struct of_device_id brcmstb_sata_phy_of_match[] = {
	{ .compatible	= "brcm,bcm7445-sata-phy" },
	{},
};
MODULE_DEVICE_TABLE(of, brcmstb_sata_phy_of_match);

static int brcmstb_sata_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *dn = dev->of_node, *child;
	struct brcm_sata_phy *priv;
	struct resource *res;
	struct phy_provider *provider;
	int count = 0;

	if (of_get_child_count(dn) == 0)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	dev_set_drvdata(dev, priv);
	priv->dev = dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "port-ctrl");
	if (!res) {
		dev_err(dev, "couldn't get port-ctrl resource\n");
		return -EINVAL;
	}
	/*
	 * Don't request region, since it may be within a region owned by the
	 * SATA driver
	 */
	priv->port_ctrl = devm_ioremap(dev, res->start, resource_size(res));
	if (!priv->port_ctrl) {
		dev_err(dev, "couldn't remap: %pR\n", res);
		return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "phy");
	priv->phy_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->phy_base))
		return PTR_ERR(priv->phy_base);

	for_each_available_child_of_node(dn, child) {
		unsigned int id;
		struct brcm_sata_port *port;

		if (of_property_read_u32(child, "reg", &id)) {
			dev_err(dev, "missing reg property in node %s\n",
					child->name);
			return -EINVAL;
		}

		if (id >= MAX_PORTS) {
			dev_err(dev, "invalid reg: %u\n", id);
			return -EINVAL;
		}
		if (priv->phys[id].phy) {
			dev_err(dev, "already registered port %u\n", id);
			return -EINVAL;
		}

		port = &priv->phys[id];
		port->portnum = id;
		port->phy_priv = priv;
		port->phy = devm_phy_create(dev, NULL, &phy_ops_28nm);
		port->ssc_en = of_property_read_bool(child, "brcm,enable-ssc");
		if (IS_ERR(port->phy)) {
			dev_err(dev, "failed to create PHY\n");
			return PTR_ERR(port->phy);
		}

		phy_set_drvdata(port->phy, port);
		count++;
	}

	provider = devm_of_phy_provider_register(dev, brcm_sata_phy_xlate);
	if (IS_ERR(provider)) {
		dev_err(dev, "could not register PHY provider\n");
		return PTR_ERR(provider);
	}

	dev_info(dev, "registered %d ports\n", count);

	return 0;
}

static int brcmstb_sata_phy_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver brcmstb_sata_phy_driver = {
	.probe	= brcmstb_sata_phy_probe,
	.remove	= brcmstb_sata_phy_remove,
	.driver	= {
		.of_match_table	= brcmstb_sata_phy_of_match,
		.name		= "brcmstb-sata-phy",
	}
};
module_platform_driver(brcmstb_sata_phy_driver);

MODULE_DESCRIPTION("Broadcom STB SATA PHY driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Marc Carino");
MODULE_AUTHOR("Brian Norris");
MODULE_ALIAS("platform:phy-brcmstb-sata");
