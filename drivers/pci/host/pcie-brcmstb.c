/*
 * Broadcom STB PCIe root complex driver
 *
 * Copyright (C) 2009 - 2016 Broadcom
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/msi.h>
#include <linux/printk.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/sizes.h>

#include "pcie-brcmstb.h"

static void wr_fld(void __iomem *p, u32 mask, int shift, u32 val)
{
	u32 reg;

	reg = bpcie_readl(p);
	reg = (reg & ~mask) | (val << shift);
	bpcie_writel(reg, p);
}

static void wr_fld_rb(void __iomem *p, u32 mask, int shift, u32 val)
{
	wr_fld(p, mask, shift, val);
	(void) bpcie_readl(p);
}

/* Helper macro to define low-level operations for read/write/reset */
#define PCIE_LL_OPS(name, def) \
static void name##_pcie_rgr1_sw_init(struct brcm_pcie *pcie, u32 mask,	\
				      int shift, u32 val)		\
{									\
	wr_fld_rb(pcie->base + def##PCIE_RGR1_SW_INIT_1, mask, shift, val); \
}									\
static u32 name##_pcie_read_config(struct brcm_pcie *pcie, int cfg_idx) \
{									\
	bpcie_writel(cfg_idx, pcie->base + def##PCIE_EXT_CFG_INDEX);	\
	bpcie_readl(pcie->base + def##PCIE_EXT_CFG_INDEX);		\
	return bpcie_readl(pcie->base + def##PCIE_EXT_CFG_DATA);	\
}									\
static void name##_pcie_write_config(struct brcm_pcie *pcie,		\
				     int cfg_idx, u32 val)		\
{									\
	bpcie_writel(cfg_idx, pcie->base + def##PCIE_EXT_CFG_INDEX);	\
	bpcie_readl(pcie->base + def##PCIE_EXT_CFG_INDEX);		\
	bpcie_writel(val, pcie->base + def##PCIE_EXT_CFG_DATA);		\
	bpcie_readl(pcie->base + def##PCIE_EXT_CFG_DATA);		\
}

PCIE_LL_OPS(bcm7425, BCM7425_);
/* Optional second argument */
PCIE_LL_OPS(gen,);

static const struct brcm_pcie_cfg_data bcm7425_cfg = {
	.type = BCM7425,
	.ops = {
		.read_config = bcm7425_pcie_read_config,
		.write_config = bcm7425_pcie_write_config,
		.rgr1_sw_init = bcm7425_pcie_rgr1_sw_init,
	},
};

static const struct brcm_pcie_cfg_data bcm7435_cfg = {
	.type = BCM7435,
	.ops = {
		.read_config = gen_pcie_read_config,
		.write_config = gen_pcie_write_config,
		.rgr1_sw_init = gen_pcie_rgr1_sw_init,
	},
};

static const struct brcm_pcie_cfg_data generic_cfg = {
	.type = GENERIC,
	.ops = {
		.read_config = gen_pcie_read_config,
		.write_config = gen_pcie_write_config,
		.rgr1_sw_init = gen_pcie_rgr1_sw_init,
	},
};

#if defined(__BIG_ENDIAN)
#define	DATA_ENDIAN		2	/* PCI->DDR inbound accesses */
#define MMIO_ENDIAN		2	/* CPU->PCI outbound accesses */
#else
#define	DATA_ENDIAN		0
#define MMIO_ENDIAN		0
#endif

/* negative return value indicates error */
static int mdio_read(void __iomem *base, u8 phyad, u8 regad)
{
	u32 data = ((phyad & 0xf) << 16)
		| (regad & 0x1f)
		| 0x100000;

	bpcie_writel(data, base + PCIE_RC_DL_MDIO_ADDR);
	bpcie_readl(base + PCIE_RC_DL_MDIO_ADDR);

	data = bpcie_readl(base + PCIE_RC_DL_MDIO_RD_DATA);
	if (!(data & 0x80000000)) {
		msleep(1);
		data = bpcie_readl(base + PCIE_RC_DL_MDIO_RD_DATA);
	}

	return (data & 0x80000000) ? (data & 0xffff) : -EIO;
}

/* negative return value indicates error */
static int mdio_write(void __iomem *base, u8 phyad, u8 regad, u16 wrdata)
{
	u32 data = ((phyad & 0xf) << 16) | (regad & 0x1f);

	bpcie_writel(data, base + PCIE_RC_DL_MDIO_ADDR);
	bpcie_readl(base + PCIE_RC_DL_MDIO_ADDR);

	bpcie_writel(0x80000000 | wrdata, base + PCIE_RC_DL_MDIO_WR_DATA);
	data = bpcie_readl(base + PCIE_RC_DL_MDIO_WR_DATA);
	if (!(data & 0x80000000)) {
		msleep(1);
		data = bpcie_readl(base + PCIE_RC_DL_MDIO_WR_DATA);
	}

	return (data & 0x80000000) ? 0 : -EIO;
}

/* configures device for ssc mode; negative return value indicates error */
static int set_ssc(void __iomem *base)
{
	int tmp;
	u16 wrdata;

	tmp = mdio_write(base, 0, 0x1f, 0x1100);
	if (tmp < 0)
		return tmp;

	tmp = mdio_read(base, 0, 2);
	if (tmp < 0)
		return tmp;

	wrdata = ((u16)tmp & 0x3fff) | 0xc000;
	tmp = mdio_write(base, 0, 2, wrdata);
	if (tmp < 0)
		return tmp;

	msleep(1);
	tmp = mdio_read(base, 0, 1);
	if (tmp < 0)
		return tmp;

	return 0;
}


/* returns 0 if in ssc mode, 1 if not, <0 on error */
static int is_ssc(void __iomem *base)
{
	int tmp = mdio_write(base, 0, 0x1f, 0x1100);

	if (tmp < 0)
		return tmp;
	tmp = mdio_read(base, 0, 1);
	if (tmp < 0)
		return tmp;
	return (tmp & 0xc00) == 0xc00 ? 0 : 1;
}

/* limits operation to a specific generation (1, 2, or 3) */
static void set_gen(void __iomem *base, int gen)
{
	wr_fld(base + PCIE_RC_CFG_PCIE_LINK_CAPABILITY, 0xf, 0, gen);
	wr_fld(base + PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL_2, 0xf, 0, gen);
}

static void set_pcie_outbound_win(void __iomem *base, unsigned int win,
				  u64 start, u64 len)
{
	u32 tmp;

	bpcie_writel((u32)(start) + MMIO_ENDIAN,
		     base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO + (win * 8));
	bpcie_writel((u32)(start >> 32),
		     base + PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI + (win * 8));
	tmp = ((((u32)start) >> 20) << 4)
		| (((((u32)start) + ((u32)len) - 1) >> 20) << 20);
	bpcie_writel(tmp, base +
		     PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT + (win * 4));
}

static int is_pcie_link_up(struct brcm_pcie *pcie)
{
	void __iomem *base = pcie->base;
	u32 val = bpcie_readl(base + PCIE_MISC_PCIE_STATUS);

	return ((val & 0x30) == 0x30) ? 1 : 0;
}

static int brcm_pcie_setup_early(struct brcm_pcie *pcie)
{
	void __iomem *base = pcie->base;
	struct resource_entry *win;
	unsigned int scb_size_val;
	struct resource *r;
	int i, ret;

	/* reset the bridge and the endpoint device
	 * field: PCIE_BRIDGE_SW_INIT = 1
	 */
	brcm_pcie_rgr1_sw_init(pcie, 0x00000002, 1, 1);

	/* field: PCIE_SW_PERST = 1 */
	brcm_pcie_rgr1_sw_init(pcie, 0x00000001, 0, 1);

	/* delay 100us */
	usleep_range(100, 1000);

	/* take the bridge out of reset
	 * field: PCIE_BRIDGE_SW_INIT = 0
	 */
	brcm_pcie_rgr1_sw_init(pcie, 0x00000002, 1, 0);

	/* Grab the PCIe hw revision number */
	pcie->rev = bpcie_readl(base + PCIE_MISC_REVISION) & 0xffff;

	/* enable SCB_MAX_BURST_SIZE | CSR_READ_UR_MODE | SCB_ACCESS_EN */
	if (pcie->type == GENERIC)
		bpcie_writel(0x81e03000, base + PCIE_MISC_MISC_CTRL);
	else
		bpcie_writel(0x00103000, base + PCIE_MISC_MISC_CTRL);

	i = 0;
	resource_list_for_each_entry(win, &pcie->resource) {
		r = win->res;

		if (!r->flags)
			continue;

		switch (resource_type(r)) {
		case IORESOURCE_MEM:
			/* Program PCIe outbound windows */
			set_pcie_outbound_win(base, i, r->start,
					      resource_size(r));
			i++;

			/* Request memory region resources the first time */
			if (!pcie->bridge_setup_done) {
				ret = devm_request_resource(pcie->dev,
							    &iomem_resource,
							    r);
				if (ret)
					return ret;
			}

			if (i == BRCM_NUM_PCI_OUT_WINS)
				dev_warn(pcie->dev,
					 "exceeded number of windows\n");
			break;

		default:
			/* No support for IORESOURCE_IO or IORESOURCE_BUS */
			continue;
		}
	}

	/* set up 4GB PCIE->SCB memory window on BAR2 */
	bpcie_writel(0x00000011, base + PCIE_MISC_RC_BAR2_CONFIG_LO);
	bpcie_writel(0x00000000, base + PCIE_MISC_RC_BAR2_CONFIG_HI);

	/* field: SCB0_SIZE, default = 0xf (1 GB) */
	scb_size_val = pcie->scb_size_vals[0] ? pcie->scb_size_vals[0] : 0xf;
	wr_fld(base + PCIE_MISC_MISC_CTRL, 0xf8000000, 27, scb_size_val);

	/* field: SCB1_SIZE, default = 0xf (1 GB) */
	if (pcie->num_memc > 1) {
		scb_size_val = pcie->scb_size_vals[1]
			? pcie->scb_size_vals[1] : 0xf;
		wr_fld(base + PCIE_MISC_MISC_CTRL, 0x07c00000,
		       22, scb_size_val);
	}

	/* field: SCB2_SIZE, default = 0xf (1 GB) */
	if (pcie->num_memc > 2) {
		scb_size_val = pcie->scb_size_vals[2]
			? pcie->scb_size_vals[2] : 0xf;
		wr_fld(base + PCIE_MISC_MISC_CTRL, 0x0000001f,
		       0, scb_size_val);
	}

	/* disable the PCIE->GISB memory window */
	bpcie_writel(0x00000000, base + PCIE_MISC_RC_BAR1_CONFIG_LO);

	/* disable the PCIE->SCB memory window */
	bpcie_writel(0x00000000, base + PCIE_MISC_RC_BAR3_CONFIG_LO);

	if (!pcie->suspended) {
		/* clear any interrupts we find on boot */
		bpcie_writel(0xffffffff, base + PCIE_INTR2_CPU_BASE + CLR);
		(void) bpcie_readl(base + PCIE_INTR2_CPU_BASE + CLR);
	}

	/* Mask all interrupts since we are not handling any yet */
	bpcie_writel(0xffffffff, base + PCIE_INTR2_CPU_BASE + MASK_SET);
	(void) bpcie_readl(base + PCIE_INTR2_CPU_BASE + MASK_SET);

	if (pcie->ssc)
		if (set_ssc(base))
			dev_err(pcie->dev, "error while configuring ssc mode\n");
	if (pcie->gen)
		set_gen(base, pcie->gen);

	/* take the EP device out of reset */
	/* field: PCIE_SW_PERST = 0 */
	brcm_pcie_rgr1_sw_init(pcie, 0x00000001, 0, 0);

	return 0;
}

static void brcm_pcie_turn_off(struct brcm_pcie *pcie)
{
	void __iomem *base = pcie->base;

	/* Reset endpoint device */
	brcm_pcie_rgr1_sw_init(pcie, 0x00000001, 0, 1);

	/* deassert request for L23 in case it was asserted */
	wr_fld_rb(base + PCIE_MISC_PCIE_CTRL, 0x1, 0, 0);

	/* SERDES_IDDQ = 1 */
	wr_fld_rb(base + PCIE_MISC_HARD_PCIE_HARD_DEBUG, 0x08000000,
		  27, 1);
	/* Shutdown PCIe bridge */
	brcm_pcie_rgr1_sw_init(pcie, 0x00000002, 1, 1);
}

static void brcm_pcie_enter_l23(struct brcm_pcie *pcie)
{
	void __iomem *base = pcie->base;
	int timeout = 1000;
	int l23;

	/* assert request for L23 */
	wr_fld_rb(base + PCIE_MISC_PCIE_CTRL, 0x1, 0, 1);
	do {
		/* poll L23 status */
		l23 = bpcie_readl(base + PCIE_MISC_PCIE_STATUS) & (1 << 6);
	} while (--timeout && !l23);

	if (!timeout)
		dev_err(pcie->dev, "failed to enter L23\n");
}

static int brcm_setup_pcie_bridge(struct brcm_pcie *pcie)
{
	static const char *link_speed[4] = { "???", "2.5", "5.0", "8.0" };
	void __iomem *base = pcie->base;
	const int limit = pcie->suspended ? 1000 : 100;
	struct clk *clk;
	unsigned int status;
	int i, j, ret;
	bool ssc_good = false;

	/* Give the RC/EP time to wake up, before trying to configure RC.
	 * Intermittently check status for link-up, up to a total of 100ms
	 * when we don't know if the device is there, and up to 1000ms if
	 * we do know the device is there.
	 */
	for (i = 1, j = 0; j < limit && !is_pcie_link_up(pcie); j += i, i = i*2)
		msleep(i + j > limit ? limit - j : i);

	if (!is_pcie_link_up(pcie)) {
		dev_info(pcie->dev, "link down\n");
		goto fail;
	}

	/* Attempt to enable MSI if we have an interrupt for it. */
	if (pcie->msi_irq > 0) {
		ret = brcm_pcie_enable_msi(pcie, pcie->num);
		if (ret < 0) {
			dev_err(pcie->dev, "failed to enable MSI support: %d\n",
				ret);
		}
	}

	/* For config space accesses on the RC, show the right class for
	 * a PCI-PCI bridge
	 */
	wr_fld_rb(base + PCIE_RC_CFG_PRIV1_ID_VAL3, 0x00ffffff, 0, 0x060400);

	status = bpcie_readl(base + PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL);

	if (pcie->ssc) {
		if (is_ssc(base) == 0)
			ssc_good = true;
		else
			dev_err(pcie->dev, "failed to enter SSC mode\n");
	}

	dev_info(pcie->dev, "link up, %s Gbps x%u %s\n",
		 link_speed[((status & 0x000f0000) >> 16) & 0x3],
		 (status & 0x03f00000) >> 20, ssc_good ? "(SSC)" : "(!SSC)");

	/* Enable configuration request retry (see pci_scan_device()) */
	/* field RC_CRS_EN = 1
	 */
	wr_fld(base + PCIE_RC_CFG_PCIE_ROOT_CAP_CONTROL, 0x00000010, 4, 1);

	/* PCIE->SCB endian mode for BAR ield ENDIAN_MODE_BAR2 = DATA_ENDIAN
	 */
	wr_fld_rb(base + PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1, 0x0000000c, 2,
		  DATA_ENDIAN);

	/* Refclk from RC should be gated with CLKREQ# input when ASPM L0s,L1
	 * is enabled =>  setting the CLKREQ_DEBUG_ENABLE field to 1.
	 */
	wr_fld_rb(base + PCIE_MISC_HARD_PCIE_HARD_DEBUG, 0x00000002, 1, 1);

	/* Add bogus IO resource structure so that pcibios_init_resources()
	 * does not allocate the same IO region for different domains
	 */

	pcie->bridge_setup_done = true;

	return 0;
fail:
	if (IS_ENABLED(CONFIG_PM))
		brcm_pcie_turn_off(pcie);

	clk = pcie->clk;
	if (pcie->suspended)
		clk_disable(clk);
	else {
		clk_disable_unprepare(clk);
		clk_put(clk);
	}

	pcie->bridge_setup_done = false;

	return -ENODEV;
}

#ifdef CONFIG_PM_SLEEP
static int brcm_pcie_suspend(struct device *dev)
{
	struct brcm_pcie *pcie = dev_get_drvdata(dev);

	if (!pcie->bridge_setup_done)
		return 0;

	brcm_pcie_enter_l23(pcie);
	brcm_pcie_turn_off(pcie);
	clk_disable(pcie->clk);
	pcie->suspended = true;

	return 0;
}

static int brcm_pcie_resume(struct device *dev)
{
	struct brcm_pcie *pcie = dev_get_drvdata(dev);

	if (!pcie->bridge_setup_done)
		return 0;

	/* Take bridge out of reset so we can access the SERDES reg */
	brcm_pcie_rgr1_sw_init(pcie, 0x00000002, 1, 0);

	/* SERDES_IDDQ = 0 */
	wr_fld_rb(pcie->base + PCIE_MISC_HARD_PCIE_HARD_DEBUG, 0x08000000,
		  27, 0);
	/* wait for serdes to be stable */
	usleep_range(100, 1000);

	brcm_pcie_setup_early(pcie);

	brcm_setup_pcie_bridge(pcie);
	pcie->suspended = false;

	return 0;
}

static const struct dev_pm_ops brcm_pcie_pm_ops = {
	.suspend_noirq = brcm_pcie_suspend,
	.resume_noirq = brcm_pcie_resume,
};
#else
#define brcm_pcie_pm_ops	NULL
#endif /* CONFIG_PM_SLEEP */

static int cfg_index(int busnr, int devfn, int reg)
{
	return ((PCI_SLOT(devfn) & 0x1f) << PCI_SLOT_SHIFT)
		| ((PCI_FUNC(devfn) & 0x07) << PCI_FUNC_SHIFT)
		| (busnr << PCI_BUSNUM_SHIFT)
		| (reg & ~3);
}

static int brcm_pcie_write_config(struct pci_bus *bus, unsigned int devfn,
				  int where, int size, u32 data)
{
	struct brcm_pcie *pcie = bus->sysdata;
	u32 val = 0, mask, shift;
	void __iomem *base;
	bool rc_access;
	int idx;

	if (!is_pcie_link_up(pcie))
		return PCIBIOS_DEVICE_NOT_FOUND;

	base = pcie->base;

	rc_access = !!pci_is_root_bus(bus);

	idx = cfg_index(bus->number, devfn, where);
	WARN_ON(((where & 3) + size) > 4);

	if (rc_access && PCI_SLOT(devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (size < 4) {
		/* partial word - read, modify, write */
		if (rc_access)
			val = bpcie_readl(base + (where & ~3));
		else
			val = brcm_pcie_ll_read_config(pcie, idx);
	}

	shift = (where & 3) << 3;
	mask = (0xffffffff >> ((4 - size) << 3)) << shift;
	val = (val & ~mask) | ((data << shift) & mask);

	if (rc_access) {
		bpcie_writel(val, base + (where & ~3));
		bpcie_readl(base + (where & ~3));
	} else {
		brcm_pcie_ll_write_config(pcie, idx, val);
	}
	return PCIBIOS_SUCCESSFUL;
}

static int brcm_pcie_read_config(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 *data)
{
	struct brcm_pcie *pcie = bus->sysdata;
	u32 val, mask, shift;
	void __iomem *base;
	bool rc_access;
	int idx;

	if (!is_pcie_link_up(pcie))
		return PCIBIOS_DEVICE_NOT_FOUND;

	base = pcie->base;

	rc_access = !!pci_is_root_bus(bus);
	idx = cfg_index(bus->number, devfn, where);
	WARN_ON(((where & 3) + size) > 4);

	if (rc_access && PCI_SLOT(devfn)) {
		*data = 0xffffffff;
		return PCIBIOS_FUNC_NOT_SUPPORTED;
	}

	if (rc_access)
		val = bpcie_readl(base + (where & ~3));
	else
		val = brcm_pcie_ll_read_config(pcie, idx);

	shift = (where & 3) << 3;
	mask = (0xffffffff >> ((4 - size) << 3)) << shift;
	*data = (val & mask) >> shift;

	return PCIBIOS_SUCCESSFUL;
}

static const struct of_device_id brcm_pcie_match[] = {
	{ .compatible = "brcm,bcm7425-pcie", .data = &bcm7425_cfg },
	{ .compatible = "brcm,bcm7435-pcie", .data = &bcm7435_cfg },
	{ .compatible = "brcm,bcm7445-pcie", .data = &generic_cfg },
	{},
};
MODULE_DEVICE_TABLE(of, brcm_pcie_match);

static struct pci_ops brcm_pcie_ops = {
	.read = brcm_pcie_read_config,
	.write = brcm_pcie_write_config,
};

static int brcm_pcie_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	const u32 *log2_scb_sizes, *dma_ranges;
	const struct brcm_pcie_cfg_data *data;
	const struct of_device_id *of_id;
	struct brcm_pcie *pcie;
	void __iomem *base;
	struct resource *r;
	int i, rlen, ret;
	u32 tmp;

	pcie = devm_kzalloc(&pdev->dev, sizeof(struct brcm_pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	of_id = of_match_node(brcm_pcie_match, dn);
	if (!of_id)
		return -EINVAL;

	data = of_id->data;
	pcie->type = data->type;
	pcie->ops = &data->ops;

	platform_set_drvdata(pdev, pcie);

	INIT_LIST_HEAD(&pcie->resource);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(base))
		return PTR_ERR(base);

	ret = of_alias_get_id(dn, "pcie");
	if (ret >= 0)
		pcie->num = ret;

	pcie->clk = devm_clk_get(&pdev->dev, "pcie");
	if (IS_ERR(pcie->clk)) {
		dev_err(&pdev->dev, "could not get clock\n");
		pcie->clk = NULL;
	}

	ret = clk_prepare_enable(pcie->clk);
	if (ret) {
		dev_err(&pdev->dev, "could not enable clock\n");
		return ret;
	}

	pcie->dn = dn;
	pcie->base = base;
	pcie->dev = &pdev->dev;
	pcie->dev->of_node = dn;
	pcie->gen = 0;

	ret = of_property_read_u32(dn, "brcm,gen", &tmp);
	if (ret == 0) {
		if (tmp > 0 && tmp < 3)
			pcie->gen = (int)tmp;
		else
			dev_warn(pcie->dev, "bad DT value for prop 'brcm,gen");
	} else if (ret != -EINVAL) {
		dev_warn(pcie->dev, "error reading DT prop 'brcm,gen");
	}

	pcie->ssc = of_property_read_bool(dn, "brcm,ssc");

	/* Get the value for the log2 of the scb sizes. Subtract 15 from
	 * each because the target register field has 0==disabled and 1==6KB.
	 */
	log2_scb_sizes = of_get_property(dn, "brcm,log2-scb-sizes", &rlen);
	if (log2_scb_sizes) {
		for (i = 0; i < rlen / sizeof(u32); i++) {
			pcie->scb_size_vals[i]
				= (int)of_read_number(log2_scb_sizes + i, 1)
					- 15;
			pcie->num_memc++;
		}
	}

	/* Look for the dma-ranges property.  If it exists, issue a warning
	 * as PCIe drivers may not work.  This is because the identity
	 * mapping between system memory and PCIe space is not preserved,
	 * and we need Linux to massage the dma_addr_t values it gets
	 * from dma memory allocation.  This functionality will be added
	 * in the near future.
	 */
	dma_ranges = of_get_property(dn, "dma-ranges", &rlen);
	if (dma_ranges != NULL)
		dev_warn(pcie->dev, "no identity map; PCI drivers may fail");

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		ret = irq_of_parse_and_map(pdev->dev.of_node, 1);
		if (ret == 0)
			dev_warn(pcie->dev, "cannot get msi intr; MSI disabled\n");
		else
			pcie->msi_irq = ret;
	}

	ret = of_pci_get_host_bridge_resources(dn, 0, 0xff,
					       &pcie->resource, NULL);
	if (ret) {
		dev_err(pcie->dev, "ranges parsing failed\n");
		return ret;
	}

	ret = brcm_pcie_setup_early(pcie);
	if (ret)
		goto out_err_clk;

	/* If setup bridge fails, it cleans up behind itself */
	ret = brcm_setup_pcie_bridge(pcie);
	if (ret)
		goto out_err;

	pcie->bus = pci_scan_root_bus(pcie->dev, pcie->num, &brcm_pcie_ops,
				      pcie, &pcie->resource);
	if (!pcie->bus) {
		ret = -ENOMEM;
		goto out_err_bus;
	}

	if (IS_ENABLED(CONFIG_PCI_MSI))
		brcm_pcie_msi_chip_set(pcie);

	pci_bus_size_bridges(pcie->bus);
	pci_bus_assign_resources(pcie->bus);

	pci_fixup_irqs(pci_common_swizzle, of_irq_parse_and_map_pci);
	pci_bus_add_devices(pcie->bus);

	return 0;

out_err_bus:
	brcm_pcie_enter_l23(pcie);
	brcm_pcie_turn_off(pcie);
out_err_clk:
	clk_disable_unprepare(pcie->clk);
out_err:
	return ret;
}

static int brcm_pcie_remove(struct platform_device *pdev)
{
	return brcm_pcie_suspend(&pdev->dev);
}

static struct platform_driver brcm_pcie_driver = {
	.probe = brcm_pcie_probe,
	.remove = brcm_pcie_remove,
	.driver = {
		.name = "brcm-pcie",
		.owner = THIS_MODULE,
		.of_match_table = brcm_pcie_match,
		.pm = &brcm_pcie_pm_ops,
	},
};
module_platform_driver(brcm_pcie_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Broadcom STB PCIE RC driver");
MODULE_AUTHOR("Broadcom");
