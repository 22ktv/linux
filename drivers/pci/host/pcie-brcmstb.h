#ifndef __PCIE_BRCMSTB_H
#define __PCIE_BRCMSTB_H

#include <linux/io.h>

/* Broadcom PCIE Offsets */
#define PCIE_RC_CFG_PCIE_LINK_CAPABILITY		0x00b8
#define PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL		0x00bc
#define PCIE_RC_CFG_PCIE_ROOT_CAP_CONTROL		0x00c8
#define PCIE_RC_CFG_PCIE_LINK_STATUS_CONTROL_2		0x00dc
#define PCIE_RC_CFG_VENDOR_VENDOR_SPECIFIC_REG1		0x0188
#define PCIE_RC_CFG_PRIV1_ID_VAL3			0x043c
#define PCIE_RC_DL_MDIO_ADDR				0x1100
#define PCIE_RC_DL_MDIO_WR_DATA				0x1104
#define PCIE_RC_DL_MDIO_RD_DATA				0x1108
#define PCIE_MISC_MISC_CTRL				0x4008
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_LO		0x400c
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_HI		0x4010
#define PCIE_MISC_RC_BAR1_CONFIG_LO			0x402c
#define PCIE_MISC_RC_BAR1_CONFIG_HI			0x4030
#define PCIE_MISC_RC_BAR2_CONFIG_LO			0x4034
#define PCIE_MISC_RC_BAR2_CONFIG_HI			0x4038
#define PCIE_MISC_RC_BAR3_CONFIG_LO			0x403c
#define PCIE_MISC_RC_BAR3_CONFIG_HI			0x4040
#define PCIE_MISC_MSI_BAR_CONFIG_LO			0x4044
#define PCIE_MISC_MSI_BAR_CONFIG_HI			0x4048
#define PCIE_MISC_MSI_DATA_CONFIG			0x404c
#define PCIE_MISC_PCIE_CTRL				0x4064
#define PCIE_MISC_PCIE_STATUS				0x4068
#define PCIE_MISC_REVISION				0x406c
#define PCIE_MISC_CPU_2_PCIE_MEM_WIN0_BASE_LIMIT	0x4070
#define PCIE_MISC_HARD_PCIE_HARD_DEBUG			0x4204
#define PCIE_INTR2_CPU_BASE				0x4300
#define PCIE_MSI_INTR2_BASE				0x4500

#define PCIE_RGR1_SW_INIT_1				0x9210
#define PCIE_EXT_CFG_INDEX				0x9000
#define PCIE_EXT_CFG_DATA				0x9004

/* BCM7425 specific register offsets */
#define BCM7425_PCIE_RGR1_SW_INIT_1			0x8010
#define BCM7425_PCIE_EXT_CFG_INDEX			0x8300
#define BCM7425_PCIE_EXT_CFG_DATA			0x8304

#define PCI_BUSNUM_SHIFT		20
#define PCI_SLOT_SHIFT			15
#define PCI_FUNC_SHIFT			12

#define BRCM_NUM_PCI_OUT_WINS		4
#define BRCM_MAX_SCB			4

/* Offsets from PCIE_INTR2_CPU_BASE and PCIE_MSI_INTR2_BASE */
#define STATUS				0x0
#define SET				0x4
#define CLR				0x8
#define MASK_STATUS			0xc
#define MASK_SET			0x10
#define MASK_CLR			0x14

enum brcm_pcie_type {
	BCM7425,
	BCM7435,
	GENERIC,
};

struct brcm_pcie;

/* Chip-specific PCIe operations (read/write config and reset) */
struct brcm_pcie_ll_ops {
	u32 (*read_config)(struct brcm_pcie *pcie, int cfg_idx);
	void (*write_config)(struct brcm_pcie *pcie, int cfg_idx, u32 val);
	void (*rgr1_sw_init)(struct brcm_pcie *pcie, u32 mask,
			     int shift, u32 val);
};

struct brcm_pcie_cfg_data {
	const enum brcm_pcie_type type;
	const struct brcm_pcie_ll_ops ops;
};

struct brcm_msi;

/* Internal Bus Controller Information.*/
struct brcm_pcie {
	void __iomem		*base;
	bool			suspended;
	struct clk		*clk;
	struct device_node	*dn;
	bool			ssc;
	int			gen;
	int			scb_size_vals[BRCM_MAX_SCB];
	struct pci_bus		*bus;
	struct device		*dev;
	struct list_head	resource;
	int			msi_irq;
	struct brcm_msi		*msi;
	unsigned int		rev;
	unsigned int		num;
	bool			bridge_setup_done;
	enum brcm_pcie_type	type;
	const struct brcm_pcie_ll_ops *ops;
	unsigned int		num_memc;
};

/* Helper functions to access read/write config space and software init which
 * are chip-specific
 */
static inline u32 brcm_pcie_ll_read_config(struct brcm_pcie *pcie, int cfg_idx)
{
	return pcie->ops->read_config(pcie, cfg_idx);
}

static inline void brcm_pcie_ll_write_config(struct brcm_pcie *pcie,
					     int cfg_idx, u32 val)
{
	pcie->ops->write_config(pcie, cfg_idx, val);
}

static inline void brcm_pcie_rgr1_sw_init(struct brcm_pcie *pcie, u32 mask,
					  int shift, u32 val)
{
	pcie->ops->rgr1_sw_init(pcie, mask, shift, val);
}

/*
 * MIPS endianness is configured by boot strap, which also reverses all
 * bus endianness (i.e., big-endian CPU + big endian bus ==> native
 * endian I/O).
 *
 * Other architectures (e.g., ARM) either do not support big endian, or
 * else leave I/O in little endian mode.
 */
static inline u32 bpcie_readl(void __iomem *base)
{
	if (IS_ENABLED(CONFIG_MIPS))
		return __raw_readl(base);
	else
		return readl(base);
}

static inline void bpcie_writel(u32 val, void __iomem *base)
{
	if (IS_ENABLED(CONFIG_MIPS))
		__raw_writel(val, base);
	else
		writel(val, base);
}

#ifdef CONFIG_PCIE_BRCMSTB_MSI
int brcm_pcie_enable_msi(struct brcm_pcie *pcie, int nr);
void brcm_pcie_msi_chip_set(struct brcm_pcie *pcie);
#else
static inline int brcm_pcie_enable_msi(struct brcm_pcie *pcie, int nr)
{
	return 0;
}
static inline void brcm_pcie_msi_chip_set(struct brcm_pcie *pcie) { }
#endif

#endif /* __PCIE_BRCMSTB_H */
