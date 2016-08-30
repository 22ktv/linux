/*
 * Copyright (C) 2014-2016 Broadcom
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the project nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE PROJECT AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * This module is used by both the bootloader and Linux and
 * contains USB initialization for power up and S3 resume.
 */

#include <linux/delay.h>
#include <linux/io.h>

#include <linux/soc/brcmstb/brcmstb.h>
#include "phy-brcm-usb-init.h"

/* Register definitions for the USB CTRL block */
#define USB_CTRL_SETUP			0x00
#define   USB_CTRL_SETUP_IOC_MASK			0x00000010
#define   USB_CTRL_SETUP_IPP_MASK			0x00000020
#define   USB_CTRL_SETUP_BABO_MASK			0x00000001
#define   USB_CTRL_SETUP_FNHW_MASK			0x00000002
#define   USB_CTRL_SETUP_FNBO_MASK			0x00000004
#define   USB_CTRL_SETUP_WABO_MASK			0x00000008
#define   USB_CTRL_SETUP_scb1_en_MASK			0x00004000 /* option */
#define   USB_CTRL_SETUP_scb2_en_MASK			0x00008000 /* option */
#define   USB_CTRL_SETUP_ss_ehci64bit_en_MASK		0x00020000 /* option */
#define   USB_CTRL_SETUP_ss_ehci64bit_en_var_MASK	0x00010000 /* option */
#define   USB_CTRL_SETUP_strap_ipp_sel_MASK		0x02000000 /* option */
#define   USB_CTRL_SETUP_OC3_DISABLE_MASK		0xc0000000 /* option */
#define USB_CTRL_PLL_CTL		0x04
#define   USB_CTRL_PLL_CTL_PLL_SUSPEND_EN_MASK		0x08000000
#define   USB_CTRL_PLL_CTL_PLL_RESETB_MASK		0x40000000
#define   USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK		0x80000000 /* option */
#define USB_CTRL_EBRIDGE		0x0c
#define   USB_CTRL_EBRIDGE_ESTOP_SCB_REQ_MASK		0x00020000 /* option */
#define USB_CTRL_MDIO			0x14
#define USB_CTRL_MDIO2			0x18
#define USB_CTRL_UTMI_CTL_1		0x2c
#define   USB_CTRL_UTMI_CTL_1_POWER_UP_FSM_EN_MASK	0x00000800
#define   USB_CTRL_UTMI_CTL_1_POWER_UP_FSM_EN_P1_MASK	0x08000000
#define USB_CTRL_USB_PM			0x34
#define   USB_CTRL_USB_PM_bdc_soft_resetb_MASK		0x00800000 /* option */
#define   USB_CTRL_USB_PM_xhc_soft_resetb_MASK		0x00400000 /* option */
#define   USB_CTRL_USB_PM_xhc_soft_resetb_var_MASK	0x40000000 /* option */
#define   USB_CTRL_USB_PM_USB_PWRDN_MASK		0x80000000 /* option */
#define USB_CTRL_USB30_CTL1		0x60
#define   USB_CTRL_USB30_CTL1_phy3_pll_seq_start_MASK	0x00000010
#define   USB_CTRL_USB30_CTL1_phy3_resetb_MASK		0x00010000
#define   USB_CTRL_USB30_CTL1_xhc_soft_resetb_MASK	0x00020000 /* option */
#define   USB_CTRL_USB30_CTL1_usb3_ioc_MASK		0x10000000 /* option */
#define   USB_CTRL_USB30_CTL1_usb3_ipp_MASK		0x20000000 /* option */
#define USB_CTRL_USB30_PCTL		0x70
#define   USB_CTRL_USB30_PCTL_PHY3_SOFT_RESETB_MASK	0x00000002
#define   USB_CTRL_USB30_PCTL_PHY3_SOFT_RESETB_P1_MASK	0x00020000
#define USB_CTRL_USB_DEVICE_CTL1	0x90
#define   USB_CTRL_USB_DEVICE_CTL1_port_mode_MASK	0x00000003 /* option */

/* Register definitions for the XHCI EC block */
#define USB_XHCI_EC_IRAADR 0x658
#define USB_XHCI_EC_IRADAT 0x65c

enum brcm_family_type {
	BRCM_FAMILY_DEFAULT,
	BRCM_FAMILY_3390A0,
	BRCM_FAMILY_7250B0,
	BRCM_FAMILY_7271A0,
	BRCM_FAMILY_7364A0,
	BRCM_FAMILY_7366C0,
	BRCM_FAMILY_74371A0,
	BRCM_FAMILY_7439B0,
	BRCM_FAMILY_7445D0,
	BRCM_FAMILY_COUNT,
};

enum {
	USB_CTRL_SETUP_scb1_en_SELECTOR,
	USB_CTRL_SETUP_scb2_en_SELECTOR,
	USB_CTRL_SETUP_ss_ehci64bit_en_SELECTOR,
	USB_CTRL_SETUP_strap_ipp_sel_SELECTOR,
	USB_CTRL_SETUP_OC3_DISABLE_SELECTOR,
	USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_SELECTOR,
	USB_CTRL_EBRIDGE_ESTOP_SCB_REQ_SELECTOR,
	USB_CTRL_USB_PM_bdc_soft_resetb_SELECTOR,
	USB_CTRL_USB_PM_xhc_soft_resetb_SELECTOR,
	USB_CTRL_USB_PM_USB_PWRDN_SELECTOR,
	USB_CTRL_USB30_CTL1_xhc_soft_resetb_SELECTOR,
	USB_CTRL_USB30_CTL1_usb3_ioc_SELECTOR,
	USB_CTRL_USB30_CTL1_usb3_ipp_SELECTOR,
	USB_CTRL_USB_DEVICE_CTL1_port_mode_SELECTOR,
	USB_CTRL_SELECTOR_COUNT,
};

#define USB_CTRL_REG(base, reg)	((void *)base + USB_CTRL_##reg)
#define USB_XHCI_EC_REG(base, reg) ((void *)base + USB_XHCI_EC_##reg)
#define USB_CTRL_MASK(reg, field) \
	USB_CTRL_##reg##_##field##_MASK
#define USB_CTRL_MASK_FAMILY(reg, field) \
	(usb_reg_bits_map[USB_CTRL_##reg##_##field##_SELECTOR])

#define USB_CTRL_SET_FAMILY(base, reg, field)	\
	usb_ctrl_set_family(USB_CTRL_REG(base, reg),	\
			USB_CTRL_##reg##_##field##_SELECTOR)
#define USB_CTRL_UNSET_FAMILY(base, reg, field)	\
	usb_ctrl_unset_family(USB_CTRL_REG(base, reg),	\
		USB_CTRL_##reg##_##field##_SELECTOR)
#define USB_CTRL_SET(base, reg, field)	\
	usb_ctrl_set(USB_CTRL_REG(base, reg),	\
		USB_CTRL_##reg##_##field##_MASK)
#define USB_CTRL_UNSET(base, reg, field)	\
	usb_ctrl_unset(USB_CTRL_REG(base, reg),	\
		USB_CTRL_##reg##_##field##_MASK)

#define MDIO_USB2	0
#define MDIO_USB3	(1 << 31)

#define USB_CTRL_SETUP_CONDITIONAL_BITS (	\
		USB_CTRL_MASK(SETUP, BABO) |	\
		USB_CTRL_MASK(SETUP, FNHW) |	\
		USB_CTRL_MASK(SETUP, FNBO) |	\
		USB_CTRL_MASK(SETUP, WABO) |	\
		USB_CTRL_MASK(SETUP, IOC)  |	\
		USB_CTRL_MASK(SETUP, IPP))

#ifdef __LITTLE_ENDIAN
#define ENDIAN_SETTINGS (			\
		USB_CTRL_MASK(SETUP, BABO) |	\
		USB_CTRL_MASK(SETUP, FNHW))
#else
#define ENDIAN_SETTINGS (			\
		USB_CTRL_MASK(SETUP, FNHW) |	\
		USB_CTRL_MASK(SETUP, FNBO) |	\
		USB_CTRL_MASK(SETUP, WABO))
#endif

struct id_to_type {
	uint32_t id;
	int type;
};

static const struct id_to_type id_to_type_table[] = {
	{ 0x33900000, BRCM_FAMILY_3390A0 },
	{ 0x33900010, BRCM_FAMILY_3390A0 },
	{ 0x72500010, BRCM_FAMILY_7250B0 },
	{ 0x72710000, BRCM_FAMILY_7271A0 },
	{ 0x72680000, BRCM_FAMILY_7271A0 },
	{ 0x73640000, BRCM_FAMILY_7364A0 },
	{ 0x73640010, BRCM_FAMILY_7364A0 },
	{ 0x73660020, BRCM_FAMILY_7366C0 },
	{ 0x74371000, BRCM_FAMILY_74371A0 },
	{ 0x74390010, BRCM_FAMILY_7439B0 },
	{ 0x74450030, BRCM_FAMILY_7445D0 },
	{ 0x74450040, BRCM_FAMILY_7445D0 },
};

static const uint32_t *usb_reg_bits_map;
static enum brcm_family_type my_family;

static const uint32_t
usb_reg_bits_map_table[BRCM_FAMILY_COUNT][USB_CTRL_SELECTOR_COUNT] = {
	/* DEFAULT (default to latest chip, currently the 7271) */
	[BRCM_FAMILY_DEFAULT] = {
		USB_CTRL_SETUP_scb1_en_MASK,
		USB_CTRL_SETUP_scb2_en_MASK,
		USB_CTRL_SETUP_ss_ehci64bit_en_var_MASK,
		0, /* USB_CTRL_SETUP_strap_ipp_sel_MASK */
		0, /* USB_CTRL_SETUP_OC3_DISABLE_MASK */
		USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK,
		0, /* USB_CTRL_EBRIDGE_ESTOP_SCB_REQ_MASK */
		0, /* USB_CTRL_USB_PM_bdc_soft_resetb_MASK */
		0, /* USB_CTRL_USB_PM_xhc_soft_resetb_MASK */
		0, /* USB_CTRL_USB_PM_USB_PWRDN_MASK */
		USB_CTRL_USB30_CTL1_xhc_soft_resetb_MASK,
		USB_CTRL_USB30_CTL1_usb3_ioc_MASK,
		USB_CTRL_USB30_CTL1_usb3_ipp_MASK,
		0, /* USB_CTRL_USB_DEVICE_CTL1_port_mode_MASK */
	},
	/* 3390B0 */
	[BRCM_FAMILY_3390A0] = {
		USB_CTRL_SETUP_scb1_en_MASK,
		USB_CTRL_SETUP_scb2_en_MASK,
		USB_CTRL_SETUP_ss_ehci64bit_en_MASK,
		USB_CTRL_SETUP_strap_ipp_sel_MASK,
		USB_CTRL_SETUP_OC3_DISABLE_MASK,
		0, /* USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK */
		USB_CTRL_EBRIDGE_ESTOP_SCB_REQ_MASK,
		0, /* USB_CTRL_USB_PM_bdc_soft_resetb_MASK */
		USB_CTRL_USB_PM_xhc_soft_resetb_MASK,
		USB_CTRL_USB_PM_USB_PWRDN_MASK,
		0, /* USB_CTRL_USB30_CTL1_xhc_soft_resetb_MASK */
		0, /* USB_CTRL_USB30_CTL1_usb3_ioc_MASK */
		0, /* USB_CTRL_USB30_CTL1_usb3_ipp_MASK */
		USB_CTRL_USB_DEVICE_CTL1_port_mode_MASK,
	},
	/* 7250b0 */
	[BRCM_FAMILY_7250B0] = {
		USB_CTRL_SETUP_scb1_en_MASK,
		USB_CTRL_SETUP_scb2_en_MASK,
		USB_CTRL_SETUP_ss_ehci64bit_en_MASK,
		0, /* USB_CTRL_SETUP_strap_ipp_sel_MASK */
		USB_CTRL_SETUP_OC3_DISABLE_MASK,
		USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK,
		USB_CTRL_EBRIDGE_ESTOP_SCB_REQ_MASK,
		0, /* USB_CTRL_USB_PM_bdc_soft_resetb_MASK */
		USB_CTRL_USB_PM_xhc_soft_resetb_var_MASK,
		0, /* USB_CTRL_USB_PM_USB_PWRDN_MASK */
		0, /* USB_CTRL_USB30_CTL1_xhc_soft_resetb_MASK */
		0, /* USB_CTRL_USB30_CTL1_usb3_ioc_MASK */
		0, /* USB_CTRL_USB30_CTL1_usb3_ipp_MASK */
		0, /* USB_CTRL_USB_DEVICE_CTL1_port_mode_MASK */
	},
	/* 7271a0 */
	[BRCM_FAMILY_7271A0] = {
		0, /* USB_CTRL_SETUP_scb1_en_MASK */
		0, /* USB_CTRL_SETUP_scb2_en_MASK */
		USB_CTRL_SETUP_ss_ehci64bit_en_MASK,
		USB_CTRL_SETUP_strap_ipp_sel_MASK,
		USB_CTRL_SETUP_OC3_DISABLE_MASK,
		0, /* USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK */
		USB_CTRL_EBRIDGE_ESTOP_SCB_REQ_MASK,
		USB_CTRL_USB_PM_bdc_soft_resetb_MASK,
		USB_CTRL_USB_PM_xhc_soft_resetb_MASK,
		USB_CTRL_USB_PM_USB_PWRDN_MASK,
		0, /* USB_CTRL_USB30_CTL1_xhc_soft_resetb_MASK */
		0, /* USB_CTRL_USB30_CTL1_usb3_ioc_MASK */
		0, /* USB_CTRL_USB30_CTL1_usb3_ipp_MASK */
		USB_CTRL_USB_DEVICE_CTL1_port_mode_MASK,
	},
	/* 7364a0 */
	[BRCM_FAMILY_7364A0] = {
		USB_CTRL_SETUP_scb1_en_MASK,
		USB_CTRL_SETUP_scb2_en_MASK,
		USB_CTRL_SETUP_ss_ehci64bit_en_MASK,
		0, /* USB_CTRL_SETUP_strap_ipp_sel_MASK */
		USB_CTRL_SETUP_OC3_DISABLE_MASK,
		USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK,
		USB_CTRL_EBRIDGE_ESTOP_SCB_REQ_MASK,
		0, /* USB_CTRL_USB_PM_bdc_soft_resetb_MASK */
		USB_CTRL_USB_PM_xhc_soft_resetb_var_MASK,
		0, /* USB_CTRL_USB_PM_USB_PWRDN_MASK */
		0, /* USB_CTRL_USB30_CTL1_xhc_soft_resetb_MASK */
		0, /* USB_CTRL_USB30_CTL1_usb3_ioc_MASK */
		0, /* USB_CTRL_USB30_CTL1_usb3_ipp_MASK */
		0, /* USB_CTRL_USB_DEVICE_CTL1_port_mode_MASK */
	},
	/* 7366c0 */
	[BRCM_FAMILY_7366C0] = {
		USB_CTRL_SETUP_scb1_en_MASK,
		USB_CTRL_SETUP_scb2_en_MASK,
		USB_CTRL_SETUP_ss_ehci64bit_en_MASK,
		0, /* USB_CTRL_SETUP_strap_ipp_sel_MASK */
		USB_CTRL_SETUP_OC3_DISABLE_MASK,
		0, /* USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK */
		USB_CTRL_EBRIDGE_ESTOP_SCB_REQ_MASK,
		0, /* USB_CTRL_USB_PM_bdc_soft_resetb_MASK */
		USB_CTRL_USB_PM_xhc_soft_resetb_var_MASK,
		USB_CTRL_USB_PM_USB_PWRDN_MASK,
		0, /* USB_CTRL_USB30_CTL1_xhc_soft_resetb_MASK */
		0, /* USB_CTRL_USB30_CTL1_usb3_ioc_MASK */
		0, /* USB_CTRL_USB30_CTL1_usb3_ipp_MASK */
		0, /* USB_CTRL_USB_DEVICE_CTL1_port_mode_MASK */
	},
	/* 74371A0 */
	[BRCM_FAMILY_74371A0] = {
		USB_CTRL_SETUP_scb1_en_MASK,
		USB_CTRL_SETUP_scb2_en_MASK,
		USB_CTRL_SETUP_ss_ehci64bit_en_var_MASK,
		0, /* USB_CTRL_SETUP_strap_ipp_sel_MASK */
		0, /* USB_CTRL_SETUP_OC3_DISABLE_MASK */
		USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK,
		0, /* USB_CTRL_EBRIDGE_ESTOP_SCB_REQ_MASK */
		0, /* USB_CTRL_USB_PM_bdc_soft_resetb_MASK */
		0, /* USB_CTRL_USB_PM_xhc_soft_resetb_MASK */
		0, /* USB_CTRL_USB_PM_USB_PWRDN_MASK */
		USB_CTRL_USB30_CTL1_xhc_soft_resetb_MASK,
		USB_CTRL_USB30_CTL1_usb3_ioc_MASK,
		USB_CTRL_USB30_CTL1_usb3_ipp_MASK,
		0, /* USB_CTRL_USB_DEVICE_CTL1_port_mode_MASK */
	},
	/* 7439B0 */
	[BRCM_FAMILY_7439B0] = {
		USB_CTRL_SETUP_scb1_en_MASK,
		USB_CTRL_SETUP_scb2_en_MASK,
		USB_CTRL_SETUP_ss_ehci64bit_en_MASK,
		USB_CTRL_SETUP_strap_ipp_sel_MASK,
		USB_CTRL_SETUP_OC3_DISABLE_MASK,
		0, /* USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK */
		0, /* USB_CTRL_EBRIDGE_ESTOP_SCB_REQ_MASK */
		USB_CTRL_USB_PM_bdc_soft_resetb_MASK,
		USB_CTRL_USB_PM_xhc_soft_resetb_MASK,
		USB_CTRL_USB_PM_USB_PWRDN_MASK,
		0, /* USB_CTRL_USB30_CTL1_xhc_soft_resetb_MASK */
		0, /* USB_CTRL_USB30_CTL1_usb3_ioc_MASK */
		0, /* USB_CTRL_USB30_CTL1_usb3_ipp_MASK */
		USB_CTRL_USB_DEVICE_CTL1_port_mode_MASK,
	},
	/* 7445d0 */
	[BRCM_FAMILY_7445D0] = {
		USB_CTRL_SETUP_scb1_en_MASK,
		USB_CTRL_SETUP_scb2_en_MASK,
		USB_CTRL_SETUP_ss_ehci64bit_en_var_MASK,
		0, /* USB_CTRL_SETUP_strap_ipp_sel_MASK */
		USB_CTRL_SETUP_OC3_DISABLE_MASK,
		USB_CTRL_PLL_CTL_PLL_IDDQ_PWRDN_MASK,
		0, /* USB_CTRL_EBRIDGE_ESTOP_SCB_REQ_MASK */
		0, /* USB_CTRL_USB_PM_bdc_soft_resetb_MASK */
		0, /* USB_CTRL_USB_PM_xhc_soft_resetb_MASK */
		0, /* USB_CTRL_USB_PM_USB_PWRDN_MASK */
		USB_CTRL_USB30_CTL1_xhc_soft_resetb_MASK,
		0, /* USB_CTRL_USB30_CTL1_usb3_ioc_MASK */
		0, /* USB_CTRL_USB30_CTL1_usb3_ipp_MASK */
		0, /* USB_CTRL_USB_DEVICE_CTL1_port_mode_MASK */
	},
};

static inline uint32_t brcmusb_readl(void __iomem *addr)
{
	/*
	 * MIPS endianness is configured by boot strap, which also reverses all
	 * bus endianness (i.e., big-endian CPU + big endian bus ==> native
	 * endian I/O).
	 *
	 * Other architectures (e.g., ARM) either do not support big endian, or
	 * else leave I/O in little endian mode.
	 */
	if (IS_ENABLED(CONFIG_MIPS) && IS_ENABLED(__BIG_ENDIAN))
		return __raw_readl(addr);
	else
		return readl_relaxed(addr);
}

static inline void brcmusb_writel(uint32_t val, void __iomem *addr)
{
	/* See brcmnand_readl() comments */
	if (IS_ENABLED(CONFIG_MIPS) && IS_ENABLED(__BIG_ENDIAN))
		__raw_writel(val, addr);
	else
		writel_relaxed(val, addr);
}

static inline void usb_ctrl_unset(void __iomem *reg, uint32_t mask)
{
	brcmusb_writel(brcmusb_readl(reg) & ~(mask), reg);
};

static inline void usb_ctrl_set(void __iomem *reg, uint32_t mask)
{
	brcmusb_writel(brcmusb_readl(reg) | (mask), reg);
};

static inline void usb_ctrl_unset_family(void __iomem *reg,	uint32_t field)
{
	uint32_t mask;

	mask = usb_reg_bits_map[field];
	usb_ctrl_unset(reg, mask);
};

static inline void usb_ctrl_set_family(void __iomem *reg, uint32_t field)
{
	uint32_t mask;

	mask = usb_reg_bits_map[field];
	usb_ctrl_set(reg, mask);
};


static uint32_t usb_mdio_read(void __iomem *ctrl_base, uint32_t reg, int mode)
{
	uint32_t data;

	data = (reg << 16) | mode;
	brcmusb_writel(data, USB_CTRL_REG(ctrl_base, MDIO));
	data |= (1 << 24);
	brcmusb_writel(data, USB_CTRL_REG(ctrl_base, MDIO));
	data &= ~(1 << 24);
	udelay(10);
	brcmusb_writel(data, USB_CTRL_REG(ctrl_base, MDIO));
	udelay(10);

	return brcmusb_readl(USB_CTRL_REG(ctrl_base, MDIO2)) & 0xffff;
}

static void usb_mdio_write(void __iomem *ctrl_base, uint32_t reg,
			uint32_t val, int mode)
{
	uint32_t data;

	data = (reg << 16) | val | mode;
	brcmusb_writel(data, USB_CTRL_REG(ctrl_base, MDIO));
	data |= (1 << 25);
	brcmusb_writel(data, USB_CTRL_REG(ctrl_base, MDIO));
	data &= ~(1 << 25);
	udelay(10);
	brcmusb_writel(data, USB_CTRL_REG(ctrl_base, MDIO));
}


static void usb_phy_ldo_fix(void __iomem *ctrl_base)
{
	/* first disable FSM but also leave it that way */
	/* to allow normal suspend/resume */
	USB_CTRL_UNSET(ctrl_base, UTMI_CTL_1, POWER_UP_FSM_EN);
	USB_CTRL_UNSET(ctrl_base, UTMI_CTL_1, POWER_UP_FSM_EN_P1);

	/* reset USB 2.0 PLL */
	USB_CTRL_UNSET(ctrl_base, PLL_CTL, PLL_RESETB);
	msleep(1);
	USB_CTRL_SET(ctrl_base, PLL_CTL, PLL_RESETB);
	msleep(10);

}


static void usb2_eye_fix(void __iomem *ctrl_base)
{
	/* Increase USB 2.0 TX level to meet spec requirement */
	usb_mdio_write(ctrl_base, 0x1f, 0x80a0, MDIO_USB2);
	usb_mdio_write(ctrl_base, 0x0a, 0xc6a0, MDIO_USB2);
}


static void usb3_pll_fix(void __iomem *ctrl_base)
{
	/* Set correct window for PLL lock detect */
	usb_mdio_write(ctrl_base, 0x1f, 0x8000, MDIO_USB3);
	usb_mdio_write(ctrl_base, 0x07, 0x1503, MDIO_USB3);
}


static void usb3_enable_pipe_reset(void __iomem *ctrl_base)
{
	uint32_t val;

	/* Re-enable USB 3.0 pipe reset */
	usb_mdio_write(ctrl_base, 0x1f, 0x8000, MDIO_USB3);
	val = usb_mdio_read(ctrl_base, 0x0f, MDIO_USB3) | 0x200;
	usb_mdio_write(ctrl_base, 0x0f, val, MDIO_USB3);
}


static void usb3_enable_sigdet(void __iomem *ctrl_base)
{
	uint32_t val, ofs;
	int ii;

	ofs = 0;
	for (ii = 0; ii < 2; ++ii) {
		/* Set correct default for sigdet */
		usb_mdio_write(ctrl_base, 0x1f, (0x8080 + ofs), MDIO_USB3);
		val = usb_mdio_read(ctrl_base, 0x05, MDIO_USB3);
		val = (val & ~0x800f) | 0x800d;
		usb_mdio_write(ctrl_base, 0x05, val, MDIO_USB3);
		ofs = 0x1000;
	}
}


static void usb3_enable_skip_align(void __iomem *ctrl_base)
{
	uint32_t val, ofs;
	int ii;

	ofs = 0;
	for (ii = 0; ii < 2; ++ii) {
		/* Set correct default for SKIP align */
		usb_mdio_write(ctrl_base, 0x1f, (0x8060 + ofs), MDIO_USB3);
		val = usb_mdio_read(ctrl_base, 0x01, MDIO_USB3) | 0x200;
		usb_mdio_write(ctrl_base, 0x01, val, MDIO_USB3);
		ofs = 0x1000;
	}
}


static void usb3_pll_54Mhz(void __iomem *ctrl_base)
{
	uint32_t ofs;
	int ii;

	if (my_family != BRCM_FAMILY_7271A0)
		return;
	/*
	 * On the 7271a0 and 7268a0, the reference clock for the
	 * 3.0 PLL has been changed from 50MHz to 54MHz so the
	 * PLL needs to be reprogramed. Later chips will have
	 * the PLL programmed correctly on power-up.
	 * See SWLINUX-4006.
	 */

	/* set USB 3.0 PLL to accept 54Mhz reference clock */
	USB_CTRL_UNSET(ctrl_base, USB30_CTL1, phy3_pll_seq_start);

	usb_mdio_write(ctrl_base, 0x1f, 0x8000, MDIO_USB3);
	usb_mdio_write(ctrl_base, 0x10, 0x5784, MDIO_USB3);
	usb_mdio_write(ctrl_base, 0x11, 0x01d0, MDIO_USB3);
	usb_mdio_write(ctrl_base, 0x12, 0x1DE8, MDIO_USB3);
	usb_mdio_write(ctrl_base, 0x13, 0xAA80, MDIO_USB3);
	usb_mdio_write(ctrl_base, 0x14, 0x8826, MDIO_USB3);
	usb_mdio_write(ctrl_base, 0x15, 0x0044, MDIO_USB3);
	usb_mdio_write(ctrl_base, 0x16, 0x8000, MDIO_USB3);
	usb_mdio_write(ctrl_base, 0x17, 0x0851, MDIO_USB3);
	usb_mdio_write(ctrl_base, 0x18, 0x0000, MDIO_USB3);

	/* both ports */
	ofs = 0;
	for (ii = 0; ii < 2; ++ii) {
		usb_mdio_write(ctrl_base, 0x1f, (0x8040 + ofs), MDIO_USB3);
		usb_mdio_write(ctrl_base, 0x03, 0x0090, MDIO_USB3);
		usb_mdio_write(ctrl_base, 0x04, 0x0134, MDIO_USB3);
		usb_mdio_write(ctrl_base, 0x1f, (0x8020 + ofs), MDIO_USB3);
		usb_mdio_write(ctrl_base, 0x01, 0x00e2, MDIO_USB3);
		ofs = 0x1000;
	}

	/* restart PLL sequence */
	USB_CTRL_SET(ctrl_base, USB30_CTL1, phy3_pll_seq_start);
	msleep(1);
}


static void usb3_ssc_enable(void __iomem *ctrl_base)
{
	uint32_t val;

	/* Enable USB 3.0 TX spread spectrum */
	usb_mdio_write(ctrl_base, 0x1f, 0x8040, MDIO_USB3);
	val = usb_mdio_read(ctrl_base, 0x01, MDIO_USB3) | 0xf;
	usb_mdio_write(ctrl_base, 0x01, val, MDIO_USB3);

	/* Currently, USB 3.0 SSC is enabled via port 0 MDIO registers,
	 * which should have been adequate. However, due to a bug in the
	 * USB 3.0 PHY, it must be enabled via both ports (HWUSB3DVT-26).
	 */
	usb_mdio_write(ctrl_base, 0x1f, 0x9040, MDIO_USB3);
	val = usb_mdio_read(ctrl_base, 0x01, MDIO_USB3) | 0xf;
	usb_mdio_write(ctrl_base, 0x01, val, MDIO_USB3);
}


static void usb3_phy_workarounds(void __iomem *ctrl_base)
{
	usb3_pll_fix(ctrl_base);
	usb3_pll_54Mhz(ctrl_base);
	usb3_ssc_enable(ctrl_base);
	usb3_enable_pipe_reset(ctrl_base);
	usb3_enable_sigdet(ctrl_base);
	usb3_enable_skip_align(ctrl_base);
}


static void memc_fix(struct brcm_usb_common_init_params *params)
{
	void __iomem *ctrl_base = params->ctrl_regs;
	uint32_t prid;

	if (my_family != BRCM_FAMILY_7445D0)
		return;
	/*
	 * This is a workaround for HW7445-1869 where a DMA write ends up
	 * doing a read pre-fetch after the end of the DMA buffer. This
	 * causes a problem when the DMA buffer is at the end of physical
	 * memory, causing the pre-fetch read to access non-existent memory,
	 * and the chip bondout has MEMC2 disabled. When the pre-fetch read
	 * tries to use the disabled MEMC2, it hangs the bus. The workaround
	 * is to disable MEMC2 access in the usb controller which avoids
	 * the hang.
	 */

	prid = params->product_id & 0xfffff000;
	switch (prid) {
	case 0x72520000:
	case 0x74480000:
	case 0x74490000:
	case 0x07252000:
	case 0x07448000:
	case 0x07449000:
		USB_CTRL_UNSET_FAMILY(ctrl_base, SETUP, scb2_en);
	}
}

static void usb3_otp_fix(struct brcm_usb_common_init_params *params)
{
	void __iomem *xhci_ec_base = params->xhci_ec_regs;
	uint32_t val;

	if ((params->family_id != 0x74371000) || (xhci_ec_base == 0))
		return;
	brcmusb_writel(0xa20c, USB_XHCI_EC_REG(xhci_ec_base, IRAADR));
	val = brcmusb_readl(USB_XHCI_EC_REG(xhci_ec_base, IRADAT));

	/* set cfg_pick_ss_lock */
	val |= (1 << 27);
	brcmusb_writel(val, USB_XHCI_EC_REG(xhci_ec_base, IRADAT));

	/* Reset USB 3.0 PHY for workaround to take effect */
	USB_CTRL_UNSET(params->ctrl_regs, USB30_CTL1, phy3_resetb);
	USB_CTRL_SET(params->ctrl_regs,	USB30_CTL1, phy3_resetb);
}


static void xhci_soft_reset(void __iomem *ctrl, int on_off)
{
	/* Assert reset */
	if (on_off) {
		if (USB_CTRL_MASK_FAMILY(USB_PM, xhc_soft_resetb))
			USB_CTRL_UNSET_FAMILY(ctrl, USB_PM, xhc_soft_resetb);
		else
			USB_CTRL_UNSET_FAMILY(ctrl,
					USB30_CTL1, xhc_soft_resetb);
	}
	/* De-assert reset */
	else {
		if (USB_CTRL_MASK_FAMILY(USB_PM, xhc_soft_resetb))
			USB_CTRL_SET_FAMILY(ctrl, USB_PM, xhc_soft_resetb);
		else
			USB_CTRL_SET_FAMILY(ctrl, USB30_CTL1, xhc_soft_resetb);
	}
}

static enum brcm_family_type get_family_type(
	struct brcm_usb_common_init_params *params)
{
	unsigned int x;

	for (x = 0;
	     x < (sizeof(id_to_type_table) / sizeof(struct id_to_type));
	     x++)
		/* ignore the minor rev */
		if ((params->family_id & 0xfffffff0) ==
			id_to_type_table[x].id) {
			return id_to_type_table[x].type;
		}
	return BRCM_FAMILY_DEFAULT;
}

void brcm_usb_common_init(struct brcm_usb_common_init_params *params)
{
	uint32_t reg;
	void __iomem *ctrl = params->ctrl_regs;
	int change_ipp = 0;

	my_family = get_family_type(params);
	usb_reg_bits_map = &usb_reg_bits_map_table[my_family][0];
	xhci_soft_reset(ctrl, 1);

	if (BRCM_ID(params->family_id) == 0x7366) {
		/*
		 * The PHY3_SOFT_RESETB bits default to the wrong state.
		 */
		USB_CTRL_SET(ctrl, USB30_PCTL, PHY3_SOFT_RESETB);
		USB_CTRL_SET(ctrl, USB30_PCTL, PHY3_SOFT_RESETB_P1);
	}
	if (my_family == BRCM_FAMILY_7366C0)
		/*
		 * Don't enable this so the memory controller doesn't read
		 * into memory holes. NOTE: This bit is low true on 7366C0.
		 */
		USB_CTRL_SET_FAMILY(ctrl, EBRIDGE, ESTOP_SCB_REQ);

	/* Take USB out of power down */
	if (USB_CTRL_MASK_FAMILY(PLL_CTL, PLL_IDDQ_PWRDN)) {
		USB_CTRL_UNSET_FAMILY(ctrl, PLL_CTL, PLL_IDDQ_PWRDN);
		/* 1 millisecond - for USB clocks to settle down */
		msleep(1);
	}

	/* 3390a0 & 7439b0 so far. */
	if (USB_CTRL_MASK_FAMILY(USB_PM, USB_PWRDN)) {
		USB_CTRL_UNSET_FAMILY(ctrl, USB_PM, USB_PWRDN);
		/* 1 millisecond - for USB clocks to settle down */
		msleep(1);
	}

	/* Starting with the 7445d0, there are no longer separate 3.0
	 * versions of IOC and IPP.
	 */
	if (USB_CTRL_MASK_FAMILY(USB30_CTL1, usb3_ioc)) {
		if (params->ioc)
			USB_CTRL_SET_FAMILY(ctrl, USB30_CTL1, usb3_ioc);
		if (params->ipp == 1)
			USB_CTRL_SET_FAMILY(ctrl, USB30_CTL1, usb3_ipp);
	}

	if ((my_family != BRCM_FAMILY_74371A0) &&
		(BRCM_ID(params->family_id) != 0x7364))
		/*
		 * HW7439-637: 7439a0 and its derivatives do not have large
		 * enough descriptor storage for this.
		 */
		USB_CTRL_SET_FAMILY(ctrl, SETUP, ss_ehci64bit_en);

	/*
	 * Kick start USB3 PHY
	 * Make sure it's low to insure a rising edge.
	 */
	USB_CTRL_UNSET(ctrl, USB30_CTL1, phy3_pll_seq_start);
	USB_CTRL_SET(ctrl, USB30_CTL1, phy3_pll_seq_start);

	/* Block auto PLL suspend by USB2 PHY */
	USB_CTRL_SET(ctrl, PLL_CTL, PLL_SUSPEND_EN);

	usb_phy_ldo_fix(ctrl);
	usb2_eye_fix(ctrl);
	if (params->has_xhci)
		usb3_phy_workarounds(ctrl);

	/* Setup the endian bits */
	reg = brcmusb_readl(USB_CTRL_REG(ctrl, SETUP));
	reg &= ~USB_CTRL_SETUP_CONDITIONAL_BITS;
	reg |= ENDIAN_SETTINGS;

	if (my_family == BRCM_FAMILY_7364A0)
		/* Suppress overcurrent indication from USB30 ports for A0 */
		reg |= USB_CTRL_MASK_FAMILY(SETUP, OC3_DISABLE);

	if (USB_CTRL_MASK_FAMILY(SETUP, strap_ipp_sel))
		if (params->ipp != 2)
			/* override ipp strap pin (if it exits) */
			reg &= ~(USB_CTRL_MASK_FAMILY(SETUP, strap_ipp_sel));

	/*
	 * Make sure the the second and third memory controller
	 * interfaces are enabled.
	 */
	if (USB_CTRL_MASK_FAMILY(SETUP, scb1_en))
		reg |= USB_CTRL_MASK_FAMILY(SETUP, scb1_en);
	if (USB_CTRL_MASK_FAMILY(SETUP, scb2_en))
		reg |= USB_CTRL_MASK_FAMILY(SETUP, scb2_en);

	/* Override the default OC and PP polarity */
	if (params->ioc)
		reg |= USB_CTRL_MASK(SETUP, IOC);
	if ((params->ipp == 1) &&
		((reg & USB_CTRL_MASK(SETUP, IPP)) == 0)) {
		change_ipp = 1;
		reg |= USB_CTRL_MASK(SETUP, IPP);
	}
	brcmusb_writel(reg, USB_CTRL_REG(ctrl, SETUP));

	/*
	 * If we're changing IPP, make sure power is off long enough
	 * to turn off any connected devices.
	 */
	if (change_ipp)
		msleep(50);
	memc_fix(params);
	if (params->has_xhci) {
		xhci_soft_reset(ctrl, 0);
		usb3_otp_fix(params);
	}
	if (USB_CTRL_MASK_FAMILY(USB_DEVICE_CTL1, port_mode)) {
		reg = brcmusb_readl(USB_CTRL_REG(ctrl, USB_DEVICE_CTL1));
		reg &= ~USB_CTRL_MASK_FAMILY(USB_DEVICE_CTL1, port_mode);
		reg |= params->device_mode;
		brcmusb_writel(reg, USB_CTRL_REG(ctrl, USB_DEVICE_CTL1));
	}
	if (USB_CTRL_MASK_FAMILY(USB_PM, bdc_soft_resetb)) {
		switch (params->device_mode) {
		case USB_CTLR_DEVICE_OFF:
			USB_CTRL_UNSET_FAMILY(ctrl, USB_PM, bdc_soft_resetb);
			break;
		case USB_CTLR_DEVICE_ON:
		case USB_CTLR_DEVICE_DUAL:
			USB_CTRL_SET_FAMILY(ctrl, USB_PM, bdc_soft_resetb);
		break;
		}
	}
}
