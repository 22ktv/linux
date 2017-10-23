/*
 * Copyright (C) 2015-2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <linux/bitops.h>
#include <linux/compiler.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/types.h>

#include "pci-brcmstb.h"

#define PCIE_MISC_MSI_DATA_CONFIG			0x404c
#define PCIE_MSI_INTR2_BASE				0x4500
#define PCIE_MISC_MSI_BAR_CONFIG_LO			0x4044
#define PCIE_MISC_MSI_BAR_CONFIG_HI			0x4048

/* Offsets from PCIE_INTR2_CPU_BASE and PCIE_MSI_INTR2_BASE */
#define STATUS				0x0
#define SET				0x4
#define CLR				0x8
#define MASK_STATUS			0xc
#define MASK_SET			0x10
#define MASK_CLR			0x14

struct brcm_msi {
	struct irq_domain *msi_domain;
	struct irq_domain *inner_domain;
	struct mutex lock; /* guards the alloc/free operations */
	u64 target_addr;
	int irq;
	/* intr_base is the base pointer for interrupt status/set/clr regs */
	void __iomem *intr_base;
	/* intr_legacy_mask indicates how many bits are MSI interrupts */
	u32 intr_legacy_mask;
	/* intr_legacy_offset indicates bit position of MSI_01. It is
	 * to map the register bit position to a hwirq that starts at 0.
	 */
	u32 intr_legacy_offset;
	/* used indicates which MSI interrupts have been alloc'd */
	unsigned long used;

	void __iomem *base;
	struct device *dev;
	struct device_node *dn;
	unsigned int rev;
};

static struct irq_chip brcm_msi_irq_chip = {
	.name = "Brcm_MSI",
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

static struct msi_domain_info brcm_msi_domain_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		   MSI_FLAG_PCI_MSIX),
	.chip	= &brcm_msi_irq_chip,
};

static void brcm_pcie_msi_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct brcm_msi *msi;
	unsigned long status, virq;
	u32 mask, bit, hwirq;

	chained_irq_enter(chip, desc);
	msi = irq_desc_get_handler_data(desc);
	mask = msi->intr_legacy_mask;

	while ((status = bcm_readl(msi->intr_base + STATUS) & mask)) {
		for_each_set_bit(bit, &status, BRCM_INT_PCI_MSI_NR) {
			/* clear the interrupt */
			bcm_writel(1 << bit, msi->intr_base + CLR);

			/* Account for legacy interrupt offset */
			hwirq = bit - msi->intr_legacy_offset;

			virq = irq_find_mapping(msi->inner_domain, hwirq);
			if (virq) {
				if (msi->used & (1 << hwirq))
					generic_handle_irq(virq);
				else
					dev_info(msi->dev, "unhandled MSI %d\n",
						 hwirq);
			} else {
				/* Unknown MSI, just clear it */
				dev_dbg(msi->dev, "unexpected MSI\n");
			}
		}
	}
	chained_irq_exit(chip, desc);
}

static void brcm_compose_msi_msg(struct irq_data *data, struct msi_msg *msg)
{
	struct brcm_msi *msi = irq_data_get_irq_chip_data(data);
	u32 temp;

	msg->address_lo = lower_32_bits(msi->target_addr);
	msg->address_hi = upper_32_bits(msi->target_addr);
	temp = bcm_readl(msi->base + PCIE_MISC_MSI_DATA_CONFIG);
	msg->data = ((temp >> 16) & (temp & 0xffff)) | data->hwirq;
}

static int brcm_msi_set_affinity(struct irq_data *irq_data,
				 const struct cpumask *mask, bool force)
{
	return -EINVAL;
}

static struct irq_chip brcm_msi_bottom_irq_chip = {
	.name			= "Brcm MSI",
	.irq_compose_msi_msg	= brcm_compose_msi_msg,
	.irq_set_affinity	= brcm_msi_set_affinity,
};

static int brcm_msi_alloc(struct brcm_msi *msi)
{
	int bit, hwirq;

	mutex_lock(&msi->lock);
	bit = ~msi->used ? ffz(msi->used) : -1;

	if (bit >= 0 && bit < BRCM_INT_PCI_MSI_NR) {
		msi->used |= (1 << bit);
		hwirq = bit - msi->intr_legacy_offset;
	} else {
		hwirq = -ENOSPC;
	}

	mutex_unlock(&msi->lock);
	return hwirq;
}

static void brcm_msi_free(struct brcm_msi *msi, unsigned long hwirq)
{
	mutex_lock(&msi->lock);
	msi->used &= ~(1 << (hwirq + msi->intr_legacy_offset));
	mutex_unlock(&msi->lock);
}

static int brcm_irq_domain_alloc(struct irq_domain *domain, unsigned int virq,
				 unsigned int nr_irqs, void *args)
{
	struct brcm_msi *msi = domain->host_data;
	int hwirq;

	hwirq = brcm_msi_alloc(msi);

	if (hwirq < 0)
		return hwirq;

	irq_domain_set_info(domain, virq, (irq_hw_number_t)hwirq,
			    &brcm_msi_bottom_irq_chip, domain->host_data,
			    handle_simple_irq, NULL, NULL);
	return 0;
}

static void brcm_irq_domain_free(struct irq_domain *domain,
				 unsigned int virq, unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct brcm_msi *msi = irq_data_get_irq_chip_data(d);

	brcm_msi_free(msi, d->hwirq);
}

void brcm_msi_set_regs(struct brcm_msi *msi)
{
	u32 data_val, msi_lo, msi_hi;

	if (msi->rev >= BRCM_PCIE_HW_REV_33) {
		/* ffe0 -- least sig 5 bits are 0 indicating 32 msgs
		 * 6540 -- this is our arbitrary unique data value
		 */
		data_val = 0xffe06540;
	} else {
		/* fff8 -- least sig 3 bits are 0 indicating 8 msgs
		 * 6540 -- this is our arbitrary unique data value
		 */
		data_val = 0xfff86540;
	}

	/* Make sure we are not masking MSIs.  Note that MSIs can be masked,
	 * but that occurs on the PCIe EP device
	 */
	bcm_writel(0xffffffff & msi->intr_legacy_mask,
		   msi->intr_base + MASK_CLR);

	msi_lo = lower_32_bits(msi->target_addr);
	msi_hi = upper_32_bits(msi->target_addr);
	/* The 0 bit of PCIE_MISC_MSI_BAR_CONFIG_LO is repurposed to MSI
	 * enable, which we set to 1.
	 */
	bcm_writel(msi_lo | 1, msi->base + PCIE_MISC_MSI_BAR_CONFIG_LO);
	bcm_writel(msi_hi, msi->base + PCIE_MISC_MSI_BAR_CONFIG_HI);
	bcm_writel(data_val, msi->base + PCIE_MISC_MSI_DATA_CONFIG);
}
EXPORT_SYMBOL(brcm_msi_set_regs);

static const struct irq_domain_ops msi_domain_ops = {
	.alloc	= brcm_irq_domain_alloc,
	.free	= brcm_irq_domain_free,
};

static int brcm_allocate_domains(struct brcm_msi *msi)
{
	struct fwnode_handle *fwnode = of_node_to_fwnode(msi->dn);

	msi->inner_domain = irq_domain_add_linear(NULL, BRCM_INT_PCI_MSI_NR,
						  &msi_domain_ops, msi);
	if (!msi->inner_domain) {
		dev_err(msi->dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}

	msi->msi_domain = pci_msi_create_irq_domain(fwnode,
						    &brcm_msi_domain_info,
						    msi->inner_domain);
	if (!msi->msi_domain) {
		dev_err(msi->dev, "failed to create MSI domain\n");
		irq_domain_remove(msi->inner_domain);
		return -ENOMEM;
	}

	return 0;
}

static void brcm_free_domains(struct brcm_msi *msi)
{
	irq_domain_remove(msi->msi_domain);
	irq_domain_remove(msi->inner_domain);
}

void brcm_msi_remove(struct brcm_msi *msi)
{
	if (!msi)
		return;
	irq_set_chained_handler(msi->irq, NULL);
	irq_set_handler_data(msi->irq, NULL);
	brcm_free_domains(msi);
}
EXPORT_SYMBOL(brcm_msi_remove);

int brcm_msi_probe(struct platform_device *pdev, struct brcm_info *info)
{
	struct brcm_msi *msi;
	int irq, ret;

	irq = irq_of_parse_and_map(pdev->dev.of_node, 1);
	if (irq <= 0) {
		dev_err(&pdev->dev, "cannot map msi intr\n");
		return -ENODEV;
	}

	msi = devm_kzalloc(&pdev->dev, sizeof(struct brcm_msi), GFP_KERNEL);
	if (!msi)
		return -ENOMEM;

	msi->dev = &pdev->dev;
	msi->base = info->base;
	msi->rev =  info->rev;
	msi->dn = pdev->dev.of_node;
	msi->target_addr = info->msi_target_addr;
	msi->irq = irq;

	ret = brcm_allocate_domains(msi);
	if (ret)
		return ret;

	irq_set_chained_handler_and_data(msi->irq, brcm_pcie_msi_isr, msi);

	if (msi->rev >= BRCM_PCIE_HW_REV_33) {
		msi->intr_base = msi->base + PCIE_MSI_INTR2_BASE;
		/* This version of PCIe hw has only 32 intr bits
		 * starting at bit position 0.
		 */
		msi->intr_legacy_mask = 0xffffffff;
		msi->intr_legacy_offset = 0x0;
		msi->used = 0x0;

	} else {
		msi->intr_base = msi->base + PCIE_INTR2_CPU_BASE;
		/* This version of PCIe hw has only 8 intr bits starting
		 * at bit position 24.
		 */
		msi->intr_legacy_mask = 0xff000000;
		msi->intr_legacy_offset = 24;
		msi->used = 0x00ffffff;
	}

	brcm_msi_set_regs(msi);
	info->msi = msi;

	return 0;
}
EXPORT_SYMBOL(brcm_msi_probe);
