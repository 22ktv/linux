/*
 * Broadcom STB PCIe root complex MSI driver
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
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/msi.h>
#include <linux/printk.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/module.h>
#include <linux/irqdomain.h>
#include <linux/string.h>
#include <linux/sizes.h>

#include "pcie-brcmstb.h"

#define BRCM_INT_PCI_MSI_NR		32
#define BRCM_PCIE_HW_REV_33		0x0303
#define BRCM_MSI_TARGET_ADDR_LO		0x0
#define BRCM_MSI_TARGET_ADDR_HI		0xffffffff

struct brcm_msi {
	struct irq_domain *domain;
	struct irq_chip irq_chip;
	struct msi_controller chip;
	struct brcm_pcie *pcie;
	struct mutex lock;
	int irq;
	/* intr_base is the base pointer for interrupt status/set/clr regs */
	void __iomem *intr_base;
	/* intr_legacy_mask indicates how many bits are MSI interrupts */
	u32 intr_legacy_mask;
	/* intr_legacy_offset indicates bit position of MSI_01 */
	u32 intr_legacy_offset;
	/* used indicates which MSI interrupts have been alloc'd */
	unsigned long used;
	/* working indicates that on boot we have brought up MSI */
	bool working;
};

static inline struct brcm_msi *to_brcm_msi(struct msi_controller *chip)
{
	return container_of(chip, struct brcm_msi, chip);
}

static int brcm_msi_alloc(struct brcm_msi *chip)
{
	int msi;

	mutex_lock(&chip->lock);
	msi = ~chip->used ? ffz(chip->used) : -1;

	if (msi >= 0 && msi < BRCM_INT_PCI_MSI_NR)
		chip->used |= (1 << msi);
	else
		msi = -ENOSPC;

	mutex_unlock(&chip->lock);
	return msi;
}

static void brcm_msi_free(struct brcm_msi *chip, unsigned long irq)
{
	mutex_lock(&chip->lock);
	chip->used &= ~(1 << irq);
	mutex_unlock(&chip->lock);
}

static irqreturn_t brcm_pcie_msi_irq(int irq, void *data)
{
	struct brcm_pcie *pcie = data;
	struct brcm_msi *msi = pcie->msi;
	unsigned long status;

	status = bpcie_readl(msi->intr_base + STATUS) & msi->intr_legacy_mask;

	if (!status)
		return IRQ_NONE;

	while (status) {
		unsigned int index = ffs(status) - 1;
		unsigned int irq;

		/* clear the interrupt */
		bpcie_writel(1 << index, msi->intr_base + CLR);
		status &= ~(1 << index);

		/* Account for legacy interrupt offset */
		index -= msi->intr_legacy_offset;

		irq = irq_find_mapping(msi->domain, index);
		if (irq) {
			if (msi->used & (1 << index))
				generic_handle_irq(irq);
			else
				dev_info(pcie->dev, "unhandled MSI %d\n",
					 index);
		} else {
			/* Unknown MSI, just clear it */
			dev_dbg(pcie->dev, "unexpected MSI\n");
		}
	}
	return IRQ_HANDLED;
}

static int brcm_msi_setup_irq(struct msi_controller *chip, struct pci_dev *pdev,
			      struct msi_desc *desc)
{
	struct brcm_msi *msi = to_brcm_msi(chip);
	struct brcm_pcie *pcie = msi->pcie;
	struct msi_msg msg;
	unsigned int irq;
	int hwirq;
	u32 data;

	hwirq = brcm_msi_alloc(msi);
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(msi->domain, hwirq);
	if (!irq) {
		brcm_msi_free(msi, hwirq);
		return -EINVAL;
	}

	irq_set_msi_desc(irq, desc);

	msg.address_lo = BRCM_MSI_TARGET_ADDR_LO;
	msg.address_hi = BRCM_MSI_TARGET_ADDR_HI;
	data = bpcie_readl(pcie->base + PCIE_MISC_MSI_DATA_CONFIG);
	msg.data = ((data >> 16) & (data & 0xffff)) | hwirq;
	wmb(); /* just being cautious */
	write_msi_msg(irq, &msg);

	return 0;
}

static void brcm_msi_teardown_irq(struct msi_controller *chip, unsigned int irq)
{
	struct brcm_msi *msi = to_brcm_msi(chip);
	struct irq_data *d = irq_get_irq_data(irq);

	brcm_msi_free(msi, d->hwirq);
}

static int brcm_msi_map(struct irq_domain *domain, unsigned int irq,
			irq_hw_number_t hwirq)
{
	struct brcm_pcie *pcie = domain->host_data;

	irq_set_chip_and_handler(irq, &pcie->msi->irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = brcm_msi_map,
};

int brcm_pcie_enable_msi(struct brcm_pcie *pcie, int nr)
{
	static const char brcm_msi_name[] = "brcmstb_pcieX_msi";
	struct brcm_msi *msi;
	u32 data_val;
	char *name;
	int err;

	pcie->msi = devm_kzalloc(pcie->dev, sizeof(*msi), GFP_KERNEL);
	if (!pcie->msi)
		return -ENODEV;

	msi = pcie->msi;
	msi->pcie = pcie;

	if (!pcie->suspended) {
		/* We are only here on cold boot */
		mutex_init(&msi->lock);

		msi->chip.dev = pcie->dev;
		msi->chip.setup_irq = brcm_msi_setup_irq;
		msi->chip.teardown_irq = brcm_msi_teardown_irq;

		/* We have multiple RC controllers.  We may have as many
		 * MSI controllers for them.  We want each to have a
		 * unique name, so we go to the trouble of having an
		 * irq_chip per RC (instead of one for all of them).
		 */
		name = devm_kzalloc(pcie->dev, sizeof(brcm_msi_name),
				    GFP_KERNEL);
		if (name) {
			char *p;

			strcpy(name, brcm_msi_name);
			p = strchr(name, 'X');
			if (p)
				*p = '0' + nr;
			msi->irq_chip.name = name;
		} else {
			msi->irq_chip.name = brcm_msi_name;
		}

		msi->irq_chip.irq_enable = unmask_msi_irq;
		msi->irq_chip.irq_disable = mask_msi_irq;
		msi->irq_chip.irq_mask = mask_msi_irq;
		msi->irq_chip.irq_unmask = unmask_msi_irq;

		msi->domain =
			irq_domain_add_linear(pcie->dn, BRCM_INT_PCI_MSI_NR,
					      &msi_domain_ops, pcie);
		if (!msi->domain) {
			dev_err(pcie->dev,
				"failed to create IRQ domain for MSI\n");
			return -ENOMEM;
		}

		err = devm_request_irq(pcie->dev, msi->irq, brcm_pcie_msi_irq,
				       IRQF_SHARED, msi->irq_chip.name,
				       pcie);
		if (err < 0) {
			dev_err(pcie->dev,
				"failed to request IRQ (%d) for MSI\n",	err);
			goto msi_en_err;
		}

		if (pcie->rev >= BRCM_PCIE_HW_REV_33) {
			msi->intr_base = pcie->base + PCIE_MSI_INTR2_BASE;
			/* This version of PCIe hw has only 32 intr bits
			 * starting at bit position 0.
			 */
			msi->intr_legacy_mask = 0xffffffff;
			msi->intr_legacy_offset = 0x0;
			msi->used = 0x0;

		} else {
			msi->intr_base = pcie->base + PCIE_INTR2_CPU_BASE;
			/* This version of PCIe hw has only 8 intr bits starting
			 * at bit position 24.
			 */
			msi->intr_legacy_mask = 0xff000000;
			msi->intr_legacy_offset = 24;
			msi->used = 0xffffff00;
		}
		msi->working = true;
	}

	/* If we are here, and msi->working is false, it means that we've
	 * already tried and failed to bring up MSI.  Just return 0
	 * since there is nothing to be done.
	 */
	if (!msi->working)
		return 0;

	if (pcie->rev >= BRCM_PCIE_HW_REV_33) {
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
	bpcie_writel(0xffffffff & msi->intr_legacy_mask,
		     msi->intr_base + MASK_CLR);

	/* The 0 bit of BRCM_MSI_TARGET_ADDR_LO is repurposed to MSI enable,
	 * which we set to 1.
	 */
	bpcie_writel(BRCM_MSI_TARGET_ADDR_LO | 1, pcie->base
		     + PCIE_MISC_MSI_BAR_CONFIG_LO);
	bpcie_writel(BRCM_MSI_TARGET_ADDR_HI, pcie->base
		     + PCIE_MISC_MSI_BAR_CONFIG_HI);
	bpcie_writel(data_val, pcie->base + PCIE_MISC_MSI_DATA_CONFIG);

	return 0;

msi_en_err:
	irq_domain_remove(msi->domain);
	return err;
}

void brcm_pcie_msi_chip_set(struct brcm_pcie *pcie)
{
	pcie->bus->msi = &pcie->msi->chip;
}
