// SPDX-License-Identifier: GPL-2.0+
/* Copyright (C) 2017-2018 Broadcom */

/**
 * DOC: Broadcom VC5 MMU
 *
 * The VC5 hardware (compared to VC4) now includes an MMU.  It has a
 * single level of page tables for the V3D's 4GB address space to map
 * to AXI bus addresses, thus it could need up to 4MB of physically
 * contiguous memory to store the PTEs.
 *
 * Because the 4MB of contiguous memory for page tables is precious,
 * and switching between them is expensive, we load all BOs into the
 * same 4GB address space, and use the GMP to shield clients from each
 * other.  The GMP is an 8KB bitmask for memory protection at 128kb
 * granularity, which can be quickly re-loaded at context switch time.
 *
 * (XXX: Still need to do the GMP bits)
 */

#include "vc5_drv.h"
#include "vc5_regs.h"

#define VC5_MMU_PAGE_SHIFT 12

/* Note: All PTEs for the 1MB superpage must be filled with the
 * superpage bit set.
 */
#define VC5_PTE_SUPERPAGE BIT(31)
#define VC5_PTE_WRITEABLE BIT(29)
#define VC5_PTE_VALID BIT(28)

static int vc5_mmu_flush_all(struct vc5_dev *vc5)
{
	int ret;

	/* Make sure that another flush isn't already running when we
	 * start this one.
	 */
	ret = wait_for(!(V3D_READ(V3D_MMU_CTL) &
			 V3D_MMU_CTL_TLB_CLEARING), 100);
	if (ret)
		dev_err(vc5->dev, "TLB clear wait idle failed\n");

	V3D_WRITE(V3D_MMU_CTL,
		  V3D_MMU_CTL_ENABLE |
		  V3D_MMU_CTL_TLB_CLEAR);

	V3D_WRITE(V3D_MMUC_CONTROL,
		  V3D_MMUC_CONTROL_FLUSH |
		  V3D_MMUC_CONTROL_ENABLE);

	ret = wait_for(!(V3D_READ(V3D_MMU_CTL) &
			 V3D_MMU_CTL_TLB_CLEARING), 100);
	if (ret)
		return ret;

	ret = wait_for(!(V3D_READ(V3D_MMUC_CONTROL) &
			 V3D_MMUC_CONTROL_FLUSHING), 100);

	return ret;
}

int vc5_mmu_set_page_table(struct vc5_dev *vc5)
{
	V3D_WRITE(V3D_MMU_PT_PA_BASE, vc5->pt_paddr >> VC5_MMU_PAGE_SHIFT);
	V3D_WRITE(V3D_MMU_CTL,
		  V3D_MMU_CTL_ENABLE |
		  V3D_MMU_CTL_PT_INVALID |
		  V3D_MMU_CTL_PT_INVALID_ABORT |
		  V3D_MMU_CTL_WRITE_VIOLATION_ABORT |
		  V3D_MMU_CTL_CAP_EXCEEDED_ABORT);
	V3D_WRITE(V3D_MMU_ILLEGAL_ADDR,
		  (vc5->mmu_scratch_paddr >> VC5_MMU_PAGE_SHIFT) |
		  V3D_MMU_ILLEGAL_ADDR_ENABLE);
	V3D_WRITE(V3D_MMUC_CONTROL, V3D_MMUC_CONTROL_ENABLE);

	return vc5_mmu_flush_all(vc5);
}

void vc5_mmu_insert_ptes(struct vc5_bo *bo)
{
	struct vc5_dev *vc5 = to_vc5_dev(bo->base.dev);
	u32 page = bo->node.start;
	u32 page_prot = VC5_PTE_WRITEABLE | VC5_PTE_VALID;
	unsigned count;
	struct scatterlist *sgl;

	for_each_sg(bo->sgt->sgl, sgl, bo->sgt->nents, count) {
		u32 page_address = sg_dma_address(sgl) >> VC5_MMU_PAGE_SHIFT;
		u32 pte = page_prot | page_address;
		u32 i;

		BUG_ON(page_address + (sg_dma_len(sgl) >> VC5_MMU_PAGE_SHIFT) >=
		       BIT(24));

		for (i = 0; i < sg_dma_len(sgl) >> VC5_MMU_PAGE_SHIFT; i++)
			vc5->pt[page++] = pte + i;
	}

	if (vc5_mmu_flush_all(vc5))
		dev_err(vc5->dev, "MMU flush timeout\n");
}

void vc5_mmu_remove_ptes(struct vc5_bo *bo)
{
	struct vc5_dev *vc5 = to_vc5_dev(bo->base.dev);
	u32 npages = bo->base.size >> VC5_MMU_PAGE_SHIFT;
	u32 page;

	for (page = bo->node.start; page < bo->node.start + npages; page++)
		vc5->pt[page] = 0;

	if (vc5_mmu_flush_all(vc5))
		dev_err(vc5->dev, "MMU flush timeout\n");
}
