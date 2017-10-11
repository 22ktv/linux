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
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/smp.h>

#include "pci-brcmstb.h"

static const struct dma_map_ops *arch_dma_ops;
static struct dma_map_ops brcm_dma_ops;

static void *brcm_dma_alloc(struct device *dev, size_t size, dma_addr_t *handle,
			    gfp_t gfp, unsigned long attrs)
{
	void *ret;

	ret = arch_dma_ops->alloc(dev, size, handle, gfp, attrs);
	if (ret)
		*handle = brcm_to_pci(*handle);
	return ret;
}

static void brcm_dma_free(struct device *dev, size_t size, void *cpu_addr,
			  dma_addr_t handle, unsigned long attrs)
{
	handle = brcm_to_cpu(handle);
	arch_dma_ops->free(dev, size, cpu_addr, handle, attrs);
}

static int brcm_dma_mmap(struct device *dev, struct vm_area_struct *vma,
			 void *cpu_addr, dma_addr_t dma_addr, size_t size,
			 unsigned long attrs)
{
	dma_addr = brcm_to_cpu(dma_addr);
	return arch_dma_ops->mmap(dev, vma, cpu_addr, dma_addr, size, attrs);
}

static int brcm_dma_get_sgtable(struct device *dev, struct sg_table *sgt,
				void *cpu_addr, dma_addr_t handle, size_t size,
				unsigned long attrs)
{
	handle = brcm_to_cpu(handle);
	return arch_dma_ops->get_sgtable(dev, sgt, cpu_addr, handle, size,
				       attrs);
}

static dma_addr_t brcm_dma_map_page(struct device *dev, struct page *page,
				    unsigned long offset, size_t size,
				    enum dma_data_direction dir,
				    unsigned long attrs)
{
	return brcm_to_pci(arch_dma_ops->map_page(dev, page, offset, size,
						  dir, attrs));
}

static void brcm_dma_unmap_page(struct device *dev, dma_addr_t handle,
				size_t size, enum dma_data_direction dir,
				unsigned long attrs)
{
	handle = brcm_to_cpu(handle);
	arch_dma_ops->unmap_page(dev, handle, size, dir, attrs);
}

static int brcm_dma_map_sg(struct device *dev, struct scatterlist *sgl,
			   int nents, enum dma_data_direction dir,
			   unsigned long attrs)
{
	int ret, i;
	struct scatterlist *sg;

	ret = arch_dma_ops->map_sg(dev, sgl, nents, dir, attrs);
	/* The ARM and MIPS implementations of map_sg and unmap_sg
	 * make calls to ops->map_page(), which we already intercept.
	 * The ARM64 does not, so we must iterate through the SG list
	 * and  convert each dma_address to one that is compatible
	 * with our PCI RC implementation.
	 */
	if (IS_ENABLED(CONFIG_ARM64))
		for_each_sg(sgl, sg, ret, i)
			sg->dma_address = brcm_to_pci(sg->dma_address);
	return ret;
}

static void brcm_dma_unmap_sg(struct device *dev,
			      struct scatterlist *sgl, int nents,
			      enum dma_data_direction dir,
			      unsigned long attrs)
{
	int i;
	struct scatterlist *sg;

	/* The ARM and MIPS implementations of map_sg and unmap_sg
	 * make calls to ops->map_page(), which we already intercept.
	 * The ARM64 does not, so we must iterate through the SG list
	 * and  convert each dma_address to one that is compatible
	 * with our PCI RC implementation.
	 */
	if (IS_ENABLED(CONFIG_ARM64))
		for_each_sg(sgl, sg, nents, i)
			sg->dma_address = brcm_to_cpu(sg->dma_address);
	arch_dma_ops->map_sg(dev, sgl, nents, dir, attrs);
}

static void brcm_dma_sync_single_for_cpu(struct device *dev,
					 dma_addr_t handle, size_t size,
					 enum dma_data_direction dir)
{
	handle = brcm_to_cpu(handle);
	arch_dma_ops->sync_single_for_cpu(dev, handle, size, dir);
}

static void brcm_dma_sync_single_for_device(struct device *dev,
					    dma_addr_t handle, size_t size,
					    enum dma_data_direction dir)
{
	handle = brcm_to_cpu(handle);
	arch_dma_ops->sync_single_for_device(dev, handle, size, dir);
}

static dma_addr_t brcm_dma_map_resource(struct device *dev, phys_addr_t phys,
					size_t size,
					enum dma_data_direction dir,
					unsigned long attrs)
{
	return brcm_to_pci(arch_dma_ops->map_resource
			   (dev, phys, size, dir, attrs));
}

static void brcm_dma_unmap_resource(struct device *dev, dma_addr_t handle,
				    size_t size, enum dma_data_direction dir,
				    unsigned long attrs)
{
	handle = brcm_to_cpu(handle);
	arch_dma_ops->unmap_resource(dev, handle, size, dir, attrs);
}

static int brcm_mapping_error(struct device *dev, dma_addr_t dma_addr)
{
	return dma_addr == BRCMSTB_ERROR_CODE;
}

static const struct dma_map_ops *brcm_get_arch_dma_ops(struct device *dev)
{
#if defined(CONFIG_MIPS)
	return mips_dma_map_ops;
#elif defined(CONFIG_ARM)
	return &arm_dma_ops;
#elif defined(CONFIG_ARM64)
	/* swiotlb_dma_ops is a static var, so we get ahold
	 * of it by calling arch_setup_dma_ops(...).
	 */
	arch_setup_dma_ops(dev, 0, 0, NULL, false);
	return dev->dma_ops;
#endif
	return 0;
}

static void brcm_set_dma_ops(struct device *dev)
{
	arch_dma_ops = brcm_get_arch_dma_ops(dev);
	if (!arch_dma_ops)
		return;

	/* Set all of the base operations; some will be overridden */
	brcm_dma_ops = *arch_dma_ops;

	/* Insert the Brcm-specific override operations */
	brcm_dma_ops.alloc = brcm_dma_alloc;
	brcm_dma_ops.free = brcm_dma_free;
	brcm_dma_ops.mmap = brcm_dma_mmap;
	brcm_dma_ops.get_sgtable = brcm_dma_get_sgtable;
	brcm_dma_ops.map_page = brcm_dma_map_page;
	brcm_dma_ops.unmap_page = brcm_dma_unmap_page;
	brcm_dma_ops.sync_single_for_cpu = brcm_dma_sync_single_for_cpu;
	brcm_dma_ops.sync_single_for_device = brcm_dma_sync_single_for_device;
	brcm_dma_ops.map_sg = brcm_dma_map_sg;
	brcm_dma_ops.unmap_sg = brcm_dma_unmap_sg;
	if (arch_dma_ops->map_resource)
		brcm_dma_ops.map_resource = brcm_dma_map_resource;
	if (arch_dma_ops->unmap_resource)
		brcm_dma_ops.unmap_resource = brcm_dma_unmap_resource;
	brcm_dma_ops.mapping_error = brcm_mapping_error;

	/* Use our brcm_dma_ops for this driver */
	set_dma_ops(dev, &brcm_dma_ops);
}

static int brcmstb_platform_notifier(struct notifier_block *nb,
				     unsigned long event, void *__dev)
{
	struct device *dev = __dev;

	if (event != BUS_NOTIFY_ADD_DEVICE)
		return NOTIFY_DONE;

	brcm_set_dma_ops(dev);
	return NOTIFY_OK;
}

struct notifier_block brcmstb_platform_nb = {
	.notifier_call = brcmstb_platform_notifier,
};
EXPORT_SYMBOL(brcmstb_platform_nb);
