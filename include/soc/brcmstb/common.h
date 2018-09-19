/*
 * Copyright © 2014 NVIDIA Corporation
 * Copyright © 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SOC_BRCMSTB_COMMON_H__
#define __SOC_BRCMSTB_COMMON_H__

bool soc_is_brcmstb(void);

#if defined(CONFIG_PCIE_BRCMSTB)
dma_addr_t brcm_phys_to_dma(struct device *dev, phys_addr_t paddr);
phys_addr_t brcm_dma_to_phys(struct device *dev, dma_addr_t dev_addr);
#else
static inline dma_addr_t brcm_phys_to_dma(struct device *dev, phys_addr_t paddr)
{
	return (dma_addr_t)paddr;
}

static inline phys_addr_t brcm_dma_to_phys(struct device *dev,
					   dma_addr_t dev_addr)
{
	return (phys_addr_t)dev_addr;
}
#endif

#endif /* __SOC_BRCMSTB_COMMON_H__ */
