/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __ASM_ARCH_MEMORY_H__
#define __ASM_ARCH_MEMORY_H__
#ifndef __ASSEMBLY__

struct device;

#include <soc/brcmstb/common.h>

#ifdef CONFIG_PCIE_BRCMSTB
#define __arch_pfn_to_dma(dev, pfn)					\
	({								\
		if (dev)						\
			pfn -= dev->dma_pfn_offset;			\
		(dma_addr_t)brcm_phys_to_dma(dev, __pfn_to_phys(pfn));	\
	})

#define  __arch_dma_to_pfn(dev, addr)					\
	({								\
		unsigned long pfn = __phys_to_pfn(brcm_dma_to_phys(dev, addr));\
		if (dev)						\
			pfn += dev->dma_pfn_offset;			\
		pfn;							\
	})

#define __arch_dma_to_virt(dev, addr)					\
	({								\
		void *v;						\
		if (dev) {						\
			unsigned long pfn = dma_to_pfn(dev, addr);	\
			v = phys_to_virt(__pfn_to_phys(pfn));		\
		} else {						\
			v = (void *)__bus_to_virt((unsigned long)addr);	\
		}							\
		v;							\
	})

#define __arch_virt_to_dma(dev, addr)					\
	({								\
		(dev) ? pfn_to_dma(dev, virt_to_pfn(addr))		\
		      : (dma_addr_t)__virt_to_bus((unsigned long)(addr));\
	})

#endif /* CONFIG_PCIE_BRCMSTB */
#endif /* __ASSEMBLY__ */
#endif /* __ASM_ARCH_MEMORY_H__ */
