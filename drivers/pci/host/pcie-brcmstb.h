#ifndef __BRCMSTB_PCI_H
#define __BRCMSTB_PCI_H
/*
 * Copyright (C) 2015 - 2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

int brcm_register_notifier(void);
int brcm_unregister_notifier(void);

extern struct of_pci_range *dma_ranges;
extern int num_dma_ranges;

#endif /* __BRCMSTB_PCI_H */
