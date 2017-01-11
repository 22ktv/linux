/*
 * Copyright (C) 2013-2014 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/irqchip.h>
#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <linux/libfdt.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

/*
 * Storage for debug-macro.S's state.
 *
 * This must be in .data not .bss so that it gets initialized each time the
 * kernel is loaded. The data is declared here rather than debug-macro.S so
 * that multiple inclusions of debug-macro.S point at the same data.
 */
u32 brcmstb_uart_config[3] = {
	/* Debug UART initialization required */
	1,
	/* Debug UART physical address */
	0,
	/* Debug UART virtual address */
	0,
};

static void __init brcmstb_init_irq(void)
{
	irqchip_init();
}

static const char *const brcmstb_match[] __initconst = {
	"brcm,bcm7445",
	"brcm,brcmstb",
	NULL
};

static void __init brcmstb_dt_fixup(void)
{
	const char *name;
	u64 rmem_base = 0, rmem_size = 0, bmem_base = 0, bmem_size = 0;

	name = of_flat_dt_get_machine_name();

	if (!strcmp(name, "OSMINI4KTYP2") || !strcmp(name, "OSMINI4K")) {
		rmem_size = 0x1c000000;
		rmem_base = 0x40000000 - rmem_size;
		bmem_size = 0x0f000000;
		bmem_base = rmem_base - bmem_size;
	} else if (!strcmp(name, "OSMIO4K") || !strcmp(name, "OSMIO4KPLUS")) {
		rmem_size = 0x1c000000;
		rmem_base = 0x80000000 - rmem_size;
		bmem_size = 0x18000000;
		bmem_base = rmem_base - bmem_size;
	}

	if (rmem_size && rmem_base) {
		u64 base, size;
		int i;

		for (i = 0; i < fdt_num_mem_rsv(initial_boot_params); i++) {
			if (fdt_get_mem_rsv(initial_boot_params, i, &base, &size) < 0)
				continue;
			if (((rmem_base < (base + size)) && (base < (rmem_base + rmem_size))) ||
			    (base > (rmem_base + rmem_size)))
				fdt_del_mem_rsv(initial_boot_params, i);
		}

		fdt_add_mem_rsv(initial_boot_params, rmem_base, rmem_size);
	}

	if (bmem_size && bmem_base) {
		int offset;
		const void *prop;
		u64 regs[2];
		char new_name[20];
		const char *p;
		int current_len;

		offset = fdt_path_offset(initial_boot_params, "/reserved-memory/bmem");
		if (offset < 0)
			return;

		prop = fdt_getprop(initial_boot_params, offset, "reg", NULL);
		if (prop) {
			regs[0] = cpu_to_fdt64(bmem_base);
			regs[1] = cpu_to_fdt64(bmem_size);

			fdt_setprop(initial_boot_params, offset, "reg", regs, sizeof(regs));
		}

		snprintf(new_name, sizeof(new_name), "bmem@%llx", bmem_base);
		p = fdt_get_name(initial_boot_params, offset, &current_len);
		if (p && current_len == strlen(new_name))
			fdt_set_name(initial_boot_params, offset, new_name);
	}
}

DT_MACHINE_START(BRCMSTB, "Broadcom STB (Flattened Device Tree)")
	.dt_compat	= brcmstb_match,
	.init_irq	= brcmstb_init_irq,
	.dt_fixup	= brcmstb_dt_fixup,
MACHINE_END
