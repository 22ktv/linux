/*
 * Copyright (C) 2015-2016, Broadcom
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
 * A copy of the GPL is available at
 * http://www.broadcom.com/licenses/GPLv2.php or from the Free Software
 * Foundation at https://www.gnu.org/licenses/ .
 */

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_irq.h>

int brcmstb_irqs_get_virq(const char *node_name, const char *irq_name)
{
	struct device_node *np;
	int i, num_elems, ret;
	const char *int_name;
	u32 hwirq;

	np = of_find_node_by_name(NULL, node_name);
	if (!np)
		return -ENOENT;

	num_elems = of_property_count_strings(np, "interrupt-names");
	if (num_elems <= 0) {
		pr_err(
		    "Unable to find an interrupt-names property, check DT\n");
		return -EINVAL;
	}

	for (i = 0; i < num_elems; i++) {
		ret = of_property_read_u32_index(np, "interrupts", i, &hwirq);
		if (ret < 0)
			return ret;

		ret = of_property_read_string_index(np, "interrupt-names", i,
						    &int_name);
		if (ret < 0)
			return ret;

		/* We may be requesting to match, eg: "gio" with "gio_aon" */
		if (!strncasecmp(int_name, irq_name, strlen(int_name)))
			break;
	}

	if (i == num_elems) {
		pr_debug("%s: exceeded search for %s\n", __func__, irq_name);
		return -ENOENT;
	}

	pr_debug("%s IRQ name: %s Node: %s @%d mapped to: %d\n", __func__,
		 irq_name, np->full_name, i, hwirq);

	return of_irq_get(np, i);
}
EXPORT_SYMBOL(brcmstb_irqs_get_virq);
