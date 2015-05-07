/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2014 Jaedon Shin <jaedon.shin@gmail.com>
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <asm/bootinfo.h>
#include <asm/fw/cfe/cfe_api.h>
#include <asm/fw/cfe/cfe_error.h>

static char cfe_buff[COMMAND_LINE_SIZE] __initdata;

void __init of_cfe_early_param(void)
{
	uint64_t cfe_ept, cfe_handle;
	unsigned int cfe_eptseal;
	int argc = fw_arg0;
	char **envp = (char **)fw_arg2;
	int *prom_vec = (int *)fw_arg3;

	cfe_handle = (uint64_t)(long)argc;
	cfe_ept = (long)envp;
	cfe_eptseal = (uint32_t)(unsigned long)prom_vec;

	if (cfe_eptseal != CFE_EPTSEAL)
		return;

	cfe_init(cfe_handle, cfe_ept);

	if (cfe_getenv("BOOT_FLAGS", cfe_buff, COMMAND_LINE_SIZE) == CFE_OK) {
		strlcat(arcs_cmdline, " ", COMMAND_LINE_SIZE);
		strlcat(arcs_cmdline, cfe_buff, COMMAND_LINE_SIZE);
	}
}

static int __init of_cfe_ethernet(const char *s)
{
	struct device_node *node;
	struct property *newmac;
	const u8 *macaddr;
	int prop_len;

	node = of_find_node_by_name(NULL, "ethernet");
	if (node) {
		macaddr = of_get_property(node, "mac-address", &prop_len);
		if (macaddr == NULL || prop_len != 6)
			return -EINVAL;

		newmac = kzalloc(sizeof(*newmac) + 6, GFP_KERNEL);
		if (newmac == NULL)
			return -ENOMEM;

		newmac->value = newmac + 1;
		newmac->length = 6;
		newmac->name = kstrdup("mac-address", GFP_KERNEL);
		if (newmac->name == NULL) {
			kfree(newmac);
			return -ENOMEM;
		}

		mac_pton(s, newmac->value);
		of_update_property(node, newmac);
	}

	return 0;
}

static int __init of_cfe_mtd_partition(const char *name, u32 offset, u32 size)
{
	struct device_node *node;
	struct property *newreg;
	const __be32 *reg;
	int prop_len;

	node = of_find_node_by_name(NULL, name);
	if (node) {
		reg = of_get_property(node, "reg", &prop_len);
		if (reg == NULL || prop_len != 8)
			return -EINVAL;

		newreg = kzalloc(sizeof(*newreg) + 8, GFP_KERNEL);
		if (newreg == NULL)
			return -ENOMEM;

		newreg->value = newreg + 1;
		newreg->length = 8;
		newreg->name = kstrdup("reg", GFP_KERNEL);
		if (newreg->name == NULL) {
			kfree(newreg);
			return -ENOMEM;
		}

		*(u32 *)(newreg->value+0) = cpu_to_be32(offset);
		*(u32 *)(newreg->value+4) = cpu_to_be32(size);
		of_update_property(node, newreg);
	}

	return 0;
}

static int __init of_cfe_device_setup(void)
{
	uint64_t cfe_ept, cfe_handle;
	unsigned int cfe_eptseal;
	int argc = fw_arg0;
	char **envp = (char **)fw_arg2;
	int *prom_vec = (int *)fw_arg3;
	unsigned long offset, size;

	cfe_handle = (uint64_t)(long)argc;
	cfe_ept = (long)envp;
	cfe_eptseal = (uint32_t)(unsigned long)prom_vec;

	if (cfe_eptseal != CFE_EPTSEAL)
		return 0;

	cfe_init(cfe_handle, cfe_ept);

	if (cfe_getenv("ETH0_HWADDR", cfe_buff, 32) == CFE_OK)
		of_cfe_ethernet(cfe_buff);

	if (cfe_getenv("LINUX_PART_STARTAD", cfe_buff, 16) == CFE_OK &&
	    cfe_getenv("LINUX_PART_SIZE", cfe_buff+16, 16) == CFE_OK) {
		if (!kstrtoul(cfe_buff, 16, &offset) &&
		    !kstrtoul(cfe_buff+16, 16, &size))
			of_cfe_mtd_partition("kernel", offset, size);
	}

	if (cfe_getenv("SPLASH_PART_STARTAD", cfe_buff, 16) == CFE_OK &&
	    cfe_getenv("SPLASH_PART_SIZE", cfe_buff+16, 16) == CFE_OK) {
		if (!kstrtoul(cfe_buff, 16, &offset) &&
		    !kstrtoul(cfe_buff+16, 16, &size))
			of_cfe_mtd_partition("splash", offset, size);
	}

	if (cfe_getenv("LINUX_FFS_STARTAD", cfe_buff, 16) == CFE_OK &&
	    cfe_getenv("LINUX_FFS_SIZE", cfe_buff+16, 16) == CFE_OK) {
		if (!kstrtoul(cfe_buff, 16, &offset) &&
		    !kstrtoul(cfe_buff+16, 16, &size))
			of_cfe_mtd_partition("rootfs", offset, size);
	}

	return 0;
}

subsys_initcall(of_cfe_device_setup);
