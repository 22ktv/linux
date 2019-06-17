/*
 * Copyright © 2015-2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * A copy of the GPL is available at
 * http://www.broadcom.com/licenses/GPLv2.php or from the Free Software
 * Foundation at https://www.gnu.org/licenses/ .
 */

#include <linux/module.h>
#include <linux/cma.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>

#define MAX_BMEM_REGIONS	8

struct bmem_region {
	struct device *dev;
	phys_addr_t addr;
	phys_addr_t size;
	bool valid;
};

static struct bmem_region bmem_regions[MAX_BMEM_REGIONS];
static unsigned int n_bmem_regions;

struct device *brcmstb_bmem_get_device(int region_num)
{
	if (region_num >= n_bmem_regions)
		return NULL;

	return bmem_regions[region_num].dev;
}
EXPORT_SYMBOL(brcmstb_bmem_get_device);

static int brcmstb_bmem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	ret = of_reserved_mem_device_init(dev);
	if (ret)
		return ret;

	bmem_regions[n_bmem_regions].dev = dev;
	bmem_regions[n_bmem_regions].valid = true;
	n_bmem_regions++;

	return 0;
}

static const struct of_device_id brcmstb_bmem_of_match[] = {
	{ .compatible = "brcm,bmem" },
	{},
};
MODULE_DEVICE_TABLE(of, brcmstb_bmem_of_match);

static struct platform_driver brcmstb_bmem_driver = {
	.driver		= {
		.name	= "brcmstb-bmem",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(brcmstb_bmem_of_match),
	},
	.probe		= brcmstb_bmem_probe,
};

module_platform_driver(brcmstb_bmem_driver);
