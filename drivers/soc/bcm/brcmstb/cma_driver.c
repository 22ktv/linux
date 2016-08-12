/*
 *  cma_driver.c - Broadcom STB platform CMA driver
 *
 *  Copyright © 2009 - 2015 Broadcom Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  A copy of the GPL is available at
 *  http://www.broadcom.com/licenses/GPLv2.php or from the Free Software
 *  Foundation at https://www.gnu.org/licenses/ .
 */

#include <linux/module.h>
#include <linux/cma.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>

struct device *cma_dev[MAX_CMA_AREAS];

struct device *brcmstb_cma_get_dev(int region_num)
{
	if (region_num >= ARRAY_SIZE(cma_dev))
		return NULL;

	return cma_dev[region_num];
}
EXPORT_SYMBOL(brcmstb_cma_get_dev);

static int brcmstb_cma_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int region_num;
	int ret;

	ret = of_property_read_u32(dev->of_node, "brcm,region-num",
				   &region_num);
	if (ret)
		return ret;

	ret = of_reserved_mem_device_init(dev);
	if (ret)
		return ret;

	cma_dev[region_num] = dev;

	return 0;
}

static const struct of_device_id brcmstb_cma_of_match[] = {
	{ .compatible = "brcm,cma-plat-dev" },
	{},
};
MODULE_DEVICE_TABLE(of, brcmstb_cma_of_match);

static struct platform_driver brcmstb_cma_driver = {
	.driver		= {
		.name	= "brcm-cma",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(brcmstb_cma_of_match),
	},
	.probe		= brcmstb_cma_probe,
};

module_platform_driver(brcmstb_cma_driver);
