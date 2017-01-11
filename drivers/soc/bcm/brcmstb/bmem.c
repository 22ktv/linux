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
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>

static int brcmstb_bmem_probe(struct platform_device *pdev)
{
	int ret;

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret)
		return ret;

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

static int __init brcmstb_bmem_init(void)
{
	return platform_driver_register(&brcmstb_bmem_driver);
}

late_initcall(brcmstb_bmem_init)
