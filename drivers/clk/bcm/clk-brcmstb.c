/*
 * Copyright (C) 2009-2016 Broadcom
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#define pr_fmt(fmt) "clk-brcmstb: " fmt

#include <linux/io.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/syscore_ops.h>

static void __iomem *cpu_clk_div_reg;

struct bcm_clk_gate {
	struct clk_hw hw;
	void __iomem *reg;
	u8 bit_idx;
	u8 flags;
	u32 delay[2];
	spinlock_t *lock;
	struct clk_ops ops;
};

#define to_brcmstb_clk_gate(p) container_of(p, struct bcm_clk_gate, hw)

static DEFINE_SPINLOCK(lock);

static int cpu_clk_div_pos __initdata;
static int cpu_clk_div_width __initdata;

#ifdef CONFIG_PM_SLEEP
static u32 cpu_clk_div_reg_dump;

static int brcmstb_clk_suspend(void)
{
	if (cpu_clk_div_reg)
		cpu_clk_div_reg_dump = __raw_readl(cpu_clk_div_reg);
	return 0;
}

static void brcmstb_clk_resume(void)
{
	if (cpu_clk_div_reg)
		__raw_writel(cpu_clk_div_reg_dump, cpu_clk_div_reg);
}

static struct syscore_ops brcmstb_clk_syscore_ops = {
	.suspend = brcmstb_clk_suspend,
	.resume = brcmstb_clk_resume,
};
#endif /* CONFIG_PM_SLEEP */

static int __init parse_cpu_clk_div_dimensions(struct device_node *np)
{
	struct property *prop;
	const __be32 *p = NULL;
	int len;
	int elem_cnt;
	const char *propname = "div-shift-width";

	prop = of_find_property(np, propname, &len);
	if (!prop) {
		pr_err("%s property undefined\n", propname);
		return -EINVAL;
	}

	elem_cnt = len / sizeof(u32);

	if (elem_cnt != 2) {
		pr_err("%s should have only 2 elements\n", propname);
		return -EINVAL;
	}

	p = of_prop_next_u32(prop, p, &cpu_clk_div_pos);
	of_prop_next_u32(prop, p, &cpu_clk_div_width);

	return 0;
}

static struct clk_div_table *cpu_clk_div_table;

static int __init parse_cpu_clk_div_table(struct device_node *np)
{
	struct property *prop;
	const __be32 *p = NULL;
	struct clk_div_table *cur_tbl_ptr;
	int len;
	int elem_cnt;
	int i;
	const char *propname = "div-table";

	prop = of_find_property(np, propname, &len);
	if (!prop) {
		pr_err("%s property undefined\n", propname);
		return -EINVAL;
	}

	elem_cnt = len / sizeof(u32);

	if (elem_cnt < 2) {
		pr_err("%s should have at least 2 elements\n", propname);
		return -EINVAL;
	}

	if ((elem_cnt % 2) != 0) {
		pr_err("%s should have even number of elements\n", propname);
		return -EINVAL;
	}

	/* need room for last sentinel entry */
	len += 2 * sizeof(u32);

	cpu_clk_div_table = kmalloc(len, GFP_KERNEL);
	if (!cpu_clk_div_table)
		return -ENOMEM;

	cur_tbl_ptr = cpu_clk_div_table;

	for (i = 0; i < elem_cnt; i += 2) {
		p = of_prop_next_u32(prop, p, &cur_tbl_ptr->val);
		p = of_prop_next_u32(prop, p, &cur_tbl_ptr->div);

		cur_tbl_ptr++;
	}

	/* last entry should be zeroed out */
	cur_tbl_ptr->val = 0;
	cur_tbl_ptr->div = 0;

	return 0;
}

static void __init of_brcmstb_cpu_clk_div_setup(struct device_node *np)
{
	struct clk *clk;
	int rc;

	cpu_clk_div_reg = of_iomap(np, 0);
	if (!cpu_clk_div_reg) {
		pr_err("unable to iomap cpu clk divider register!\n");
		return;
	}

	rc = parse_cpu_clk_div_dimensions(np);
	if (rc)
		goto err;

	rc = parse_cpu_clk_div_table(np);
	if (rc)
		goto err;

	clk = clk_register_divider_table(NULL, "cpu-clk-div",
					 of_clk_get_parent_name(np, 0), 0,
					 cpu_clk_div_reg,
					 cpu_clk_div_pos, cpu_clk_div_width,
					 0, cpu_clk_div_table, &lock);
	if (IS_ERR(clk))
		goto err;

	rc = of_clk_add_provider(np, of_clk_src_simple_get, clk);
	if (rc) {
		pr_err("error adding clock provider (%d)\n", rc);
		goto err;
	}

#ifdef CONFIG_PM_SLEEP
	register_syscore_ops(&brcmstb_clk_syscore_ops);
#endif
	return;

err:
	kfree(cpu_clk_div_table);
	cpu_clk_div_table = NULL;

	if (cpu_clk_div_reg) {
		iounmap(cpu_clk_div_reg);
		cpu_clk_div_reg = NULL;
	}
}
CLK_OF_DECLARE(brcmstb_cpu_clk_div, "brcm,brcmstb-cpu-clk-div",
		of_brcmstb_cpu_clk_div_setup);

/*
 * It works on following logic:
 *
 * For enabling clock, enable = 1
 *	set2dis = 1	-> clear bit	-> set = 0
 *	set2dis = 0	-> set bit	-> set = 1
 *
 * For disabling clock, enable = 0
 *	set2dis = 1	-> set bit	-> set = 1
 *	set2dis = 0	-> clear bit	-> set = 0
 *
 * So, result is always: enable xor set2dis.
 */
static void brcmstb_clk_gate_endisable(struct clk_hw *hw, int enable)
{
	struct bcm_clk_gate *gate = to_brcmstb_clk_gate(hw);
	int set = gate->flags & CLK_GATE_SET_TO_DISABLE ? 1 : 0;
	unsigned long flags = 0;
	u32 reg;

	set ^= enable;

	if (gate->lock)
		spin_lock_irqsave(gate->lock, flags);

	reg = readl(gate->reg);

	if (set)
		reg |= BIT(gate->bit_idx);
	else
		reg &= ~BIT(gate->bit_idx);

	writel(reg, gate->reg);

	if (set == 0 && gate->delay[0])
		udelay(gate->delay[0]);
	else if (set == 1 && gate->delay[1])
		udelay(gate->delay[1]);

	if (gate->lock)
		spin_unlock_irqrestore(gate->lock, flags);
}

static int brcmstb_clk_gate_enable(struct clk_hw *hw)
{
	brcmstb_clk_gate_endisable(hw, 1);
	return 0;
}

static void brcmstb_clk_gate_disable(struct clk_hw *hw)
{
	brcmstb_clk_gate_endisable(hw, 0);
}

static int brcmstb_clk_gate_is_enabled(struct clk_hw *hw)
{
	u32 reg;
	struct bcm_clk_gate *gate = to_brcmstb_clk_gate(hw);

	reg = readl(gate->reg);

	/* if a set bit disables this clk, flip it before masking */
	if (gate->flags & CLK_GATE_SET_TO_DISABLE)
		reg ^= BIT(gate->bit_idx);

	reg &= BIT(gate->bit_idx);
	return reg ? 1 : 0;
}

static const struct clk_ops brcmstb_clk_gate_ops = {
	.enable = brcmstb_clk_gate_enable,
	.disable = brcmstb_clk_gate_disable,
	.is_enabled = brcmstb_clk_gate_is_enabled,
};

static const struct clk_ops brcmstb_clk_gate_inhib_dis_ops = {
	.enable = brcmstb_clk_gate_enable,
	.is_enabled = brcmstb_clk_gate_is_enabled,
};

static const struct clk_ops brcmstb_clk_gate_ro_ops = {
	.is_enabled = brcmstb_clk_gate_is_enabled,
};

/**
 * brcm_clk_gate_register - register a bcm gate clock with the clock framework.
 * @dev: device that is registering this clock
 * @name: name of this clock
 * @parent_name: name of this clock's parent
 * @flags: framework-specific flags for this clock
 * @reg: register address to control gating of this clock
 * @bit_idx: which bit in the register controls gating of this clock
 * @clk_gate_flags: gate-specific flags for this clock
 * @delay: usec delay in turning on, off.
 * @lock: shared register lock for this clock
 */
static struct clk __init *brcm_clk_gate_register(
	struct device *dev, const char *name, const char *parent_name,
	unsigned long flags, void __iomem *reg, u8 bit_idx,
	u8 clk_gate_flags, u32 delay[2], spinlock_t *lock,
	bool read_only, bool inhibit_disable)
{
	struct bcm_clk_gate *gate;
	struct clk *clk;
	struct clk_init_data init;

	/* allocate the gate */
	gate = kzalloc(sizeof(struct bcm_clk_gate), GFP_KERNEL);
	if (!gate) {
		pr_err("%s: could not allocate bcm gated clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = inhibit_disable ? &brcmstb_clk_gate_inhib_dis_ops
		: read_only ? &brcmstb_clk_gate_ro_ops : &brcmstb_clk_gate_ops;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);
	init.flags = flags;
	init.flags |= CLK_IGNORE_UNUSED;

	/* struct bcm_gate assignments */
	gate->reg = reg;
	gate->bit_idx = bit_idx;
	gate->flags = clk_gate_flags;
	gate->lock = lock;
	gate->delay[0] = delay[0];
	gate->delay[1] = delay[1];
	gate->hw.init = &init;

	clk = clk_register(dev, &gate->hw);

	if (IS_ERR(clk))
		kfree(gate);

	return clk;
}

/**
 * of_brcmstb_gate_clk_setup() - Setup function for brcmstb gate clock
 */
static void __init of_brcmstb_clk_gate_setup(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	void __iomem *reg;
	const char *parent_name;
	u8 clk_gate_flags = 0;
	u32 bit_idx = 0;
	u32 delay[2] = {0, 0};
	int ret;
	bool read_only = false;
	bool inhibit_disable = false;
	unsigned long flags = 0;

	of_property_read_string(node, "clock-output-names", &clk_name);
	parent_name = of_clk_get_parent_name(node, 0);
	if (of_property_read_u32(node, "bit-shift", &bit_idx)) {
		pr_err("%s: missing bit-shift property for %s\n",
				__func__, node->name);
		return;
	}
	reg = of_iomap(node, 0);
	if (!reg) {
		pr_err("unable to iomap cpu clk divider register!\n");
		return;
	}

	of_property_read_u32_array(node, "brcm,delay", delay, 2);

	if (of_property_read_bool(node, "set-bit-to-disable"))
		clk_gate_flags |= CLK_GATE_SET_TO_DISABLE;

	if (of_property_read_bool(node, "brcm,read-only"))
		read_only = true;

	if (of_property_read_bool(node, "brcm,inhibit-disable"))
		inhibit_disable = true;

	if (of_property_read_bool(node, "brcm,set-rate-parent"))
		flags |= CLK_SET_RATE_PARENT;

	clk = brcm_clk_gate_register(NULL, clk_name, parent_name, flags, reg,
				     (u8) bit_idx, clk_gate_flags, delay,
				     &lock, read_only, inhibit_disable);
	if (!IS_ERR(clk)) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		ret = clk_register_clkdev(clk, clk_name, NULL);
		if (ret)
			pr_err("%s: clk device registration failed for '%s'\n",
			       __func__, clk_name);
	}
}
CLK_OF_DECLARE(brcmstb_clk_gate, "brcm,brcmstb-gate-clk",
		of_brcmstb_clk_gate_setup);

struct bcm_clk_sw {
	struct clk_hw hw;
	u8 parent;
	struct clk_ops ops;
};

#define to_brcmstb_clk_sw(p) container_of(p, struct bcm_clk_sw, hw)

static u8 brcmstb_clk_sw_get_parent(struct clk_hw *hw)
{
	struct bcm_clk_sw *sw_clk = to_brcmstb_clk_sw(hw);

	return sw_clk->parent;
}

static int brcmstb_clk_sw_set_parent(struct clk_hw *hw, u8 index)
{
	struct bcm_clk_sw *sw_clk = to_brcmstb_clk_sw(hw);

	sw_clk->parent = index;
	return 0;
}

static const struct clk_ops brcmstb_clk_sw_ops = {
	.get_parent = brcmstb_clk_sw_get_parent,
	.set_parent = brcmstb_clk_sw_set_parent,
};

/**
 * brcmstb_clk_sw_register - register a bcm gate clock with the clock framework.
 * @dev: device that is registering this clock
 * @name: name of this clock
 * @parents: name of this clock's parents; not known by clock framework
 * @num_parents: number of parents
 * @flags: framework-specific flags for this clock
 * @lock: shared register lock for this clock
 */
static struct clk __init *brcmstb_clk_sw_register(
	struct device *dev, const char *name, const char **parent_names,
	int num_parents, unsigned long flags, spinlock_t *lock)
{
	struct bcm_clk_sw *sw_clk;
	struct clk *clk;
	struct clk_init_data init;

	/* allocate the gate */
	sw_clk = kzalloc(sizeof(struct bcm_clk_sw), GFP_KERNEL);
	if (!sw_clk) {
		pr_err("%s: could not allocate bcm sw clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &brcmstb_clk_sw_ops;
	init.parent_names = parent_names;
	init.num_parents = num_parents;
	init.flags = flags | CLK_IS_SW;
	init.flags |= CLK_IGNORE_UNUSED;

	sw_clk->hw.init = &init;
	clk = clk_register(dev, &sw_clk->hw);
	if (IS_ERR(clk))
		kfree(sw_clk);
	return clk;
}

static void __init of_brcmstb_clk_sw_setup(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	int num_parents;
	const char **parent_names;
	int ret, i;

	of_property_read_string(node, "clock-output-names", &clk_name);
	num_parents = of_property_count_strings(node, "clock-names");
	if (num_parents < 1) {
		pr_err("%s: brcm-sw-clock %s must have parent(s)\n",
				__func__, node->name);
		return;
	}
	parent_names = kzalloc((sizeof(char *) * num_parents),
			GFP_KERNEL);
	if (!parent_names) {
		pr_err("%s: failed to alloc parent_names\n", __func__);
		return;
	}

	for (i = 0; i < num_parents; i++)
		parent_names[i] = of_clk_get_parent_name(node, i);

	clk = brcmstb_clk_sw_register(NULL, clk_name, parent_names, num_parents,
				   0, NULL);
	kfree(parent_names);

	if (!IS_ERR(clk)) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		ret = clk_register_clkdev(clk, clk_name, NULL);
		if (ret)
			pr_err("%s: clk device registration failed for '%s'\n",
			       __func__, clk_name);
	}
}
CLK_OF_DECLARE(brcmstb_clk_sw, "brcm,brcmstb-sw-clk",
		of_brcmstb_clk_sw_setup);

#define to_clk_mux(_hw) container_of(_hw, struct clk_mux, hw)

static struct clk_ops clk_mux_ops_brcm = {
	.determine_rate = __clk_mux_determine_rate_closest,
};

static struct clk *clk_register_mux_table_brcm(struct device *dev,
		const char *name, const char **parent_names, u8 num_parents,
		unsigned long flags, void __iomem *reg, u8 shift, u32 mask,
		u8 clk_mux_flags, u32 *table, spinlock_t *lock)
{
	struct clk_mux *mux;
	struct clk *clk;
	struct clk_init_data init;
	u8 width = 0;

	if (clk_mux_ops_brcm.get_parent == NULL) {
		/* we would like to set these at compile time but
		 * that is not possible */
		clk_mux_ops_brcm.get_parent = clk_mux_ops.get_parent;
		clk_mux_ops_brcm.set_parent = clk_mux_ops.set_parent;
	}

	if (clk_mux_flags & CLK_MUX_HIWORD_MASK) {
		width = fls(mask) - ffs(mask) + 1;
		if (width + shift > 16) {
			pr_err("mux value exceeds LOWORD field\n");
			return ERR_PTR(-EINVAL);
		}
	}

	/* allocate the mux */
	mux = kzalloc(sizeof(struct clk_mux), GFP_KERNEL);
	if (!mux)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	if (clk_mux_flags & CLK_MUX_READ_ONLY)
		init.ops = &clk_mux_ro_ops;
	else
		init.ops = &clk_mux_ops_brcm;
	init.flags = flags;
	init.parent_names = parent_names;
	init.num_parents = num_parents;

	/* struct clk_mux assignments */
	mux->reg = reg;
	mux->shift = shift;
	mux->mask = mask;
	mux->flags = clk_mux_flags;
	mux->lock = lock;
	mux->table = table;
	mux->hw.init = &init;

	clk = clk_register(dev, &mux->hw);

	if (IS_ERR(clk))
		kfree(mux);

	return clk;
}

/**
 * of_mux_clk_setup_brcm() - Setup function for simple mux rate clock
 */
void of_mux_clk_setup_brcm(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	void __iomem *reg;
	int num_parents;
	const char **parent_names;
	int i;
	u8 clk_mux_flags = 0;
	u32 mask = 0;
	u32 shift = 0;

	of_property_read_string(node, "clock-output-names", &clk_name);

	num_parents = of_clk_get_parent_count(node);
	if (num_parents < 1) {
		pr_err("%s: mux-clock %s must have parent(s)\n",
				__func__, node->name);
		return;
	}

	parent_names = kzalloc((sizeof(char *) * num_parents),
			GFP_KERNEL);

	if (!parent_names) {
		pr_err("%s: could not allocate parent names\n", __func__);
		return;
	}

	for (i = 0; i < num_parents; i++)
		parent_names[i] = of_clk_get_parent_name(node, i);

	reg = of_iomap(node, 0);
	if (!reg) {
		pr_err("%s: no memory mapped for property reg\n", __func__);
		goto fail;
	}

	if (of_property_read_u32(node, "bit-mask", &mask)) {
		pr_err("%s: missing bit-mask property for %s\n",
		       __func__, node->name);
		goto fail;
	}

	if (of_property_read_u32(node, "bit-shift", &shift)) {
		shift = __ffs(mask);
		pr_debug("%s: bit-shift property defaults to 0x%x for %s\n",
				__func__, shift, node->name);
	}

	if (of_property_read_bool(node, "index-starts-at-one"))
		clk_mux_flags |= CLK_MUX_INDEX_ONE;

	if (of_property_read_bool(node, "hiword-mask"))
		clk_mux_flags |= CLK_MUX_HIWORD_MASK;

	clk = clk_register_mux_table_brcm(NULL, clk_name, parent_names,
			num_parents, 0, reg, shift,
			mask, clk_mux_flags, NULL, NULL);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);

	return;
fail:
	kfree(parent_names);
}
EXPORT_SYMBOL_GPL(of_mux_clk_setup_brcm);
CLK_OF_DECLARE(mux_clk, "brcm,mux-clock", of_mux_clk_setup_brcm);

static struct clk_div_table *of_clk_get_div_table(struct device_node *node)
{
	int i;
	unsigned int table_size;
	struct clk_div_table *table;
	const __be32 *tablespec;
	u32 val;

	tablespec = of_get_property(node, "table", (int *) &table_size);

	if (!tablespec)
		return NULL;

	table_size /= sizeof(struct clk_div_table);

	table = kzalloc(sizeof(struct clk_div_table) * table_size, GFP_KERNEL);
	if (!table) {
		pr_err("%s: unable to allocate memory for %s table\n", __func__,
		       node->name);
		return NULL;
	}

	for (i = 0; i < table_size; i++) {
		of_property_read_u32_index(node, "table", i * 2, &val);
		table[i].div = val;
		of_property_read_u32_index(node, "table", i * 2 + 1, &val);
		table[i].val = val;
	}

	return table;
}

/**
 * of_divider_clk_setup() - Setup function for simple div rate clock
 */
static void __init of_divider_clk_setup(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	void __iomem *reg;
	const char *parent_name;
	u8 clk_divider_flags = 0;
	u32 mask = 0;
	u32 shift = 0;
	u32 width;
	struct clk_div_table *table;

	of_property_read_string(node, "clock-output-names", &clk_name);

	parent_name = of_clk_get_parent_name(node, 0);

	reg = of_iomap(node, 0);
	if (!reg) {
		pr_err("%s: no memory mapped for property reg\n", __func__);
		return;
	}

	if (of_property_read_u32(node, "bit-mask", &mask)) {
		pr_err("%s: missing bit-mask property for %s\n", __func__,
		       node->name);
		return;
	}
	width = fls(mask);
	if ((1 << width) - 1 != mask) {
		pr_err("%s: bad bit-mask for %s\n", __func__, node->name);
		return;
	}

	if (of_property_read_u32(node, "bit-shift", &shift)) {
		shift = __ffs(mask);
		pr_debug("%s: bit-shift property defaults to 0x%x for %s\n",
				__func__, shift, node->name);
	}

	if (of_property_read_bool(node, "index-starts-at-one"))
		clk_divider_flags |= CLK_DIVIDER_ONE_BASED;

	if (of_property_read_bool(node, "index-power-of-two"))
		clk_divider_flags |= CLK_DIVIDER_POWER_OF_TWO;

	if (of_property_read_bool(node, "index-allow-zero"))
		clk_divider_flags |= CLK_DIVIDER_ALLOW_ZERO;

	if (of_property_read_bool(node, "index-max-mult-at-zero"))
		clk_divider_flags |= CLK_DIVIDER_MAX_AT_ZERO;

	if (of_property_read_bool(node, "hiword-mask"))
		clk_divider_flags |= CLK_DIVIDER_HIWORD_MASK;

	table = of_clk_get_div_table(node);
	if (IS_ERR(table))
		return;

	clk = clk_register_divider_table(NULL, clk_name, parent_name, 0, reg, shift,
			width, clk_divider_flags, table, NULL);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(divider_clk, "divider-clock", of_divider_clk_setup);

#define CLK_MULTIPLIER_ONE_BASED	BIT(0)
#define CLK_MULTIPLIER_POWER_OF_TWO	BIT(1)
#define CLK_MULTIPLIER_ALLOW_ZERO	BIT(2)
#define CLK_MULTIPLIER_HIWORD_MASK	BIT(3)
#define CLK_MULTIPLIER_READ_ONLY	BIT(4)
#define CLK_MULTIPLIER_MAX_MULT_AT_ZERO	BIT(5)

/*
 * DOC: basic adjustable multiplier clock that cannot gate
 *
 * Traits of this clock:
 * prepare - clk_prepare only ensures that parents are prepared
 * enable - clk_enable only ensures that parents are enabled
 * rate - rate is adjustable.  clk->rate = (parent->rate * multiplier)
 * parent - fixed parent.  No clk_set_parent support
 */

#define to_clk_multiplier(_hw) container_of(_hw, struct clk_multiplier, hw)

#define mult_mask(width)	((1 << (width)) - 1)

static unsigned int _get_table_minmult(const struct clk_mult_table *table)
{
	unsigned int minmult = UINT_MAX;
	const struct clk_mult_table *clkt;

	for (clkt = table; clkt->mult; clkt++)
		if (clkt->mult < minmult)
			minmult = clkt->mult;
	return minmult;
}

static unsigned int _get_table_maxmult(const struct clk_mult_table *table)
{
	unsigned int maxmult = 0;
	const struct clk_mult_table *clkt;

	for (clkt = table; clkt->mult; clkt++)
		if (clkt->mult > maxmult)
			maxmult = clkt->mult;
	return maxmult;
}

static unsigned int _get_minmult(const struct clk_mult_table *table)
{
	if (table)
		return _get_table_minmult(table);
	return 1;
}

static unsigned int _get_maxmult(const struct clk_mult_table *table, u8 width,
				unsigned long flags)
{
	if (flags & CLK_MULTIPLIER_ONE_BASED)
		return mult_mask(width);
	if (flags & CLK_MULTIPLIER_POWER_OF_TWO)
		return 1 << mult_mask(width);
	if (table)
		return _get_table_maxmult(table);
	return mult_mask(width) + 1;
}

static unsigned int _get_table_mult(const struct clk_mult_table *table,
							unsigned int val)
{
	const struct clk_mult_table *clkt;

	for (clkt = table; clkt->mult; clkt++)
		if (clkt->val == val)
			return clkt->mult;
	return 0;
}

static unsigned int _get_mult(const struct clk_mult_table *table,
			      u8 width, unsigned int val, unsigned long flags)
{
	if (flags & CLK_MULTIPLIER_ONE_BASED)
		return val;
	if (flags & CLK_MULTIPLIER_POWER_OF_TWO)
		return 1 << val;
	if (flags & CLK_MULTIPLIER_MAX_MULT_AT_ZERO)
		return val ? val : mult_mask(width) + 1;
	if (table)
		return _get_table_mult(table, val);
	return val + 1;
}

static unsigned int _get_table_val(const struct clk_mult_table *table,
				   unsigned int mult)
{
	const struct clk_mult_table *clkt;

	for (clkt = table; clkt->mult; clkt++)
		if (clkt->mult == mult)
			return clkt->val;
	return 0;
}

static unsigned int _get_val(const struct clk_mult_table *table,
			     u8 width, unsigned int mult, unsigned long flags)
{
	if (flags & CLK_MULTIPLIER_ONE_BASED)
		return mult;
	if (flags & CLK_MULTIPLIER_POWER_OF_TWO)
		return __ffs(mult);
	if (flags & CLK_MULTIPLIER_MAX_MULT_AT_ZERO)
		return (mult == mult_mask(width) + 1)
			? 0 : mult;
	if (table)
		return  _get_table_val(table, mult);
	return mult - 1;
}

static unsigned long multiplier_recalc_rate(struct clk_hw *hw,
				     unsigned long parent_rate,
				     unsigned int val,
				     const struct clk_mult_table *table,
				     unsigned long flags)
{
	struct clk_multiplier *multiplier = to_clk_multiplier(hw);
	unsigned int mult;

	mult = _get_mult(table, multiplier->width, val, flags);
	if (!mult) {
		WARN(!(flags & CLK_MULTIPLIER_ALLOW_ZERO),
			"%s: Zero multiplier and CLK_MULTIPLIER_ALLOW_ZERO not set\n",
			__clk_get_name(hw->clk));
		return parent_rate;
	}

	return parent_rate * mult;
}

static unsigned long clk_multiplier_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_multiplier *multiplier = to_clk_multiplier(hw);
	unsigned int val;

	val = readl(multiplier->reg) >> multiplier->shift;
	val &= mult_mask(multiplier->width);

	return multiplier_recalc_rate(hw, parent_rate, val, multiplier->table,
				   multiplier->flags);
}

static bool _is_valid_table_mult(const struct clk_mult_table *table,
							 unsigned int mult)
{
	const struct clk_mult_table *clkt;

	for (clkt = table; clkt->mult; clkt++)
		if (clkt->mult == mult)
			return true;
	return false;
}

static bool _is_valid_mult(const struct clk_mult_table *table,
			   unsigned int mult, unsigned long flags)
{
	if (flags & CLK_MULTIPLIER_POWER_OF_TWO)
		return is_power_of_2(mult);
	if (table)
		return _is_valid_table_mult(table, mult);
	return true;
}

static int clk_multiplier_bestmult(struct clk_hw *hw, unsigned long rate,
			       unsigned long *best_parent_rate,
			       const struct clk_mult_table *table, u8 width,
			       unsigned long flags)
{
	int i, bestmult = 0;
	unsigned long parent_rate, best = 0, now, maxmult, minmult;
	unsigned long parent_rate_saved = *best_parent_rate;

	if (!rate)
		rate = 1;

	minmult = _get_minmult(table);
	maxmult = _get_maxmult(table, width, flags);

	if (!(clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT)) {
		parent_rate = *best_parent_rate;
		bestmult = rate / parent_rate;
		bestmult = bestmult == 0 ? minmult : bestmult;
		bestmult = bestmult > maxmult ? maxmult : bestmult;
		return bestmult;
	}

	/*
	 * The maximum multiplier we can use without overflowing
	 * unsigned long in rate * i below
	 */
	maxmult = min(ULONG_MAX / parent_rate_saved, maxmult);

	for (i = 1; i <= maxmult; i++) {
		if (!_is_valid_mult(table, i, flags))
			continue;
		if (rate == parent_rate_saved * i) {
			/*
			 * It's the most ideal case if the requested rate can be
			 * multiplied from parent clock without needing to
			 * change the parent rate, so return the multiplier
			 * immediately.
			 */
			*best_parent_rate = parent_rate_saved;
			return i;
		}
		parent_rate = clk_hw_round_rate(clk_hw_get_parent(hw),
					       rate / i);
		now = parent_rate * i;
		if (now <= rate && now > best) {
			bestmult = i;
			best = now;
			*best_parent_rate = parent_rate;
		}
	}

	if (!bestmult) {
		bestmult = _get_minmult(table);
		*best_parent_rate
			= clk_hw_round_rate(clk_hw_get_parent(hw), 1);
	}

	return bestmult;
}

static long multiplier_round_rate(struct clk_hw *hw, unsigned long rate,
			   unsigned long *prate,
			   const struct clk_mult_table *table,
			   u8 width, unsigned long flags)
{
	int mult;

	mult = clk_multiplier_bestmult(hw, rate, prate, table, width, flags);

	return *prate * mult;
}

static long clk_multiplier_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	struct clk_multiplier *multiplier = to_clk_multiplier(hw);
	int bestmult;

	/* if read only, just return current value */
	if (multiplier->flags & CLK_MULTIPLIER_READ_ONLY) {
		bestmult = readl(multiplier->reg) >> multiplier->shift;
		bestmult &= mult_mask(multiplier->width);
		bestmult = _get_mult(multiplier->table, multiplier->width,
				     bestmult, multiplier->flags);
		if ((clk_hw_get_flags(hw) & CLK_SET_RATE_PARENT))
			*prate = clk_hw_round_rate(clk_hw_get_parent(hw),
						  rate);
		return *prate * bestmult;
	}

	return multiplier_round_rate(hw, rate, prate, multiplier->table,
				  multiplier->width, multiplier->flags);
}

static int multiplier_get_val(unsigned long rate, unsigned long parent_rate,
		    const struct clk_mult_table *table, u8 width,
		    unsigned long flags)
{
	unsigned int mult, value;

	mult = rate / parent_rate;
	value = _get_val(table, width, mult, flags);
	return min_t(unsigned int, value, mult_mask(width));
}

static int clk_multiplier_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct clk_multiplier *multiplier = to_clk_multiplier(hw);
	unsigned int value;
	unsigned long flags = 0;
	u32 val;

	value = multiplier_get_val(rate, parent_rate, multiplier->table,
				multiplier->width, multiplier->flags);

	if (multiplier->lock)
		spin_lock_irqsave(multiplier->lock, flags);

	if (multiplier->flags & CLK_MULTIPLIER_HIWORD_MASK) {
		val = mult_mask(multiplier->width) << (multiplier->shift + 16);
	} else {
		val = readl(multiplier->reg);
		val &= ~(mult_mask(multiplier->width) << multiplier->shift);
	}
	val |= value << multiplier->shift;
	writel(val, multiplier->reg);

	if (multiplier->lock)
		spin_unlock_irqrestore(multiplier->lock, flags);

	return 0;
}

static const struct clk_ops bcm_clk_multiplier_ops = {
	.recalc_rate = clk_multiplier_recalc_rate,
	.round_rate = clk_multiplier_round_rate,
	.set_rate = clk_multiplier_set_rate,
};

static const struct clk_ops bcm_clk_multiplier_ro_ops = {
	.recalc_rate = clk_multiplier_recalc_rate,
};

static struct clk *_register_multiplier(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		void __iomem *reg, u8 shift, u8 width,
		u8 clk_multiplier_flags, const struct clk_mult_table *table,
		spinlock_t *lock)
{
	struct clk_multiplier *mult;
	struct clk *clk;
	struct clk_init_data init;

	if (clk_multiplier_flags & CLK_MULTIPLIER_HIWORD_MASK) {
		if (width + shift > 16) {
			pr_warn("multiplier value exceeds LOWORD field\n");
			return ERR_PTR(-EINVAL);
		}
	}

	/* allocate the multiplier */
	mult = kzalloc(sizeof(struct clk_multiplier), GFP_KERNEL);
	if (!mult)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	if (clk_multiplier_flags & CLK_MULTIPLIER_READ_ONLY)
		init.ops = &bcm_clk_multiplier_ro_ops;
	else
		init.ops = &bcm_clk_multiplier_ops;
	init.flags = flags;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	/* struct clk_multiplier assignments */
	mult->reg = reg;
	mult->shift = shift;
	mult->width = width;
	mult->flags = clk_multiplier_flags;
	mult->lock = lock;
	mult->hw.init = &init;
	mult->table = table;

	/* register the clock */
	clk = clk_register(dev, &mult->hw);

	if (IS_ERR(clk))
		kfree(mult);

	return clk;
}

static struct clk_mult_table *of_clk_get_mult_table(struct device_node *node)
{
	int i;
	unsigned int table_size;
	struct clk_mult_table *table;
	const __be32 *tablespec;
	u32 val;

	tablespec = of_get_property(node, "table", (int *) &table_size);

	if (!tablespec)
		return NULL;

	table_size /= sizeof(struct clk_mult_table);

	if (!table_size) {
		pr_err("%s: %s table has zero length\n", __func__,
		       node->name);
		return ERR_PTR(-EINVAL);
	}

	table = kzalloc(sizeof(struct clk_mult_table) * (table_size + 1),
			GFP_KERNEL);
	if (!table) {
		pr_err("%s: unable to allocate memory for %s table\n", __func__,
		       node->name);
		return NULL;
	}

	for (i = 0; i < table_size; i++) {
		of_property_read_u32_index(node, "table", i * 2, &val);
		table[i].mult = val;
		of_property_read_u32_index(node, "table", i * 2 + 1, &val);
		table[i].val = val;
	}

	return table;
}

/**
 * of_multiplier_clk_setup() - Setup function for simple mult rate clock
 */
static void __init of_multiplier_clk_setup(struct device_node *node)
{
	struct clk *clk;
	const char *clk_name = node->name;
	void __iomem *reg;
	const char *parent_name;
	u8 clk_multiplier_flags = 0;
	u32 mask = 0;
	u32 shift = 0;
	u32 width;
	struct clk_mult_table *table;

	of_property_read_string(node, "clock-output-names", &clk_name);

	parent_name = of_clk_get_parent_name(node, 0);

	reg = of_iomap(node, 0);
	if (!reg) {
		pr_err("%s: no memory mapped for property reg\n", __func__);
		return;
	}

	if (of_property_read_u32(node, "bit-mask", &mask)) {
		pr_err("%s: missing bit-mask property for %s\n", __func__,
		       node->name);
		return;
	}
	width = fls(mask);
	if ((1 << width) - 1 != mask) {
		pr_err("%s: bad bit-mask for %s\n", __func__, node->name);
		return;
	}

	if (of_property_read_u32(node, "bit-shift", &shift)) {
		shift = __ffs(mask);
		pr_debug("%s: bit-shift property defaults to 0x%x for %s\n",
				__func__, shift, node->name);
	}

	if (of_property_read_bool(node, "index-starts-at-one"))
		clk_multiplier_flags |= CLK_MULTIPLIER_ONE_BASED;

	if (of_property_read_bool(node, "index-power-of-two"))
		clk_multiplier_flags |= CLK_MULTIPLIER_POWER_OF_TWO;

	if (of_property_read_bool(node, "index-allow-zero"))
		clk_multiplier_flags |= CLK_MULTIPLIER_ALLOW_ZERO;

	if (of_property_read_bool(node, "index-max-mult-at-zero"))
		clk_multiplier_flags |= CLK_MULTIPLIER_MAX_MULT_AT_ZERO;

	table = of_clk_get_mult_table(node);
	if (IS_ERR(table))
		return;

	clk = _register_multiplier(NULL, clk_name, parent_name, 0, reg, shift,
			width, clk_multiplier_flags, table, NULL);

	if (!IS_ERR(clk))
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
}
CLK_OF_DECLARE(multiplier_clk, "multiplier-clock", of_multiplier_clk_setup);
