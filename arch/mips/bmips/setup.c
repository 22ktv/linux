/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2008 Maxime Bizon <mbizon@freebox.fr>
 * Copyright (C) 2014 Kevin Cernekee <cernekee@gmail.com>
 */

#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/bootmem.h>
#include <linux/clk-provider.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/libfdt.h>
#include <linux/smp.h>
#include <asm/addrspace.h>
#include <asm/bmips.h>
#include <asm/bootinfo.h>
#include <asm/cpu-type.h>
#include <asm/mipsregs.h>
#include <asm/prom.h>
#include <asm/smp-ops.h>
#include <asm/time.h>
#include <asm/traps.h>
#include <asm/fw/cfe/cfe_api.h>
#include <asm/fw/cfe/cfe_error.h>

#include <linux/soc/brcmstb/brcmstb.h>

#define RELO_NORMAL_VEC		BIT(18)

#define REG_BCM6328_OTP		((void __iomem *)CKSEG1ADDR(0x1000062c))
#define BCM6328_TP1_DISABLED	BIT(9)

static const unsigned long kbase = VMLINUX_LOAD_ADDRESS & 0xfff00000;

struct bmips_quirk {
	const char		*compatible;
	void			(*quirk_fn)(void);
};

static void kbase_setup(void)
{
	__raw_writel(kbase | RELO_NORMAL_VEC,
		     BMIPS_GET_CBR() + BMIPS_RELO_VECTOR_CONTROL_1);
	ebase = kbase;
}

static void bcm3384_viper_quirks(void)
{
	/*
	 * Some experimental CM boxes are set up to let CM own the Viper TP0
	 * and let Linux own TP1.  This requires moving the kernel
	 * load address to a non-conflicting region (e.g. via
	 * CONFIG_PHYSICAL_START) and supplying an alternate DTB.
	 * If we detect this condition, we need to move the MIPS exception
	 * vectors up to an area that we own.
	 *
	 * This is distinct from the OTHER special case mentioned in
	 * smp-bmips.c (boot on TP1, but enable SMP, then TP0 becomes our
	 * logical CPU#1).  For the Viper TP1 case, SMP is off limits.
	 *
	 * Also note that many BMIPS435x CPUs do not have a
	 * BMIPS_RELO_VECTOR_CONTROL_1 register, so it isn't safe to just
	 * write VMLINUX_LOAD_ADDRESS into that register on every SoC.
	 */
	board_ebase_setup = &kbase_setup;
	bmips_smp_enabled = 0;
}

static void bcm63xx_fixup_cpu1(void)
{
	/*
	 * The bootloader has set up the CPU1 reset vector at
	 * 0xa000_0200.
	 * This conflicts with the special interrupt vector (IV).
	 * The bootloader has also set up CPU1 to respond to the wrong
	 * IPI interrupt.
	 * Here we will start up CPU1 in the background and ask it to
	 * reconfigure itself then go back to sleep.
	 */
	memcpy((void *)0xa0000200, &bmips_smp_movevec, 0x20);
	__sync();
	set_c0_cause(C_SW0);
	cpumask_set_cpu(1, &bmips_booted_mask);
}

static void bcm6328_quirks(void)
{
	/* Check CPU1 status in OTP (it is usually disabled) */
	if (__raw_readl(REG_BCM6328_OTP) & BCM6328_TP1_DISABLED)
		bmips_smp_enabled = 0;
	else
		bcm63xx_fixup_cpu1();
}

static void bcm6358_quirks(void)
{
	/*
	 * BCM3368/BCM6358 need special handling for their shared TLB, so
	 * disable SMP for now
	 */
	bmips_smp_enabled = 0;
}

static void bcm6368_quirks(void)
{
	bcm63xx_fixup_cpu1();
}

static void bmips5000_pref30_quirk(void)
{
	__asm__ __volatile__(
	"	li	$8, 0x5a455048\n"
	"	.word	0x4088b00f\n"	/* mtc0 $8, $22, 15 */
	"	nop; nop; nop\n"
	"	.word	0x4008b008\n"	/* mfc0 $8, $22, 8 */
	/* disable "pref 30" on buggy CPUs */
	"	lui	$9, 0x0800\n"
	"	or	$8, $9\n"
	"	.word	0x4088b008\n"	/* mtc0 $8, $22, 8 */
	: : : "$8", "$9");
}

static const struct bmips_quirk bmips_quirk_list[] = {
	{ "brcm,bcm3368",		&bcm6358_quirks			},
	{ "brcm,bcm3384-viper",		&bcm3384_viper_quirks		},
	{ "brcm,bcm33843-viper",	&bcm3384_viper_quirks		},
	{ "brcm,bcm6328",		&bcm6328_quirks			},
	{ "brcm,bcm6358",		&bcm6358_quirks			},
	{ "brcm,bcm6362",		&bcm6368_quirks			},
	{ "brcm,bcm6368",		&bcm6368_quirks			},
	{ "brcm,bcm63168",		&bcm6368_quirks			},
	{ "brcm,bcm63268",		&bcm6368_quirks			},
	{ "brcm,bcm7344",		&bmips5000_pref30_quirk		},
	{ "brcm,bcm7346",		&bmips5000_pref30_quirk		},
	{ "brcm,bcm7425",		&bmips5000_pref30_quirk		},
	{ },
};

static char cfe_buf[COMMAND_LINE_SIZE] __initdata;

static void __init prom_init_cmdline(void)
{
	uint64_t cfe_ept, cfe_handle;
	unsigned int cfe_eptseal;
	int argc = fw_arg0;
	char **envp = (char **)fw_arg2;
	int *prom_vec = (int *)fw_arg3;

	cfe_handle = (uint64_t)(long)argc;
	cfe_ept = (long)envp;
	cfe_eptseal = (uint32_t)(unsigned long)prom_vec;

	cfe_init(cfe_handle, cfe_ept);

	if (cfe_eptseal != CFE_EPTSEAL)
		return;

	if (cfe_getenv("BOOT_FLAGS", cfe_buf, COMMAND_LINE_SIZE) == CFE_OK)
		strlcat(arcs_cmdline, cfe_buf, COMMAND_LINE_SIZE);
}

void __init prom_init(void)
{
	bmips_cpu_setup();
	prom_init_cmdline();
	register_bmips_smp_ops();
}

void __init prom_free_prom_memory(void)
{
}

const char *get_system_type(void)
{
	u32 family_id;
	u32 product_id;

	family_id  = brcmstb_get_family_id();
	product_id = brcmstb_get_product_id();

	if (family_id) {
		static char buf[128];

		snprintf(buf, sizeof(buf), "bcm%x/%c%d",
			 family_id >> 28 ? family_id >> 16 : family_id >> 8,
			 ((product_id & 0xf0) >> 4) + 'A', product_id & 0xf);

		return buf;
	} else
	return "Generic BMIPS kernel";
}

/*
 * MIPS frequency calibration
 */
#define TIMER_TIMER_IS		0x00
#define TIMER_TIMER_IE0		0x04
#define TIMER_TIMER0_CTRL	0x08
#define TIMER_TIMER1_CTRL	0x0c
#define TIMER_TIMER2_CTRL	0x10
#define TIMER_TIMER3_CTRL	0x14

/* Sampling period for MIPS calibration.  50 = 1/50 of a second. */
#define SAMPLE_PERIOD		50

static unsigned int __init bcm7xxx_cpu_frequency(void __iomem *timers_base)
{
	unsigned int freq;
	u32 value;

	__raw_writel(0, timers_base + TIMER_TIMER3_CTRL);
	(void)__raw_readl(timers_base + TIMER_TIMER3_CTRL);

	value = __raw_readl(timers_base + TIMER_TIMER_IS);
	__raw_writel(value | BIT(3), timers_base + TIMER_TIMER_IS);
	(void)__raw_readl(timers_base + TIMER_TIMER_IS);

	__raw_writel(0xc0000000 | (27000000 / SAMPLE_PERIOD),
		     timers_base + TIMER_TIMER0_CTRL);

	write_c0_count(0);

	while ((__raw_readl(timers_base + TIMER_TIMER_IS) & 1) == 0)
		;

	freq = read_c0_count();

	__raw_writel(0, timers_base + TIMER_TIMER0_CTRL);

	return (freq * SAMPLE_PERIOD);
}

void __init plat_time_init(void)
{
	struct device_node *np;
	u32 freq;

	np = of_find_node_by_name(NULL, "cpus");
	if (!np)
		panic("missing 'cpus' DT node");
	if (of_property_read_u32(np, "mips-hpt-frequency", &freq) < 0)
		panic("missing 'mips-hpt-frequency' property");
	of_node_put(np);

	mips_hpt_frequency = freq;

	np = of_find_compatible_node(NULL, NULL, "brcm,brcmstb-timers");
	if (np) {
		void __iomem *timer_base;

		timer_base = of_iomap(np, 0);
		if (timer_base) {
			mips_hpt_frequency = bcm7xxx_cpu_frequency(timer_base);
			iounmap(timer_base);
		}
		of_node_put(np);
	}
}

extern const char __appended_dtb;

void __init plat_mem_setup(void)
{
	void *dtb;
	const struct bmips_quirk *q;

	set_io_port_base(0);
	ioport_resource.start = 0;
	ioport_resource.end = ~0;

#ifdef CONFIG_MIPS_ELF_APPENDED_DTB
	if (!fdt_check_header(&__appended_dtb))
		dtb = (void *)&__appended_dtb;
	else
#endif
	/* intended to somewhat resemble ARM; see Documentation/arm/Booting */
	if (fw_arg0 == 0 && fw_arg1 == 0xffffffff)
		dtb = phys_to_virt(fw_arg2);
	else if (fw_passed_dtb) /* UHI interface */
		dtb = (void *)fw_passed_dtb;
	else if (__dtb_start != __dtb_end)
		dtb = (void *)__dtb_start;
	else
		panic("no dtb found");

	__dt_setup_arch(dtb);

	for (q = bmips_quirk_list; q->quirk_fn; q++) {
		if (of_flat_dt_is_compatible(of_get_flat_dt_root(),
					     q->compatible)) {
			q->quirk_fn();
		}
	}
}

void __init device_tree_init(void)
{
	struct device_node *np;

	unflatten_and_copy_device_tree();

	/* Disable SMP boot unless both CPUs are listed in DT and !disabled */
	np = of_find_node_by_name(NULL, "cpus");
	if (np && of_get_available_child_count(np) <= 1)
		bmips_smp_enabled = 0;
	of_node_put(np);
}

int __init plat_of_setup(void)
{
	return __dt_register_buses("simple-bus", NULL);
}

arch_initcall(plat_of_setup);

static int __init plat_dev_init(void)
{
	of_clk_init(NULL);
	return 0;
}

device_initcall(plat_dev_init);
