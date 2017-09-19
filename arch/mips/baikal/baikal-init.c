/*
 * Baikal-T SOC platform support code.
 *
 * Copyright (C) 2014-2016 Baikal Electronics JSC
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/bootmem.h>
#include <linux/pm.h>		/* pm_power_off */

#include <asm/fw/fw.h>
#include <asm/setup.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/sections.h>
#ifdef CONFIG_SMP
#include <asm/smp.h>
#include <asm/smp-ops.h>
#endif
#include <asm/idle.h>		/* cpu_wait */
#include <asm/reboot.h>
#include <asm/mips-cm.h>
#include <asm/mips-cpc.h>
#include <asm/cacheflush.h>
#include <asm/traps.h>
#include <asm/msa.h>
#include <asm/cdmm.h>
#include <asm/idle.h>

#include <asm/mach-baikal/hardware.h>
#include <asm/mips-boards/baikal.h> /* Base GIC and GCR addresses */

#include "common.h"

#ifdef CONFIG_KEXEC
#include <asm/kexec.h>
extern int baikal_kexec_prepare(struct kimage *);
extern void baikal_kexec_shutdown(void);
#endif

#ifndef CONFIG_MIPS_CPC
void __iomem *mips_cpc_base;
#endif

extern void baikal_be_init(void);
extern int baikal_be_handler(struct pt_regs *regs, int is_fixup);

static void __init mips_nmi_setup(void)
{
	void *base;
	extern char except_vec_nmi;

	base = (void *)(CAC_BASE + 0xa80);
	memcpy(base, &except_vec_nmi, 0x80);
	flush_icache_range((unsigned long)base, (unsigned long)base + 0x80);
}

static void __init mips_ejtag_setup(void)
{
	void *base;
	extern char except_vec_ejtag_debug;

	base = (void *)(CAC_BASE + 0x480);
	memcpy(base, &except_vec_ejtag_debug, 0x80);
	flush_icache_range((unsigned long)base, (unsigned long)base + 0x80);
}

phys_addr_t __weak mips_cpc_default_phys_base(void)
{
	return CPC_BASE_ADDR;
}

phys_addr_t __weak mips_cdmm_phys_base(void)
{
	unsigned long reg = read_c0_config3();

	if (!(reg & MIPS_CONF3_CDMM))
		return 0;
	reg = read_c0_cdmmbase() | MIPS_CDMMBASE_EN;
	write_c0_cdmmbase(reg);

	return (reg >> MIPS_CDMMBASE_ADDR_SHIFT) << MIPS_CDMMBASE_ADDR_START;
}

/*
 * Initial kernel command line, usually setup by prom_init()
 * extern char arcs_cmdline[COMMAND_LINE_SIZE];
 *
 * Registers a0, a1, a3 and a4 as passed to the kernel entry by firmware
 * extern unsigned long fw_arg0, fw_arg1, fw_arg2, fw_arg3;
 */
void __init prom_init(void)
{
	unsigned long reg;
#ifdef CONFIG_EARLY_PRINTK_8250
	setup_8250_early_printk_port(KSEG1ADDR(BAIKAL_UART0_START), 2, 1000000);
#endif
	/* Setup power management handlers */
	panic_timeout	 = 10;

	/* Setup exception handlers */
	board_nmi_handler_setup = mips_nmi_setup;
	board_ejtag_handler_setup = mips_ejtag_setup;

	/* handlers */
	board_be_init = baikal_be_init;
	board_be_handler = baikal_be_handler;

	/* Early detection of CMP support */
 	mips_cm_probe();
	mips_cpc_probe();
	/* Setup L2 prefetch */
	reg = read_gcr_l2_pft_control();
	/* Set page mask depending on actual page size */
	reg &= ~(CM_GCR_L2_PFT_CONTROL_PAGEMASK_MSK);
#if defined(CONFIG_PAGE_SIZE_4KB)
	/* 4K pages */
	reg |= 0xFFFFF000;
#elif defined(CONFIG_PAGE_SIZE_8KB)
	/* 8K pages */
	reg |= 0xFFFFE000;
#elif defined(CONFIG_PAGE_SIZE_16KB)
	/* 16K pages */
	reg |= 0xFFFFC000;
#else
	/* Other cases */
	reg |= 0xFFFFF000;
#endif
	pr_info("Enable data prefetch\n");
	write_gcr_l2_pft_control(reg | CM_GCR_L2_PFT_CONTROL_PFTEN_MSK);
	wmb();

	pr_info("Enable instruction prefetch\n");
	reg = read_gcr_l2_pft_control_b();
	write_gcr_l2_pft_control_b(reg | CM_GCR_L2_PFT_CONTROL_PFTEN_MSK);
	wmb();

#ifdef CONFIG_KEXEC
	_machine_kexec_shutdown = baikal_kexec_shutdown;
	_machine_kexec_prepare = baikal_kexec_prepare;
#endif

#ifdef CONFIG_SMP
#ifdef CONFIG_MIPS_CPS
	if (!register_cps_smp_ops())
		return;
	pr_warn("%s: register_cps_smp_ops failed\n", __func__);
#endif /* CONFIG_MIPS_CPS */
#endif /* CONFIG_SMP */
}

const char *get_system_type(void)
{
        return "Baikal-T Generic SoC";
}
