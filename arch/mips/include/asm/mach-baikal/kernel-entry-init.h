/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Chris Dearman (chris@mips.com)
 * Copyright (C) 2007 Mips Technologies, Inc.
 * Copyright (C) 2014 Imagination Technologies Ltd.
 */
#ifndef __ASM_MACH_MIPS_KERNEL_ENTRY_INIT_H
#define __ASM_MACH_MIPS_KERNEL_ENTRY_INIT_H

#include <asm/regdef.h>
#include <asm/mipsregs.h>

	/*
	 * Prepare segments for EVA boot:
	 *
	 * This is in case the processor boots in legacy configuration
	 * (SI_EVAReset is de-asserted and CONFIG5.K == 0)
	 *
	 * ========================= Mappings =============================
	 * Virtual memory           Physical memory           Mapping
	 * 0x00000000 - 0x7fffffff  0x80000000 - 0xfffffffff   MUSUK (kuseg)
	 *                          Flat 2GB physical memory
	 *
	 * 0x80000000 - 0x9fffffff  0x00000000 - 0x1ffffffff   MUSUK (kseg0)
	 * 0xa0000000 - 0xbf000000  0x00000000 - 0x1ffffffff   MUSUK (kseg1)
	 * 0xc0000000 - 0xdfffffff             -                 MK  (kseg2)
	 * 0xe0000000 - 0xffffffff             -                 MK  (kseg3)
	 *
	 *
	 * Lowmem is expanded to 2GB
	 *
	 * The following code uses the t0, t1, t2 and ra registers without
	 * previously preserving them.
	 *
	 */
	.macro	platform_eva_init

	.set	push
	.set	reorder
#if 0
	/*
	 * Get Config.K0 value and use it to program
	 * the segmentation registers
	 */
	mfc0    t1, CP0_CONFIG
	andi	t1, 0x7 /* CCA */
#endif
	/*
	 * Directly use cacheable, coherent, write-back,
	 * write-allocate, read misses request shared attribute
	 */
	li      t1, 0x5
	move	t2, t1
	ins	t2, t1, 16, 3
	/* SegCtl0 */
	li      t0, ((MIPS_SEGCFG_MK << MIPS_SEGCFG_AM_SHIFT) |		\
		(0 << MIPS_SEGCFG_PA_SHIFT) |				\
		(1 << MIPS_SEGCFG_EU_SHIFT)) |				\
		(((MIPS_SEGCFG_MK << MIPS_SEGCFG_AM_SHIFT) |		\
		(0 << MIPS_SEGCFG_PA_SHIFT) |				\
		(1 << MIPS_SEGCFG_EU_SHIFT)) << 16)
	or	t0, t2
	mtc0	t0, CP0_PAGEMASK, 2

	/* SegCtl1 */
	li      t0, ((MIPS_SEGCFG_MUSUK << MIPS_SEGCFG_AM_SHIFT) |	\
		(0 << MIPS_SEGCFG_PA_SHIFT) |				\
		(2 << MIPS_SEGCFG_C_SHIFT) |				\
		(1 << MIPS_SEGCFG_EU_SHIFT)) |				\
		(((MIPS_SEGCFG_MUSUK << MIPS_SEGCFG_AM_SHIFT) |		\
		(0 << MIPS_SEGCFG_PA_SHIFT) |				\
		(1 << MIPS_SEGCFG_EU_SHIFT)) << 16)
	ins	t0, t1, 16, 3
	mtc0	t0, CP0_PAGEMASK, 3

	/* SegCtl2 */
	li	t0, ((MIPS_SEGCFG_MUSUK << MIPS_SEGCFG_AM_SHIFT) |	\
		(6 << MIPS_SEGCFG_PA_SHIFT) |				\
		(1 << MIPS_SEGCFG_EU_SHIFT)) |				\
		(((MIPS_SEGCFG_MUSUK << MIPS_SEGCFG_AM_SHIFT) |		\
		(4 << MIPS_SEGCFG_PA_SHIFT) |				\
		(1 << MIPS_SEGCFG_EU_SHIFT)) << 16)
	or	t0, t2
	mtc0	t0, CP0_PAGEMASK, 4

	jal	mips_ihb
	mfc0    t0, CP0_CONFIG, 5
	li      t2, MIPS_CONF5_K      /* K bit */
	or      t0, t0, t2
	mtc0    t0, CP0_CONFIG, 5

	sync
	jal	mips_ihb

	mfc0    t0, CP0_CONFIG, 0
	li      t2, MIPS32R5_CONF_MM      /* Write Merge */
	or      t0, t0, t2
	mtc0    t0, CP0_CONFIG, 0
	sync
	jal	mips_ihb
	nop

	.set	pop
	.endm

	/*
	 * Prepare segments for LEGACY boot:
	 *
	 * ========================= Mappings =============================
	 * Segment   Virtual    Size   Access Mode   Physical   Caching   EU
	 * -------   -------    ----   -----------   --------   -------   --
	 *    0      e0000000   512M      MK            UND         U       0
	 *    1      c0000000   512M      MSK           UND         U       0
	 *    2      a0000000   512M      UK            000         2       0
	 *    3      80000000   512M      UK            000         3       0
	 *    4      40000000    1G       MUSK          UND         U       1
	 *    5      00000000    1G       MUSK          UND         U       1
	 *
	 * The following code uses the t0, t1, t2 and ra registers without
	 * previously preserving them.
	 *
	 */
	.macro	platform_legacy_init

	.set	push
	.set	reorder
#if 0
	/*
	 * Get Config.K0 value and use it to program
	 * the segmentation registers
	 */
	mfc0    t1, CP0_CONFIG
	andi	t1, 0x7 /* CCA */
#endif
	/*
	 * Directly use cacheable, coherent, write-back,
	 * write-allocate, read misses request shared attribute
	 */
	li      t1, 0x5
	move	t2, t1
	ins	t2, t1, 16, 3
	/* SegCtl0 */
	li      t0, ((MIPS_SEGCFG_MK << MIPS_SEGCFG_AM_SHIFT) |		\
		(0 << MIPS_SEGCFG_PA_SHIFT)) |				\
		(((MIPS_SEGCFG_MSK << MIPS_SEGCFG_AM_SHIFT) |		\
		(0 << MIPS_SEGCFG_PA_SHIFT)) << 16)
	or	t0, t2
	mtc0	t0, CP0_PAGEMASK, 2

	/* SegCtl1 */
	li      t0, ((MIPS_SEGCFG_UK << MIPS_SEGCFG_AM_SHIFT) |	\
		(0 << MIPS_SEGCFG_PA_SHIFT) |				\
		(2 << MIPS_SEGCFG_C_SHIFT)) |				\
		(((MIPS_SEGCFG_UK << MIPS_SEGCFG_AM_SHIFT) |		\
		(0 << MIPS_SEGCFG_PA_SHIFT)) << 16)
	ins	t0, t1, 16, 3
	mtc0	t0, CP0_PAGEMASK, 3

	/* SegCtl2 */
	li	t0, ((MIPS_SEGCFG_MUSK << MIPS_SEGCFG_AM_SHIFT) |	\
		(6 << MIPS_SEGCFG_PA_SHIFT) |				\
		(1 << MIPS_SEGCFG_EU_SHIFT)) |				\
		(((MIPS_SEGCFG_MUSK << MIPS_SEGCFG_AM_SHIFT) |		\
		(4 << MIPS_SEGCFG_PA_SHIFT) |				\
		(1 << MIPS_SEGCFG_EU_SHIFT)) << 16)
	or	t0, t2
	mtc0	t0, CP0_PAGEMASK, 4

	jal	mips_ihb
	nop

	mfc0    t0, CP0_CONFIG, 5
	li      t2, MIPS_CONF5_K      /* K bit */
	or      t0, t0, t2
	mtc0    t0, CP0_CONFIG, 5
	sync
	jal	mips_ihb
	nop

	mfc0    t0, CP0_CONFIG, 0
	li      t2, MIPS32R5_CONF_MM      /* Write Merge */
	or      t0, t0, t2
	mtc0    t0, CP0_CONFIG, 0
	sync
	jal	mips_ihb
	nop

	.set	pop
	.endm

#ifndef CONFIG_MACH_BAIKAL_BFK2
	.macro	platform_errata_fix

	.set	push
	.set	reorder

	jal	mips_ihb
	nop

	/*
	 * Disable load/store bonding.
	 */
	mfc0    t0, CP0_CONFIG, 6
	lui     t1, (MIPS_CONF6_DLSB >> 16)
	or      t0, t0, t1
	/*
	 * This disables all JR prediction other than JR $31.
	 */
	ori     t0, t0, MIPS_CONF6_JRCD
	mtc0    t0, CP0_CONFIG, 6
	sync
	jal	mips_ihb
	nop

	/*
	 * This disables all JR $31 prediction through return prediction stack.
	 */
	mfc0    t0, CP0_CONFIG, 7
	ori     t0, t0, MIPS_CONF7_RPS
	mtc0    t0, CP0_CONFIG, 7
	sync
	jal	mips_ihb
	nop

	.set	pop
	.endm
#endif

	.macro	platform_enable_msa

	.set	push
	.set	reorder

#ifdef CONFIG_CPU_HAS_MSA
	jal	mips_ihb
	nop

	mfc0	t0, CP0_CONFIG, 5
	li	t1, MIPS_CONF5_MSAEN
	or	t0, t0, t1
	mtc0	t0, CP0_CONFIG, 5
	sync
	jal	mips_ihb
	nop

	mfc0	t0, CP0_STATUS, 0
	li	t1, ST0_FR
	or	t0, t0, t1
	mtc0	t0, CP0_STATUS, 0
	sync
	jal	mips_ihb
	nop
#endif /* CONFIG_CPU_HAS_MSA */

	.set	pop
	.endm

	.macro	kernel_entry_setup

	sync
	ehb

#ifdef CONFIG_EVA
	mfc0    t1, CP0_CONFIG
	bgez    t1, 9f
	mfc0	t0, CP0_CONFIG, 1
	bgez	t0, 9f
	mfc0	t0, CP0_CONFIG, 2
	bgez	t0, 9f
	mfc0	t0, CP0_CONFIG, 3
	sll     t0, t0, 6   /* SC bit */
	bgez    t0, 9f

	platform_eva_init

	b       0f
9:	b	9b
	nop
#else
	platform_legacy_init
#endif /* CONFIG_EVA */
0:
#ifndef CONFIG_MACH_BAIKAL_BFK2
	platform_errata_fix
#endif
	platform_enable_msa

	.endm

/*
 * Do SMP slave processor setup necessary before we can safely execute C code.
 */
	.macro	smp_slave_setup
	sync
	ehb

#ifdef CONFIG_EVA
	platform_eva_init
#else
	platform_legacy_init
#endif  /* CONFIG_EVA */

#ifndef CONFIG_MACH_BAIKAL_BFK2
	platform_errata_fix
#endif
	platform_enable_msa

	.endm

#endif /* __ASM_MACH_MIPS_KERNEL_ENTRY_INIT_H */
