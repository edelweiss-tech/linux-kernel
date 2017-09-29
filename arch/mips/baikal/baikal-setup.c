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
 */

#include <linux/init.h>
#include <linux/pci.h>

#include <asm/mips-cm.h>            /* Coherence manager */
#include <asm/traps.h>
#include <asm/mach-baikal/hardware.h>
#include <asm/mips-boards/generic.h>

#include "common.h"              /* Common Baikal definitions */

static void __init plat_setup_iocoherency(void)
{
	if (mips_cm_numiocu() != 0) {
		pr_info("CMP IOCU detected\n");

	if (coherentio)
		pr_info("Hardware DMA cache coherency enabled\n");
	else
		pr_info("Hardware DMA cache coherency disabled\n");
	}
}

static int __init baikal_platform_setup(void)
{
#ifdef CONFIG_PCI
	/* PCI init */
	mips_pcibios_init();
#endif
	/* Setup IO Coherency */
	plat_setup_iocoherency();
	/* No critical actions - always return success */
	return 0;
}
late_initcall(baikal_platform_setup);

void baikal_be_init(void)
{
	/* Could change CM error mask register. */
}

static char *tr[8] = {
	"mem",   "gcr",   "gic",   "mmio",
	"0x04", "0x05", "0x06", "0x07"
};

static char *mcmd[32] = {
	[0x00] = "0x00",
	[0x01] = "Legacy Write",
	[0x02] = "Legacy Read",
	[0x03] = "0x03",
	[0x04] = "0x04",
	[0x05] = "0x05",
	[0x06] = "0x06",
	[0x07] = "0x07",
	[0x08] = "Coherent Read Own",
	[0x09] = "Coherent Read Share",
	[0x0a] = "Coherent Read Discard",
	[0x0b] = "Coherent Ready Share Always",
	[0x0c] = "Coherent Upgrade",
	[0x0d] = "Coherent Writeback",
	[0x0e] = "0x0e",
	[0x0f] = "0x0f",
	[0x10] = "Coherent Copyback",
	[0x11] = "Coherent Copyback Invalidate",
	[0x12] = "Coherent Invalidate",
	[0x13] = "Coherent Write Invalidate",
	[0x14] = "Coherent Completion Sync",
	[0x15] = "0x15",
	[0x16] = "0x16",
	[0x17] = "0x17",
	[0x18] = "0x18",
	[0x19] = "0x19",
	[0x1a] = "0x1a",
	[0x1b] = "0x1b",
	[0x1c] = "0x1c",
	[0x1d] = "0x1d",
	[0x1e] = "0x1e",
	[0x1f] = "0x1f"
};

static char *core[8] = {
	"Invalid/OK",  "Invalid/Data",
	"Shared/OK",   "Shared/Data",
	"Modified/OK", "Modified/Data",
	"Exclusive/OK", "Exclusive/Data"
};

static char *causes[32] = {
	"None", "GC_WR_ERR", "GC_RD_ERR", "COH_WR_ERR",
	"COH_RD_ERR", "MMIO_WR_ERR", "MMIO_RD_ERR", "0x07",
	"0x08", "0x09", "0x0a", "0x0b",
	"0x0c", "0x0d", "0x0e", "0x0f",
	"0x10", "0x11", "0x12", "0x13",
	"0x14", "0x15", "0x16", "INTVN_WR_ERR",
	"INTVN_RD_ERR", "0x19", "0x1a", "0x1b",
	"0x1c", "0x1d", "0x1e", "0x1f"
};

int baikal_be_handler(struct pt_regs *regs, int is_fixup)
{
	/* This duplicates the handling in do_be which seems wrong */
	int retval = is_fixup ? MIPS_BE_FIXUP : MIPS_BE_FATAL;

	if (mips_cm_present()) {
		unsigned long cm_error = read_gcr_error_cause();
		unsigned long cm_addr = read_gcr_error_addr();
		unsigned long cm_other = read_gcr_error_mult();
		unsigned long cause, ocause;
		char buf[256];

		cause = cm_error & CM_GCR_ERROR_CAUSE_ERRTYPE_MSK;
		if (cause != 0) {
			cause >>= CM_GCR_ERROR_CAUSE_ERRTYPE_SHF;
			if (cause < 16) {
				unsigned long cca_bits = (cm_error >> 15) & 7;
				unsigned long tr_bits = (cm_error >> 12) & 7;
				unsigned long cmd_bits = (cm_error >> 7) & 0x1f;
				unsigned long stag_bits = (cm_error >> 3) & 15;
				unsigned long sport_bits = (cm_error >> 0) & 7;

			snprintf(buf, sizeof(buf),
				 "CCA=%lu TR=%s MCmd=%s STag=%lu SPort=%lu\n",
				 cca_bits, tr[tr_bits], mcmd[cmd_bits],
				 stag_bits, sport_bits);
			} else {
				/* glob state & sresp together */
				unsigned long c3_bits = (cm_error >> 18) & 7;
				unsigned long c2_bits = (cm_error >> 15) & 7;
				unsigned long c1_bits = (cm_error >> 12) & 7;
				unsigned long c0_bits = (cm_error >> 9) & 7;
				unsigned long sc_bit = (cm_error >> 8) & 1;
				unsigned long cmd_bits = (cm_error >> 3) & 0x1f;
				unsigned long sport_bits = (cm_error >> 0) & 7;

				snprintf(buf, sizeof(buf),
					"C3=%s C2=%s C1=%s C0=%s SC=%s MCmd=%s SPort=%lu\n",
					core[c3_bits], core[c2_bits],
					core[c1_bits], core[c0_bits],
					sc_bit ? "True" : "False",
					mcmd[cmd_bits], sport_bits);
			}

			ocause = (cm_other & CM_GCR_ERROR_MULT_ERR2ND_MSK) >>
			CM_GCR_ERROR_MULT_ERR2ND_SHF;

			pr_err("CM_ERROR=%08lx %s <%s>\n", cm_error,
				causes[cause], buf);
			pr_err("CM_ADDR =%08lx\n", cm_addr);
			pr_err("CM_OTHER=%08lx %s\n", cm_other, causes[ocause]);

			/* reprime cause register */
			write_gcr_error_cause(0);
		}
	}

	return retval;
}
