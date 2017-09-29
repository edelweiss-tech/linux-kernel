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

#ifndef __BAIKAL_COMMON_H
#define __BAIKAL_COMMON_H

#define CPU_FREQ		600000000
#define CPU_CLK_DIV		2

extern __iomem void *plat_of_remap_node(const char *node);
extern int device_tree_early_init(void);

#ifdef CONFIG_KEXEC
#include <asm/kexec.h>
extern int baikal_kexec_prepare(struct kimage *kimage);
extern void baikal_kexec_shutdown(void);
#endif

extern void baikal_be_init(void);
extern int baikal_be_handler(struct pt_regs *regs, int is_fixup);

extern char except_vec_nmi;
extern char except_vec_ejtag_debug;
#endif /* __BAIKAL_COMMON_H */
