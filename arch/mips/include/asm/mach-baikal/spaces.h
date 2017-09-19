/*
 * Baikal-T SOC platform support code.
 *
 * Copyright (C) 2014  Baikal Electronics OJSC
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
#ifndef _ASM_BAIKAL_SPACES_H
#define _ASM_BAIKAL_SPACES_H

#include <asm/mach-baikal/hardware.h>

/*
 * Virtual addresses offset
 */
#define PAGE_OFFSET 		_AC(0x80000000, UL)
/*
 * Physical addresses offset
 */
#define PHYS_OFFSET 		_AC(0x00000000, UL)

/*
 * Uncached addresses offset
 */
#define UNCAC_BASE	_AC(0xa0000000, UL)	/* 0xa0000000 + PHYS_OFFSET */

/*
 * High memory segment physical addresses
 */
#define HIGHMEM_START		_AC(0x20000000, UL)
/*
 * I/O memory space
 */
#define IO_BASE		UNCAC_BASE

#include_next <spaces.h>

#endif /* __ASM_BAIKAL_SPACES_H */
