/*
 * Baikal-T SOC platform support code.
 *
 * Copyright (C) 2014-2016  Baikal Electronics OJSC
 * 
 * Author:
 *   Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>
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

#ifndef __ASM_MIPS_BOARDS_BAIKAL_H
#define __ASM_MIPS_BOARDS_BAIKAL_H

#include <asm/addrspace.h>
#include <asm/io.h>

/*
 * GCMP Specific definitions
 */
#define GCMP_BASE_ADDR			0x1fbf8000
#define GCMP_ADDRSPACE_SZ		(256 * 1024)

/*
 * GIC Specific definitions
 */
#define GIC_BASE_ADDR			0x1bdc0000
#define GIC_ADDRSPACE_SZ		(128 * 1024)

/*
 * CPC Specific definitions
 */
#define CPC_BASE_ADDR			0x1bde0000
#define CPC_ADDRSPACE_SZ		(24 * 1024)


#endif /* __ASM_MIPS_BOARDS_BAIKAL_H */
