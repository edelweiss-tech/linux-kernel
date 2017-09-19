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
#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/serial_8250.h>

#include <asm/io.h>
#include <asm/mach-baikal/hardware.h>

#define DW_UART_THR		0x00
#define DW_UART_DLL		0x00
#define DW_UART_FCR		0x08
#define DW_UART_LCR		0x0C
#define DW_UART_LSR		0x14

#define DW_UART_LSR_TEMT 	(1 << 6)
#define DW_UART_LSR_THRE	(1 << 5)

#define EARLY_CONSOLE_BASE BAIKAL_UART0_START

static __iomem void *uart_membase = (__iomem void *) KSEG1ADDR(EARLY_CONSOLE_BASE);

static inline void uart_write32(u32 val, unsigned reg)
{
	writel(val, uart_membase + reg);
}

static inline u32 uart_read32(unsigned reg)
{
	return readl(uart_membase + reg);
}

void prom_putchar(unsigned char ch)
{
	while ((uart_read32(DW_UART_LSR) & DW_UART_LSR_TEMT) == 0)
		;
	uart_write32(ch, DW_UART_THR);
}
