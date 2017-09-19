/*
 * Baikal-T SOC platform support code.
 *
 * Copyright (C) 2014,2015  Baikal Electronics OJSC
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

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H
/*
 * 26.08.2013 Dmitry Duanev
 * In terms of address constants the physical addresses has
 * suffix _START. All virtual addresses has suffix _BASE
 * The constant prefix specifies the target CPU (f.ex BAIKAL_)
 *
 * 27.01.2014 Dmitry Dunaev
 * Memory map is adopted to the MIPS architecture
 */

#include <linux/sizes.h>

/* Global IO addresses */
#define BAIKAL_IO_START			(0x1F000000)
#define BAIKAL_IO_SIZE			SZ_16M
/* PCI mapping region */
#define BAIKAL_PCI_MAP_START	(0x08000000)
#define BAIKAL_PCI_MAP_SIZE		SZ_256M

/* Physical allocation of subsystems */
#define BAIKAL_BOOT_START		(0x1FC00000)
#define BAIKAL_BOOT_SIZE		SZ_4M
#define BAIKAL_SRAM_START		(0x1BF80000)
#define BAIKAL_SRAM_SIZE		SZ_64K
#define BAIKAL_ROM_START		(0x1BFC0000)
#define BAIKAL_ROM_SIZE			SZ_64K
#define BAIKAL_DRAM_START		(0x00000000)
#define BAIKAL_DRAM_SIZE		SZ_128M
#define BAIKAL_HIGHMEM_START	(0x20000000)
#define BAIKAL_HIGHMEM_SIZE		SZ_1G

/* Peripheral addresses, offset from BAIKAL_IO_START */
#define BAIKAL_P5600			(BAIKAL_IO_START + 0x00000000)
#define BAIKAL_BOOT_CTRL_START	(BAIKAL_IO_START + 0x00040000)
#define BAIKAL_BOOT_CTRL_CSR	(BAIKAL_BOOT_CTRL_START + 0x00)
#define BAIKAL_BOOT_CTRL_MAR	(BAIKAL_BOOT_CTRL_START + 0x04)
#define BAIKAL_BOOT_CTRL_DRID	(BAIKAL_BOOT_CTRL_START + 0x08)
#define BAIKAL_BOOT_CTRL_VID	(BAIKAL_BOOT_CTRL_START + 0x0C)
#define BAIKAL_DMA_START		(BAIKAL_IO_START + 0x00041000)
#define BAIKAL_DDR_START		(BAIKAL_IO_START + 0x00042000)
#define BAIKAL_DDR_PHY			(BAIKAL_IO_START + 0x00043000)
#define BAIKAL_GPIO_START		(BAIKAL_IO_START + 0x00044000)
#define BAIKAL_CTRL_GPIO_START	(BAIKAL_IO_START + 0x00045000)
#define BAIKAL_I2C_START		(BAIKAL_IO_START + 0x00046000)
#define BAIKAL_SPI_START		(BAIKAL_IO_START + 0x00047000)
#define BAIKAL_RTC_START		(BAIKAL_IO_START + 0x00048000)
#define BAIKAL_TIMERS_START		(BAIKAL_IO_START + 0x00049000)
#define BAIKAL_UART0_START		(BAIKAL_IO_START + 0x0004A000)
#define BAIKAL_UART1_START		(BAIKAL_IO_START + 0x0004B000)
#define BAIKAL_WDT_START		(BAIKAL_IO_START + 0x0004C000)
#define BAIKAL_PMU_START		(BAIKAL_IO_START + 0x0004D000)
#define BAIKAL_PMU_I2C_START	(BAIKAL_IO_START + 0x0004D800)
#define BAIKAL_GMAC_START		(BAIKAL_IO_START + 0x0004E000)
#define BAIKAL_GMAC_DMA			(BAIKAL_IO_START + 0x0004F000)
#define BAIKAL_SATA_START		(BAIKAL_IO_START + 0x00050000)
#define BAIKAL_PCI_START		(BAIKAL_IO_START + 0x00051000)
#define BAIKAL_PCI_DMA			(BAIKAL_IO_START + 0x00052000)
#define BAIKAL_USB_START		(BAIKAL_IO_START + 0x00053000)
#define BAIKAL_USB_DMA			(BAIKAL_IO_START + 0x00054000)
#define BAIKAL_XGMAC_START		(BAIKAL_IO_START + 0x00055000)
#define BAIKAL_XGMAC_DMA		(BAIKAL_IO_START + 0x00056000)
#define BAIKAL_VIRTUAL_BLOCK	(BAIKAL_IO_START + 0x000FF000)
#define BAIKAL_VBLOCK_EXIT		(BAIKAL_VIRTUAL_BLOCK + 0x00)
#define BAIKAL_VBLOCK_REVISION	(BAIKAL_VIRTUAL_BLOCK + 0x04)

#endif /* __ASM_ARCH_HARDWARE_H */
