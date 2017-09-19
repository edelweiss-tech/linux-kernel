/*
 * Baikal-T SOC platform support code. EFUSE driver.
 *
 * Brief Baikal T1 EFUSE registers and fields declarations
 * based on Baikal-T1 EFUSE_programming_v0.4.pdf
 *
 * Copyright (C) 2014-2016 Baikal Electronics JSC
 * 
 * Author:
 *   Georgiy Vlasov <Georgy.Vlasov@baikalelectronics.ru>
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

#ifndef __EFUSE_H_
#define __EFUSE_H_

#define Gb_ETHERNET_0 0
#define Gb_ETHERNET_1 1
#define xGb_ETHERNET  2

/* EFUSE registers */
/* MODE_REG. R/W. */
#define EFUSE_MODES        0x0004
#define EFUSE_MODES_RegisterSize 32
#define EFUSE_MODES_RegisterResetValue 0x0
#define EFUSE_MODES_RegisterResetMask 0xffffffff
/* Register Field information for EFUSE_MODES */
#define EFUSE_MODES_MODE_BitAddressOffset 0
#define EFUSE_MODES_MODE_RegisterSize 2
/* Bits 31:2 - reserved */

/* ADDR_REG EFUSE. R/W. */
#define EFUSE_ADDR         0x0008
#define EFUSE_ADDR_RegisterSize 32
#define EFUSE_ADDR_RegisterResetValue 0x0
#define EFUSE_ADDR_RegisterResetMask 0xffffffff
/* Register Field information for EFUSE_ADDR */
#define EFUSE_ADDR_Addr_BitAddressOffset 0   /* [4:0] - номер строки EFUSE памяти. */
#define EFUSE_ADDR_Addr_RegisterSize 5
/* Bits 31:5 - reserved */

/* ENABLE EFUSE [0]. R/W. */
#define EFUSE_ENABLE       0x000C
#define EFUSE_ENABLE_RegisterSize 32
#define EFUSE_ENABLE_RegisterResetValue 0x0
#define EFUSE_ENABLE_RegisterResetMask 0xffffffff
/* Register Field information for EFUSE_ADDR */
#define EFUSE_ENABLE_Enable_BitAddressOffset 0   /* [4:0] - номер строки EFUSE памяти. */
#define EFUSE_ENABLE_Enable_RegisterSize 1
/* Bits 31:1 - reserved */

/* Reg for readind data. RO. */
#define EFUSE_RDATA        0x0010
#define EFUSE_RDATA_RegisterSize 32
#define EFUSE_RDATA_RegisterResetValue 0x0
#define EFUSE_RDATA_RegisterResetMask 0xffffffff

/* API Functions */
u32 be_efuse_getLocks(void);
u8 	be_efuse_getVersion(void);
u8 	be_efuse_getFab(void);
u8 	be_efuse_getProcess(void);
u8 	be_efuse_getLotID(void);
u8 	be_efuse_getRevision(void);
u32 be_efuse_getSerialNum(void);
u32 be_efuse_getCornerID(void);
u32 be_efuse_getCPUFreq(void);
u32 be_efuse_getPad(void);
u64 be_efuse_getMAC(u8 id);

#endif
