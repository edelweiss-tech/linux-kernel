/*
 *  Baikal-T SOC platform support code.
 *
 *  Copyright (C) 2015,2016 Baikal Electronics JSC.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 *  BAIKAL MIPS boards specific PCI support.
 */

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>

#include "pci-baikal.h"

#define IDT_PES32NT8AG2_ID		0x808F111D
#define IDT_PES32NT8BG2_ID		0x8088111D
#define IDT_VID				0x111D


/*
 * We can't address 8 and 16 bit words directly.  Instead we have to
 * read/write a 32bit word and mask/modify the data we actually want.
 */
static int dw_pcibios_read(struct pci_bus *bus, unsigned int devfn,
			     int where, int size, u32 *val)
{
	volatile u32 data = 0;
	volatile u8 *addr = 0;
	u16 target;

    if ((bus->number == PCIE_ROOT_BUS_NUM) && (PCI_SLOT(devfn) != 0)) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if ((size == 2) && (where & 1)) {
		*val = 0xffffffff;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	} else if ((size == 4) && (where & 3)) {
		*val = 0xffffffff;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	target = ((bus->number << 8) | devfn);

	if ((!bus->parent) || (bus->parent->number == bus->number)) {
		/* dw_set_iatu_region(dir, index, base_addr, limit_addr, target_addr, tlp_type) */
		dw_set_iatu_region(REGION_DIR_OUTBOUND, IATU_RD0_INDEX, PHYS_PCI_RD0_BASE_ADDR >> 16,
				PHYS_PCI_RD0_LIMIT_ADDR >> 16, target, TLP_TYPE_CFGRD0);
		addr = (u8 *)PCI_RD0_BASE_ADDR;
	} else {
		dw_set_iatu_region(REGION_DIR_OUTBOUND, IATU_RD1_INDEX, PHYS_PCI_RD1_BASE_ADDR >> 16,
				PHYS_PCI_RD1_LIMIT_ADDR >> 16, target, TLP_TYPE_CFGRD1);
		addr = (u8 *)PCI_RD1_BASE_ADDR;
	}

	wmb();

	addr += (where & ~0x3);
	data = readl(addr);

	if (size == 1)
		*val = (data >> ((where & 3) << 3)) & 0xff;
	else if (size == 2)
		*val = (data >> ((where & 3) << 3)) & 0xffff;
	else
		*val = data;

	dev_dbg(&bus->dev, "bus=%d devfn=0x%x where=0x%x addr=0x%x val=0x%x\n",
		bus->number, devfn, where, (unsigned int)addr, (unsigned int)*val);

	return PCIBIOS_SUCCESSFUL;
}

static int dw_pcibios_write(struct pci_bus *bus, unsigned int devfn,
			      int where, int size, u32 val)
{
	volatile u32 data = 0;
	volatile u8 *addr = 0;
	u16 target;

    if ((bus->number == PCIE_ROOT_BUS_NUM) && (PCI_SLOT(devfn) != 0)) {
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if ((size == 2) && (where & 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	else if ((size == 4) && (where & 3))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	target = ((bus->number << 8) | devfn);

	if ((!bus->parent) || (bus->parent->number == bus->number)) {
		/* dw_set_iatu_region(dir, index, base_addr, limit_addr, target_addr, tlp_type) */
		dw_set_iatu_region(REGION_DIR_OUTBOUND, IATU_RD0_INDEX, PHYS_PCI_RD0_BASE_ADDR >> 16,
				PHYS_PCI_RD0_LIMIT_ADDR >> 16, target, TLP_TYPE_CFGRD0);
		addr = (u8 *)PCI_RD0_BASE_ADDR;
	} else {
		dw_set_iatu_region(REGION_DIR_OUTBOUND, IATU_RD1_INDEX, PHYS_PCI_RD1_BASE_ADDR >> 16,
				PHYS_PCI_RD1_LIMIT_ADDR >> 16, target, TLP_TYPE_CFGRD1);
		addr = (u8 *)PCI_RD1_BASE_ADDR;
	}

	wmb();

	addr += (where & ~0x3);

	if (size == 4)
		data = val;
	else {
		data = readl(addr);

		if (size == 1)
			data = (data & ~(0xff << ((where & 3) << 3))) |
				(val << ((where & 3) << 3));
		else if (size == 2)
			data = (data & ~(0xffff << ((where & 3) << 3))) |
				(val << ((where & 3) << 3));
	}

	writel(data, addr);

	dev_dbg(&bus->dev, "bus=%d devfn=0x%x where=0x%x addr = 0x%x data=0x%x\n",
		bus->number, devfn, where, (unsigned int)addr, data);

	return PCIBIOS_SUCCESSFUL;
}

struct pci_ops dw_pci_ops = {
	.read = dw_pcibios_read,
	.write = dw_pcibios_write
};
