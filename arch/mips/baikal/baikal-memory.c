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
#include <linux/bootmem.h>
#include <linux/of_fdt.h>
#include <linux/memblock.h>

#include <asm/bootinfo.h>
#include <asm/prom.h>
#include <asm/maar.h>

#include <asm/mach-baikal/hardware.h>
#include "common.h"

#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <asm/current.h>
#include <asm/sections.h>

#define BAIKAL_MMIO_MEM_START		0x08000000
#define BAIKAL_MMIO_MEM_END		0x1FFFFFFF

void __init prom_free_prom_memory(void)
{
	/* Nothing todo here */
}

/*
 * Platform memory detection hook called by setup_arch
 * extern void plat_mem_setup(void);
 */
void __init plat_mem_setup(void)
{
	/* Setup dummy port segment */
	set_io_port_base(CKSEG1);
	if (config_enabled(CONFIG_EVA))
		/* EVA should be configured in mach-baikal/kernel-init.h */
		pr_info("Enhanced Virtual Addressing (EVA) activated\n");

#ifdef CONFIG_OF
	/* Try to parse device tree */
	if (!device_tree_early_init())
		return;
#endif
		/* Low memory region */
	add_memory_region(BAIKAL_DRAM_START, BAIKAL_DRAM_SIZE, BOOT_MEM_RAM);
#ifdef CONFIG_HIGHMEM
		/* High memory region */
	add_memory_region(BAIKAL_HIGHMEM_START, BAIKAL_HIGHMEM_SIZE, BOOT_MEM_RAM);
#endif
}

#ifdef CONFIG_CPU_SUPPORTS_UNCACHED_ACCELERATED
/* override of arch/mips/mm/cache.c: __uncached_access */
int __uncached_access(struct file *file, unsigned long addr)
{
	if (file->f_flags & O_DSYNC)
		return 1;

	return addr >= __pa(high_memory) ||
		((addr >= BAIKAL_MMIO_MEM_START) &&
		 (addr < BAIKAL_MMIO_MEM_END));
}

static unsigned long uca_start, uca_end;

pgprot_t phys_mem_access_prot(struct file *file, unsigned long pfn,
			      unsigned long size, pgprot_t vma_prot)
{
	unsigned long offset = pfn << PAGE_SHIFT;
	unsigned long end = offset + size;

	if (__uncached_access(file, offset)) {
		if (uca_start && (offset >= uca_start) &&
		    (end <= uca_end))
			return __pgprot((pgprot_val(vma_prot) &
					 ~_CACHE_MASK) |
					_CACHE_UNCACHED_ACCELERATED);
		else
			return pgprot_noncached(vma_prot);
	}
	return vma_prot;
}

int /* __init*/ baikal_find_vga_mem_init(void)
{
	struct pci_dev *dev = 0;
	struct resource *r;
	int idx;

	if (uca_start)
		return 0;

	for_each_pci_dev(dev) {
		if ((dev->class >> 16) == PCI_BASE_CLASS_DISPLAY) {
			for (idx = 0; idx < PCI_NUM_RESOURCES; idx++) {
				r = &dev->resource[idx];
				if (!r->start && r->end)
					continue;
				if (r->flags & IORESOURCE_IO)
					continue;
				if (r->flags & IORESOURCE_MEM) {
					uca_start = r->start;
					uca_end = r->end;
					return 0;
				}
			}
		}
	}

	return 0;
}
late_initcall(baikal_find_vga_mem_init);
#endif /* !CONFIG_CPU_SUPPORTS_UNCACHED_ACCELERATED */

/*
 * Platform-specific method of MAAR registers initialization
 */
unsigned platform_maar_init(unsigned num_pairs)
{
	struct maar_config cfg[BOOT_MEM_MAP_MAX];
	unsigned i, num_configured, num_cfg = 0;

	/* Collect RAM regions within MAAR config array */
	for (i = 0; i < boot_mem_map.nr_map; i++) {
		switch (boot_mem_map.map[i].type) {
		case BOOT_MEM_RAM:
		case BOOT_MEM_INIT_RAM:
			break;
		default:
			continue;
		}

		/* Avoid of low memory mapping */
		if (boot_mem_map.map[i].addr < (BAIKAL_DRAM_START + BAIKAL_DRAM_SIZE)) {
			cfg[num_cfg].upper = ((ulong)virt_to_phys(_end) & ~0xffff) - 1;
			cfg[num_cfg].lower = ((ulong)virt_to_phys(_text) + 0xffff) & ~0xffff;
		} else {
			cfg[num_cfg].upper = boot_mem_map.map[i].addr +
						boot_mem_map.map[i].size;
			cfg[num_cfg].upper = (cfg[num_cfg].upper & ~0xffff) - 1;
			cfg[num_cfg].lower = (boot_mem_map.map[i].addr + 0xffff) & ~0xffff;
		}
		cfg[num_cfg].attrs = MIPS_MAAR_S;
		num_cfg++;
	}

	num_configured = maar_config(cfg, num_cfg, num_pairs);
	if (num_configured < num_cfg)
		pr_warn("Not enough MAAR pairs (%u) for all bootmem regions (%u)\n",
			num_pairs, num_cfg);

	return num_configured;
}
