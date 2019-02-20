/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2006  Ralf Baechle <ralf@linux-mips.org>
 *
 */
#ifndef __ASM_MACH_BAIKAL_DMA_COHERENCE_H
#define __ASM_MACH_BAIKAL_DMA_COHERENCE_H

#ifdef CONFIG_PCI
extern struct bus_type pci_bus_type;
#endif

struct device;

static inline dma_addr_t plat_map_dma_mem(struct device *dev, void *addr,
	size_t size)
{
	return virt_to_phys(addr);
}

static inline dma_addr_t plat_map_dma_mem_page(struct device *dev,
	struct page *page)
{
	return page_to_phys(page);
}

static inline unsigned long plat_dma_addr_to_phys(struct device *dev,
	dma_addr_t dma_addr)
{
	return dma_addr;
}

static inline void plat_unmap_dma_mem(struct device *dev, dma_addr_t dma_addr,
	size_t size, enum dma_data_direction direction)
{
}

static inline int plat_dma_supported(struct device *dev, u64 mask)
{
	/*
	 * we fall back to GFP_DMA when the mask isn't all 1s,
	 * so we can't guarantee allocations that must be
	 * within a tighter range than GFP_DMA..
	 */
#ifdef CONFIG_PCI
	if (dev->bus == &pci_bus_type)
		return 1;	/* assume PCI device can access > 4GB */
#endif
	if (mask < DMA_BIT_MASK(24) || mask > DMA_BIT_MASK(32))
		return 0;

	return 1;
}

static inline int plat_device_is_coherent(struct device *dev)
{
#ifdef CONFIG_DMA_PERDEV_COHERENT
	return dev->archdata.dma_coherent;
#else
	switch (coherentio) {
	default:
	case IO_COHERENCE_DEFAULT:
		return hw_coherentio;
	case IO_COHERENCE_ENABLED:
		return 1;
	case IO_COHERENCE_DISABLED:
		return 0;
	}
#endif
}

static inline void plat_post_dma_flush(struct device *dev)
{
}

#endif /* __ASM_MACH_BAIKAL_DMA_COHERENCE_H */
