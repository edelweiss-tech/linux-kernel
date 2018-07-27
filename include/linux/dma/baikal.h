/*
 * Driver for the Baikal SoC PCIe eDMA Controller
 *
 * Copyright (C) 2007 Atmel Corporation
 * Copyright (C) 2010-2011 ST Microelectronics
 * Copyright (C) 2014 Intel Corporation
 * Copyright (C) 2017-2018 Baikal Electronics JSC

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _DMA_BAIKAL_H
#define _DMA_BAIKAL_H

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma/baikal-regs.h>

/**
 * struct baikal_dma_chip - representation of Baikal SoC PCIe eDMA controller hardware
 * @dev:		struct device of the DMA controller
 * @irq:		irq lines
 * @regs:		memory mapped I/O space
 * @clk:		hclk clock
 * @dmac:		struct baikal_dma that is filed by be_dma_probe()
 */
struct baikal_dma_chip {
	struct device		*dev;
	struct resource		*irq[BAIKAL_EDMA_TOTAL_CHANNELS];
	void __iomem		*regs;
	struct clk			*clk;
	struct baikal_dma	*dmac;
	struct device_node  *of_upstream;
};

/* Essential API */

/* Helper function to request a write channel via dma_request_channel() */
bool baikal_dma_wr_chan_filter(struct dma_chan *chan, void *param);
/* One may also use a capital-letter alias for a code to look more elegant */
#define BAIKAL_EDMA_GET_WR_CHANNEL baikal_dma_wr_chan_filter

/* Helper function to request a read channel via dma_request_channel() */
bool baikal_dma_rd_chan_filter(struct dma_chan *chan, void *param);
/* One may also use a capital-letter alias for a code to look more elegant */
#define BAIKAL_EDMA_GET_RD_CHANNEL baikal_dma_rd_chan_filter

/* Prepare a channel for a single block transfer */
baikal_dmareturn_t baikal_dma_transfer_prepare_single(struct baikal_dma_chan *edma_chan, phyaddr src, phyaddr dst, uint32_t size);

/* Starts a prepared DMA transfer on a given channel immediately */
void baikal_dma_transfer_start(struct baikal_dma_chan *edma_chan);

/* Immediately stops any ongoing DMA transfer on a given channel */
void baikal_dma_transfer_stop(struct baikal_dma_chan *edma_chan);

/* Get status for a given channel */
baikal_dma_chan_status baikal_dma_chan_get_status(struct baikal_dma_chan *edma_chan);

/* Set user callback for a given channel */
void baikal_dma_chan_set_callback(struct baikal_dma_chan *edma_chan, void (*callback) (void *));

/* Disable user callback for a given channel */
void baikal_dma_chan_reset_callback(struct baikal_dma_chan *edma_chan);

/* Descriptor related functions */

struct baikal_dma_desc *baikal_dma_desc_produce(phyaddr src, phyaddr dst, uint32_t size);

baikal_dmareturn_t baikal_dma_desc_submit(struct baikal_dma_chan *edma_chan, struct baikal_dma_desc *desc);

/* Enable and disable functions. Use with caution */

int baikal_dma_disable(struct baikal_dma_chip *chip);

int baikal_dma_enable(struct baikal_dma_chip *chip);

/* Miscellaneous low level helpers for use in callback functions */

bool baikal_dma_query_irq_src(struct baikal_dma *dmac, baikal_dma_dirc *dirc, uint8_t *chan, baikal_dma_irq_type *type);

bool baikal_dma_query_err_src(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan, baikal_dma_err *type);

#endif /* _DMA_BAIKAL_H */
