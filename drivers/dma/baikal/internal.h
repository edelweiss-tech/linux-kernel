/*
 * Driver for the Synopsys DesignWare DMA Controller
 *
 * Copyright (C) 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _DMA_BAIKAL_INTERNAL_H
#define _DMA_BAIKAL_INTERNAL_H

#include <linux/dma/baikal.h>
#include <linux/dma/baikal-regs.h>

int baikal_dma_probe(struct baikal_dma_chip *chip, struct baikal_dma_platform_data *pdata);

int baikal_dma_remove(struct baikal_dma_chip *chip);

#endif /* _DMA_BAIKAL_INTERNAL_H */
