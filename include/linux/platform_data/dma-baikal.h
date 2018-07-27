/*
 * Driver for the Baikal PCIe eDMA Controller
 *
 * Copyright (C) 2007 Atmel Corporation
 * Copyright (C) 2010-2011 ST Microelectronics
 * Copyright (C) 2017-2018 Baikal Electronics JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _PLATFORM_DATA_DMA_BAIKAL_H
#define _PLATFORM_DATA_DMA_BAIKAL_H

/**
 * struct baikal_dma_platform_data - Controller configuration parameters
 * @wr_channels: Number of write channels supported by hardware (max 8)
 * @rd_channels: Number of read channels supported by hardware (max 8)
 */
struct baikal_dma_platform_data {
	unsigned int	wr_channels;
	unsigned int	rd_channels;
};

#endif /* _PLATFORM_DATA_DMA_BAIKAL_H */
