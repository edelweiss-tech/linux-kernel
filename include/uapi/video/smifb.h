/*
 * smifb.h - definitions for smifb framebuffer ioctls.
 */

#ifndef _UAPI_LINUX_SMIFB_H_
#define _UAPI_LINUX_SMIFB_H_

/* Compatibility to sm750 fbturbo */

typedef struct {
	void* from;
	loff_t fb_off;
	__u32 size;
} fb_dma_req_t;

#define FBIO_DW_GET_STAT_DMA_TRANSFER	_IOWR('F', 0xAA, __u32)
#define FBIO_DW_DMA_WRITE		_IOWR('F', 0xAB, fb_dma_req_t)

#endif
