/*
 * Core driver for the Baikal SoC PCIe eDMA Controller
 *
 * Copyright (C) 2007-2008 Atmel Corporation
 * Copyright (C) 2010-2011 ST Microelectronics
 * Copyright (C) 2013 Intel Corporation
 * Copyright (C) 2017-2018 Baikal Electronics JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

#include <linux/platform_data/dma-baikal.h>
#include "internal.h"

static char *baikal_dma_err_info[DMA_ERR_CODE_NUM];
uint32_t i, new_val;

static struct device *chan2dev(struct dma_chan *chan)
{
	return &chan->dev->device;
}

static struct device *chan2parent(struct dma_chan *chan)
{
	return chan->dev->device.parent;
}

static inline void baikal_dma_chan_setup(struct baikal_dma_chan *edma_chan, uint32_t size, int nollp)
{
	edma_chan->xfer_size = size;
	edma_chan->feed_size = 0;
	edma_chan->nollp = nollp;
}

static inline void baikal_dma_desc_fill(struct baikal_dma_desc *desc, phyaddr src, phyaddr dst, uint32_t size) 
{
	if (desc) {
		desc->lli.dar.low_part = dst.low_part;
		desc->lli.dar.high_part = 0x0;
		desc->lli.sar.low_part = src.low_part;
		desc->lli.sar.high_part = 0x0;
		desc->lli.xfer_size = size;
	}
}

struct baikal_dma_desc *baikal_dma_desc_produce(phyaddr src, phyaddr dst, uint32_t size) 
{
	struct baikal_dma_desc *desc;
	desc = kzalloc(sizeof(struct baikal_dma_desc), GFP_KERNEL);
	baikal_dma_desc_fill(desc, src, dst, size);
	return desc;
}

static struct baikal_dma_desc *baikal_dma_chan_desc_get(struct baikal_dma_chan *edma_chan)
{
	struct baikal_dma_desc *desc, *_desc;
	struct baikal_dma_desc *ret = NULL;
	unsigned int i = 0;
	unsigned long flags;

	spin_lock_irqsave(&edma_chan->lock, flags);
	list_for_each_entry_safe(desc, _desc, &edma_chan->free_list, desc_node) {
		if (async_tx_test_ack(&desc->txd)) {
			list_del(&desc->desc_node);
			ret = desc;
			break;
		}
		dev_dbg(chan2dev(&edma_chan->chan), "desc %p not ACKed\n", desc);
		i++;
	}
	spin_unlock_irqrestore(&edma_chan->lock, flags);

	dev_vdbg(chan2dev(&edma_chan->chan), "scanned %u descriptors on freelist\n", i);

	return ret;
}

static void baikal_dma_chan_sync_desc_for_cpu(struct baikal_dma_chan *edma_chan, struct baikal_dma_desc *desc)
{
	struct baikal_dma_desc	*child;

	list_for_each_entry(child, &desc->tx_list, desc_node)
		dma_sync_single_for_cpu(chan2parent(&edma_chan->chan),
				child->txd.phys, sizeof(child->lli),
				DMA_TO_DEVICE);
	dma_sync_single_for_cpu(chan2parent(&edma_chan->chan),
			desc->txd.phys, sizeof(desc->lli),
			DMA_TO_DEVICE);
}

/*
 * Move a descriptor, including any children, to the free list.
 * `desc' must not be on any lists.
 */
static void baikal_dma_chan_desc_put(struct baikal_dma_chan *edma_chan, struct baikal_dma_desc *desc)
{
	unsigned long flags;

	if (desc) {
		struct baikal_dma_desc *child;

		baikal_dma_chan_sync_desc_for_cpu(edma_chan, desc);

		spin_lock_irqsave(&edma_chan->lock, flags);
		list_for_each_entry(child, &desc->tx_list, desc_node)
			dev_vdbg(chan2dev(&edma_chan->chan),
					"moving child desc %p to freelist\n",
					child);
		list_splice_init(&desc->tx_list, &edma_chan->free_list);
		dev_vdbg(chan2dev(&edma_chan->chan), "moving desc %p to freelist\n", desc);
		list_add(&desc->desc_node, &edma_chan->free_list);
		spin_unlock_irqrestore(&edma_chan->lock, flags);
	}
}

/* Debugging functions */
inline void baikal_dma_chan_dump_regs(struct baikal_dma *dmac, struct baikal_dma_chan *edma_chan)
{
	pr_err("SAR_lo: %x SAR_hi: %x DAR_lo: %x DAR_hi: %x xfer_size = %d CTL_lo: %x CTL_hi: %x\n",
			((unsigned int)channel_readl(dmac, edma_chan->id, sar_ptr_lo)),
			((unsigned int)channel_readl(dmac, edma_chan->id, sar_ptr_hi)),
			((unsigned int)channel_readl(dmac, edma_chan->id, dar_ptr_lo)),
			((unsigned int)channel_readl(dmac, edma_chan->id, dar_ptr_hi)),
			((unsigned int)channel_readl(dmac, edma_chan->id, xfer_size)),
			((unsigned int)channel_readl(dmac, edma_chan->id, ctrl_lo.dword)),
			((unsigned int)channel_readl(dmac, edma_chan->id, ctrl_hi.dword)));
	pr_err("WR_Enable: %d WR_IRQ_Mask: %x CTX: %x WR_Doorbell: %x CTRL: %x\n",
			dma_readl(dmac, write_enb.dword),
			dma_readl(dmac, write_irq_mask),
			dma_readl(dmac, ctx_sel.dword),
			dma_readl(dmac, write_db.dword),
			dma_readl(dmac, dma_ctrl.dword));
	pr_err("RD_Enable: %d RD_IRQ_Mask: %x RD_Doorbell: %x\n",
			dma_readl(dmac, read_enb.dword),
			dma_readl(dmac, read_irq_mask),
			dma_readl(dmac, read_db.dword));

}

static inline void baikal_dma_chan_dump_lli(struct baikal_dma_lli *lli)
{
	pr_err("\n\n desc: sar.low = 0x%x sar.high = 0x%x dar.low = 0x%x dar.high = 0x%x xfer_size = 0x%x ctrl = 0x%x \n\n",
			lli->sar.low_part,  lli->sar.high_part, lli->dar.low_part, lli->dar.high_part, lli->xfer_size, lli->ctrl);
}

/**
 * Enable or reset DMA read or write.
 *
 * This function sets the eDMA Enable Registers of write or read channels. The
 * ENB bit can be:
 *   - 1 Enable eDMA Write core logic.
 *   - 0 Reset the eDMA Write core logic.
 *
 * You must enable eDMA engine, before any other software setup actions,
 * for normal operation.
 *
 * You should reset the eDMA engine when there is any hardware error
 * is indicated by Error Status Registers.
 *
 */
void baikal_dma_set_op(struct baikal_dma *dmac, baikal_dma_dirc dirc, int enb)
{
	int val;
	if (dirc == DMA_WRITE) {
		dma_writel(dmac, write_enb.dword, enb);
	} else {
		dma_writel(dmac, read_enb.dword, enb);
	}

	/* We need to make sure the reset has been propagated to all logic. */
	if (!enb) {
		int i = 200;
		val = (dirc == DMA_WRITE) ?
			dma_readl(dmac, write_enb.dword)
			: dma_readl(dmac, read_enb.dword);
		while(val) {
			pr_err("baikal_dma_set_op: Wait 5 ms...");
			msleep(5);
			if (i-- == 0) {
				pr_err("baikal_dma_set_op: Reset propagation time out!");
				break;
			}
			val = (dirc == DMA_WRITE) ?
				dma_readl(dmac, write_enb.dword)
				: dma_readl(dmac, read_enb.dword);

		}
	}
}

/**
 * Set the weight of given channel.
 *
 * The weight value is used by the channel weighted round robin arbiter to
 * select the next channel write/read request.
 *
 */
void baikal_dma_set_weight(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan, uint8_t weight) 
{
	baikal_dma_weight_reg val;
	switch (chan % 4) {
		case 0:
			val.weight0 = weight;
			break;
		case 1:
			val.weight1 = weight;
			break;
		case 2:
			val.weight2 = weight;
			break;
		default:
			val.weight3 = weight;
			break;
	}

	if (dirc == DMA_WRITE) {
		if (chan < 4) {
			dma_writel(dmac, write_weight_lo.dword, val.dword);
		} else {
			dma_writel(dmac, write_weight_hi.dword, val.dword);
		}
	} else { /* DMA_READ */
		if (chan < 4) {
			dma_writel(dmac, read_weight_lo.dword, val.dword);
		} else {
			dma_writel(dmac, read_weight_hi.dword, val.dword);
		}
	}
}

/** 
 * Mask IRQ - set ABORT and/or DONE IRQ mask bit for a given channel 
 */
void baikal_dma_set_irq_mask(struct baikal_dma *dmac, struct baikal_dma_chan *chan, bool mask_done, bool mask_abort)
{
	uint32_t mask, new_mask = 0;
	if (mask_done)
		new_mask |= DMA_INT_DONE_MASK;
	if (mask_abort)
		new_mask |= DMA_INT_ABORT_MASK;
	new_mask = new_mask << chan->id;
	if (chan->dirc == DMA_WRITE) {
		mask = dma_readl(dmac, write_irq_mask) | new_mask;
		dma_writel(dmac, write_irq_mask, mask);
	} else { /* DMA_READ */
		mask = dma_readl(dmac, read_irq_mask) | new_mask;
		dma_writel(dmac, read_irq_mask, mask);
	}
}

/** 
 * Unmask IRQ - clear ABORT and/or DONE IRQ mask bit for a given channel
 */
void baikal_dma_reset_irq_mask(struct baikal_dma *dmac, struct baikal_dma_chan *chan, bool mask_done, bool mask_abort)
{
	uint32_t mask, new_mask = 0;
	if (mask_done)
		new_mask |= DMA_INT_DONE_MASK;
	if (mask_abort)
		new_mask |= DMA_INT_ABORT_MASK;
	new_mask = new_mask << chan->id;
	if (chan->dirc == DMA_WRITE) {
		mask = dma_readl(dmac, write_irq_mask) & ~new_mask;
		dma_writel(dmac, write_irq_mask, mask);
	} else { /* DMA_READ */
		mask = dma_readl(dmac, read_irq_mask) & ~new_mask;
		dma_writel(dmac, read_irq_mask, mask);
	}
}

/**
 * Set DMA Doorbell Register to kick off or stop a transfer of given channel.
 *
 * This function can be used in two scenarios:
 *   - A transfer request need to be performed. After all other DMA Context
 *     Registers are configured, you can call this function to start the
 *     transfer.
 *   - For some reason you want to terminate the transfer. By setting the STOP
 *     bit, the DMA channel stops issuing requests, and sets the Channel status
 *     to 'Stopped', and asserts the 'Abort' interrupt if it is enabled.
 *
 */
void baikal_dma_set_doorbell(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint32_t chan, uint32_t stop) 
{
	baikal_dma_doorbell_reg doorbell;
	doorbell.stop = stop;
	doorbell.channel = chan;

	if (dirc == DMA_READ) {
		dma_writel(dmac, read_db.dword, doorbell.dword);
	} else { /* DMA_WRITE */
		dma_writel(dmac, write_db.dword, doorbell.dword);
	}
}

void baikal_dma_transfer_start(struct baikal_dma_chan *edma_chan)
{
	baikal_dma_set_doorbell(to_baikal_dma(edma_chan->chan.device), edma_chan->dirc, edma_chan->id, FALSE);
}

void baikal_dma_transfer_stop(struct baikal_dma_chan *edma_chan)
{
	baikal_dma_set_doorbell(to_baikal_dma(edma_chan->chan.device), edma_chan->dirc, edma_chan->id, FALSE);
}

/**
 * Set DMA transfer size for given channel.
 *
 * This function is used only for single block transfer, in which case the user
 * must specify the size of the data to be transferred in DMA context register.
 *
 * In multi block transfer mode, the DMA overwrites the Transfer Size Register
 * with the corresponding DWORD of the linked list element, so user doesn't
 * need to call this function.
 *
 */
void baikal_dma_set_xfer_size(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint32_t chan, uint32_t size)
{
	baikal_dma_ctx_sel_reg sel;

	sel.sel = dirc;
	sel.viw_pnt = chan;

	dma_writel(dmac, ctx_sel.dword, sel.dword);
	channel_writel(dmac, chan, xfer_size, size);
}

/**
 * Set DMA transfer source for given channel.
 *
 * This function is used only for single block transfer. The address must be
 * DWORD aligned, indicating the next address to be read from.  The address is
 * always DWORD aligned even if the transfer size is byte aligned.
 *
 * In multi block transfer mode, the DMA overwrites the SAR Register
 * with the corresponding DWORD of the linked list element, so user doesn't
 * need to call this function.
 *
 */

void baikal_dma_set_src(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan, uint32_t addr ) 
{
	baikal_dma_ctx_sel_reg sel;   

	sel.sel = dirc;
	sel.viw_pnt = chan;

	dma_writel(dmac, ctx_sel.dword, sel.dword);

	channel_writel(dmac, chan, sar_ptr_lo, addr);
	channel_writel(dmac, chan, sar_ptr_hi, 0x0);
}

/**
 * Set DMA transfer destination for given channel.
 *
 * This function is used only for single block transfer. The address must be
 * DWORD aligned, indicating the next address to be written to.  The address is
 * always DWORD aligned even if the transfer size is byte aligned.
 *
 * In multi block transfer mode, the DMA overwrites the DAR Register
 * with the corresponding DWORD of the linked list element, so user doesn't
 * need to call this function.
 *
 */
void baikal_dma_set_dst(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan, uint32_t addr) 
{
	baikal_dma_ctx_sel_reg sel;

	sel.sel = dirc;
	sel.viw_pnt = chan;

	dma_writel(dmac, ctx_sel.dword, sel.dword);

	channel_writel(dmac, chan, dar_ptr_lo, addr);
	channel_writel(dmac, chan, dar_ptr_hi, 0x0);
}

/**
 * Set the Control Register for given channel.
 *
 * This function is used only for single block transfer. The DMA Write/Read
 * Control Registers contains a variaty of bits that can affect the transfer
 * behavior in different ways. See user mannual for detailed explanation. For
 * this function, user only need to provide the transfer mode, then the
 * register will be configured accordingly.
 *
 * The eDMA core can work in two transfer modes:
 *   - Single block transfer. Only one block of data is transferred.
 *   - Multi block transfer. Arbitrary blocks of data can be transferred based
 *     on the Linked List descriptor.
 *
 * In multi block transfer mode, the DMA overwrites the Control Register
 * with the corresponding DWORD of the linked list element, so user doesn't
 * need to call this function.
 *
 */
void baikal_dma_set_ctrl(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint32_t chan, baikal_dma_xfer_mode mode) 
{
	baikal_dma_chan_ctrl_lo cctrl;
	baikal_dma_ctx_sel_reg sel;

	cctrl.dword = 0x0; 

	switch (mode) {
		case DMA_SINGLE_BLOCK:
			cctrl.LIE = 1;
			cctrl.RIE = 1;
			cctrl.TD = 1;
			break;
		case DMA_MULTI_BLOCK:
			cctrl.LLEN = 1;
			cctrl.CCS = 1;
			cctrl.TD = 1;
		default:
			// Not supported
			break;
	} 

	sel.sel = dirc;
	sel.viw_pnt = chan;

	dma_writel(dmac, ctx_sel.dword, sel.dword);

	channel_writel(dmac, chan, ctrl_lo, cctrl.dword);
	channel_writel(dmac, chan, ctrl_hi, 0x0);
}

/**
 * Set the MSI address for given DMA transfer direction and interrupt type.
 *
 * This function will set the word aligned address of the MWr done/abort message
 * interrupt sent on the DXALI0 interface when an eDMA transfer completes. This
 * function is used only for remote interrupt. It doesn't make sense for local
 * interrupt.
 *
 * For your Endpoint, you can use the IMWr as an MSI or MSIX. For MSI, you must
 * program all IMWRr Address Registers with the same MSI address, as PCI
 * Express only supports a single MSI address per Function.
 */

void baikal_dma_set_msi_addr(struct baikal_dma *dmac, baikal_dma_dirc dirc, baikal_dma_irq_type type, uint32_t addr) 
{
	if (dirc == DMA_WRITE) {
		if (type == DMA_INT_DONE) {
			dma_writel(dmac, write_msg_done_lo, addr);
			dma_writel(dmac, write_msg_done_hi, 0);
		} else {
			dma_writel(dmac, write_msg_abt_lo, addr);
			dma_writel(dmac, write_msg_abt_hi, 0);
		}
	}
	else { /* DMA_READ */
		if (type == DMA_INT_DONE) {
			dma_writel(dmac, read_msg_done_lo, addr);
			dma_writel(dmac, read_msg_done_hi, 0);
		} else {
			dma_writel(dmac, read_msg_abt_lo, addr);
			dma_writel(dmac, read_msg_abt_hi, 0);
		}
	}
}

/**
 * Set the MSI data for given DMA channel.
 *
 * A single IMWr Data Register is used for both types of interrupts (Done or
 * Abort) of each channel. This function is used to specify that data. This
 * function is used only for remote interrupt. It doesn't make sense for local
 * interrupt.
 *
 */
void baikal_dma_set_msi_data(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan, int16_t data) 
{
	if (dirc == DMA_WRITE) {
		dma_writel(dmac, write_msg_data[chan], data);
	} else { /* DMA_READ */
		dma_writel(dmac, read_msg_data[chan], data);
	}
}

/**
 * Set Linked List Interrupt Error Enable Registers.
 *
 * This function can:
 * - Enable/disable generation of the local CPU linked list error fetch or
 *   data transfer bridge error interrupt status bits contained in the Error
 *   Status Registers.
 * - Enable/disable generation of the remote CPU Abort interrupt MWr TLP.
 *
 */
void baikal_dma_set_ll_irq_err(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan, int loc, int enb) 
{
	baikal_dma_ll_err_enb_reg val;

	if (loc)
		val.leie = 0x1 << chan;
	else
		val.reie = 0x1 << chan;

	if (!enb)
		val.dword = ~(val.dword);

	if (dirc == DMA_WRITE) {
		if (enb)
			val.dword |= dma_readl(dmac, write_irq_ll_err_enb.dword);
		else
			val.dword &= dma_readl(dmac, write_irq_ll_err_enb.dword);
		dma_writel(dmac, write_irq_ll_err_enb.dword, val.dword);
	} else { /* DMA_READ */
		if (enb)
			val.dword |= dma_readl(dmac, read_irq_ll_err_enb.dword);
		else
			val.dword &= dma_readl(dmac, read_irq_ll_err_enb.dword);
		dma_writel(dmac, read_irq_ll_err_enb.dword, val.dword);
	}
}

/**
 * Set the linked list for given channel.
 *
 * This function is used only for multi block transfer. It can inform the eDMA
 * core the starting address of the linked list in local memory by setting the
 * DMA Linked List Pointer Register during a multi block transfer
 * initialization.
 *
 * When the transfer get started, the Linked List Pointer Register will be
 * updated by the DMA to point to the next element in the transfer list after
 * the previous element is consumed.
 *
 */
void baikal_dma_set_ll(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan, uint32_t *addr)
{
	baikal_dma_ctx_sel_reg sel;

	sel.sel = dirc;
	sel.viw_pnt = chan;

	dma_writel(dmac, ctx_sel.dword, sel.dword);

	channel_writel(dmac, chan, el_ptr_lo, (uint32_t)addr);
	channel_writel(dmac, chan, el_ptr_hi, 0x0);
}

/**
 * Set the data element for given linked list.
 *
 * This function is used only for multi block transfer. For each linked list
 * data element, data producer should provide the source address, destination
 * address, the transfer size and the control code. User can utilize this
 * function to build a linked list one element by one element in specified
 * local memory.
 *
 * This function should be called when:
 *   - Initialize the linked list before the multi block transfer get started.
 *   - Update the data element during the multi block transfer.
 *
 */
void baikal_dma_set_data_ele(uint32_t* lladdr, uint8_t idx, int pcs, int intr,
		uint32_t xfer_size, uint32_t src, uint32_t dst)
{
	baikal_dma_chan_ctrl_lo cctrl;
	struct baikal_dma_lli *lli;

	lli = (struct baikal_dma_lli *) lladdr;

	cctrl.dword = 0x0;
	cctrl.CB = pcs;
	cctrl.RIE = 0;
	cctrl.LIE = intr;

	lli[idx].xfer_size = xfer_size;
	lli[idx].sar.low_part = src;
	lli[idx].sar.high_part = 0x0;
	lli[idx].dar.low_part = dst;
	lli[idx].dar.high_part = 0x0;
	lli[idx].ctrl = cctrl.dword;
}

/**
 * Get the data element for given linked list.
 *
 * This function can read the content of a specified element in the linked list.
 * It's an utility function that might be helpful when the low level driver code
 * need to know how the user feed the linked list.
 *
 */
void baikal_dma_get_data_ele(uint32_t *lladdr, uint8_t idx, int *pcs, int *intr,
		uint32_t *xfer_size, phyaddr *src, phyaddr *dst)
{
	struct baikal_dma_lli *lli;

	lli = (struct baikal_dma_lli *) lladdr;

	if (xfer_size != NULL)
		*xfer_size = dma_readl_native(&lli[idx].xfer_size);
	if (src != NULL) {
		src->low_part = dma_readl_native(&lli[idx].sar.low_part);
		src->high_part = dma_readl_native(&lli[idx].sar.high_part);
	}
	if (dst != NULL) {
		dst->low_part = dma_readl_native(&lli[idx].dar.low_part);
		dst->high_part = dma_readl_native(&lli[idx].dar.high_part);
	}
	if (pcs != NULL || intr != NULL) {
		baikal_dma_chan_ctrl_lo cctrl;
		cctrl.dword = dma_readl_native(&lli[idx].ctrl);
		if (pcs != NULL) *pcs = cctrl.CB;
		if (intr != NULL) *intr = cctrl.RIE;
	}
}

/**
 * Set the link element for given linked list.
 *
 * This function is used only for multi block transfer. The standard linked
 * list is made up with multiple data elements and one link element. The link
 * element contains the pointer to next element structure, informing the DMA
 * core where to go after consuming up current linked list. It can either point
 * to another linked list, or the starting element of current list
 * (called Recycling).
 *
 * Generally this function should be called to initialize the linked list
 * before the multi block transfer get started.
 *
 */
void baikal_dma_set_link_ele(uint32_t *lladdr, uint8_t idx, int pcs, uint32_t *nxt, int recyc) 
{
	baikal_dma_chan_ctrl_lo cctrl;
	struct baikal_dma_lli *lli;

	lli = (struct baikal_dma_lli *) lladdr;
	cctrl.dword = 0x0;
	cctrl.CB = pcs;
	cctrl.TCB = recyc;
	cctrl.LLP = 1;

	lli[idx].sar.low_part = (uint32_t) nxt;
	lli[idx].sar.high_part = 0x0;
	lli[idx].ctrl = cctrl.dword;
}

/**
 * Get the link element for given linked list.
 *
 * This function can read the content of the link element of the linked list.
 * It's an utility function that might be helpful when the low level driver code
 * need to know how the user feed the linked list.
 *
 */
void baikal_dma_get_link_ele(uint32_t *lladdr, uint8_t idx, int *pcs, phyaddr *nxt)
{
	struct baikal_dma_lli *lli;

	lli = (struct baikal_dma_lli *) lladdr;

	if (nxt != NULL) {
		nxt->low_part = dma_readl_native(&lli[idx].sar.low_part);
		nxt->high_part = dma_readl_native(&lli[idx].sar.high_part);
	}
	if (pcs != NULL) {
		baikal_dma_chan_ctrl_lo cctrl;
		cctrl.dword = lli[idx].ctrl;
		*pcs = cctrl.CB;
	}
}

/**
 * Get the number of enabled channels for both write and read direction.
 *
 * The eDMA core has at most 8 write channels and 8 read channels, but there
 * might be fewer enabled channels according to the static configuration of
 * the core. This function can get the actual enabled channel numbers by reading
 * the dedicated DMA register.
 *
 */
void baikal_dma_get_chan_num(struct baikal_dma *dmac, unsigned *wrch, unsigned *rdch)
{
	baikal_dma_ctrl_reg ctrl;

	ctrl.dword = dma_readl(dmac, dma_ctrl.dword);

	*wrch = ctrl.write_chans;
	*rdch = ctrl.read_chans;
}

/**
 * Get the status of given channel.
 *
 * The channel status bits of DMA Channel Control Register identify the current
 * operational state of the DMA write or read channel. The operation state
 * encoding for each DMA Channel is a s follows:
 *   - 00: Reserved
 *   - 01: Running. This channel is active and transferring data.
 *   - 10: Halted. An error condition has been detected, and the DMA has stopped
 *     this channel.
 *   - 11: Stopped. The DMA has transferred all data for this channel.
 *
 * This function gets the channel status bits and converts it to DMA_CHAN_STATUS
 * enumeration.
 *
 */
void baikal_dma_get_chan_status(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan, baikal_dma_chan_status *status)
{
	baikal_dma_ctx_sel_reg sel;
	baikal_dma_chan_ctrl_lo ctrllo;

	sel.sel = dirc;
	sel.viw_pnt = chan;

	dma_writel(dmac, ctx_sel.dword, sel.dword);

	ctrllo.dword =  channel_readl(dmac, chan, ctrl_lo.dword);

	*status = (baikal_dma_chan_status) ctrllo.CS;
}

/**
 *
 * Another variant for external API
 *
 */
baikal_dma_chan_status baikal_dma_chan_get_status(struct baikal_dma_chan *edma_chan)
{
	baikal_dma_chan_status status;
	struct baikal_dma *dmac = to_baikal_dma(edma_chan->chan.device);
	spin_lock(&dmac->lock);
	baikal_dma_get_chan_status(dmac, edma_chan->dirc, edma_chan->id, &status);
	spin_unlock(&dmac->lock);
	return status;
}

/**
 * Get the transfer size of given channel.
 *
 * The Transfer Size Register is initialized by driver before the transfer is
 * started and automatically decremented by the DMA as the transfer progresses.
 * So by reading that register, this function can return the size of the
 * remaining bytes have not yet been transferred for current block.
 *
 * @note In multi block mode, the DMA overwrites Transfer Size Register with the
 * corresponding DWORD of the linked list element.
 *
 */
void baikal_dma_get_xfer_size(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan, uint32_t *size) 
{
	baikal_dma_ctx_sel_reg sel;

	sel.sel = dirc;
	sel.viw_pnt = chan;

	dma_writel(dmac, ctx_sel.dword, sel.dword);

	*size = channel_readl(dmac, chan, xfer_size);
}

/**
 * Get the element pointer of given channel.
 *
 * When the transfer get started, the Element Pointer Register will be
 * updated by the DMA to point to the next element in the transfer list after
 * the previous element is consumed. So by reading that register, this function
 * can know which element the DMA core is currently working on. This function
 * only makes sense for multi block transfer.
 *
 */
void baikal_dma_get_element_ptr(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan, phyaddr *addr) 
{
	baikal_dma_ctx_sel_reg sel;

	sel.sel = dirc;
	sel.viw_pnt = chan;

	dma_writel(dmac, ctx_sel.dword, sel.dword);

	addr->low_part = channel_readl(dmac, chan, el_ptr_lo);
	addr->high_part = channel_readl(dmac, chan, el_ptr_hi);
}


/**
 * Query the interrupt source.
 *
 * This function queries Interrupt Status Registers to tell which channel
 * signals the interrupt. If the DMA read channel has successfully completed
 * the DMA read transfer, the corresponding Done bit of Interrupt Status
 * Register will be set to 1; If the DMA read channel has detected an
 * error, or you manually stopped the transfer, The correspoinding Abort bit
 * of Interrupt Status Register will be set to 1.
 *
 */
bool baikal_dma_query_irq_src(struct baikal_dma *dmac, baikal_dma_dirc *dirc, uint8_t *chan,
		baikal_dma_irq_type *type)
{
	bool found;
	int ch, bitv;

	baikal_dma_irq_reg wr_irq_status;
	baikal_dma_irq_reg rd_irq_status;

	found = false;

	wr_irq_status.dword =  dma_readl(dmac, write_irq_status.dword);

	for (ch = 0; ch < BAIKAL_EDMA_TOTAL_CHANNELS; ch++) {
		bitv = 1 << ch;

		if (wr_irq_status.abort_status & bitv) {
			*chan = ch;
			*dirc = DMA_WRITE;
			*type = DMA_INT_ABORT;
			found = true;
			break;
		}

		if (wr_irq_status.done_status & bitv) {
			*chan = ch;
			*dirc = DMA_WRITE;
			*type = DMA_INT_DONE;
			found = true;
			break;
		}
	}

	if (!found) {
		rd_irq_status.dword = dma_readl(dmac, read_irq_status.dword);

		for (ch = 0; ch < BAIKAL_EDMA_TOTAL_CHANNELS; ch++) {
			bitv = 1 << ch;
			if (rd_irq_status.abort_status & bitv) {
				*chan = ch;
				*dirc = DMA_READ;
				*type = DMA_INT_ABORT;
				found = true;
				break;
			}

			if (rd_irq_status.done_status & bitv) {
				*chan = ch;
				*dirc = DMA_READ;
				*type = DMA_INT_DONE;
				found = true;
				break;
			}
		}
	}

	return found;
}

/**
 * Query the source of eDMA core error.
 *
 * This function queries Error Status Registers to identify the source of the
 * hardware error. DMA core can possibly generate seven types of errors. Some
 * of them are fatal error and others are non-fatal. For non-fatal errors, the
 * software can request the DMA to continue processing.
 *
 */
bool baikal_dma_query_err_src(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan,
		baikal_dma_err *type)
{
	bool is_err = true;
	uint32_t val = (0x1 << chan);

	if (dirc == DMA_WRITE) {
		baikal_dma_write_err wr_err;
		wr_err.dword = dma_readl(dmac, write_err_status.dword);

		if (wr_err.brdg_err & val) {
			*type = DMA_ERR_WR;
			return is_err;
		}
		if (wr_err.ll_err & val) {
			*type = DMA_ERR_FETCH_LL;
			return is_err;
		}
	} else {
		baikal_dma_read_err_lo rd_err_lo;
		baikal_dma_read_err_hi rd_err_hi;

		rd_err_lo.dword = dma_readl(dmac, read_err_status_lo.dword);
		if (rd_err_lo.brdg_err & val) {
			*type = DMA_ERR_RD;
			return is_err;
		}
		if (rd_err_lo.ll_err & val) {
			*type = DMA_ERR_FETCH_LL;
			return is_err;
		}

		rd_err_hi.dword = dma_readl(dmac, read_err_status_hi.dword);
		if (rd_err_hi.ur_err & val) {
			*type = DMA_ERR_UNSUPPORTED_RQST;
			return is_err;
		}
		if (rd_err_hi.ca_err & val) {
			*type = DMA_ERR_COMPLETER_ABORT;
			return is_err;
		}
		if (rd_err_hi.to_err & val) {
			*type = DMA_ERR_CPL_TIME_OUT;
			return is_err;
		}
		if (rd_err_hi.ep_err & val) {
			*type = DMA_ERR_DATA_POISONING;
			return is_err;
		}
	}
	return !is_err;
}

/**
 * Get the state of given channel.
 *
 * This function should be called as protected code where no other thread or
 * HW interrupt can break it.
 *
 */
static void baikal_dma_get_chan_state(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan,
		uint32_t *status, uint32_t *xfer_size, phyaddr *element_pos)
{
	baikal_dma_get_chan_status(dmac, dirc, chan, status);
	baikal_dma_get_xfer_size(dmac, dirc, chan, xfer_size);
	baikal_dma_get_element_ptr(dmac, dirc, chan, element_pos);
}

/**
 * Abort an undergoing DMA transfer.
 *
 * An undergoing DMA transfer can be terminated any time as requested by calling
 * this function. The Stop bit of Doorbell Register will be set. After that,
 * this function will check the status of the channel to see if the transfer is
 * successfully terminated.
 *
 * This function takes no effect if the transfer has already stopped before we
 * terminate it.
 *
 */
baikal_dmareturn_t baikal_dma_abort(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan)
{
	uint32_t status, xfer_size;
	phyaddr element_pos;

	if (dirc == DMA_WRITE && chan >= dmac->write_chans)
		return DMA_INVALID_CHAN;

	if (dirc == DMA_READ && chan >= dmac->read_chans)
		return DMA_INVALID_CHAN;

	baikal_dma_set_doorbell(dmac, dirc, chan, TRUE);

	/* Check if the abort request is successfully performed. */
	msleep(5);

	baikal_dma_get_chan_state(dmac, dirc, chan, &status, &xfer_size, &element_pos);

	if (status == CHAN_RUNNING)
		return DMA_UNKNOWN_FAILURE;

	return DMA_SUCCESS;
}

/**
 * Stop an undergoing DMA transfer.
 *
 * This function is similar with HalDmaAbort but it uses a different way to
 * stop the undergoing transfer on specified channel. This function does not
 * touch the Stop bit of Doorbell Register. Instead, it makes the driver no
 * longer update the linked list, so the DMA engine will stop when it can't
 * find any new data in the linked list.
 *
 * Since the Doorbell Register is not touched, the DMA engine will not issue
 * Abort interrupt. Besides, there will be some delay between you call this
 * function and the DMA transfer stop, because the DMA engine will need to
 * finish all remained data in current linked list.
 *
 * This function takes no effect if the transfer has already stopped before we
 * terminate it.
 *
 */
baikal_dmareturn_t baikal_dma_chan_stop(struct baikal_dma *dmac, struct baikal_dma_chan *edma_chan)
{
	baikal_dma_dirc dirc = edma_chan->dirc; 
	uint8_t chan = edma_chan->id;
	if (dirc == DMA_WRITE && chan >= dmac->write_chans)
		return DMA_INVALID_CHAN;

	if (dirc == DMA_READ && chan >= dmac->read_chans)
		return DMA_INVALID_CHAN;

	edma_chan->recycle = 1;

	return DMA_SUCCESS;
}

static struct baikal_dma_desc *baikal_dma_chan_first_active(struct baikal_dma_chan *edma_chan)
{
	return to_baikal_dma_desc(edma_chan->active_list.next);
}

/* Prepare single block transfer */
static baikal_dmareturn_t inline baikal_dma_prepare_single_block(struct baikal_dma *dmac, struct baikal_dma_chan *chan, struct baikal_dma_desc *desc)
{
	uint32_t status;

	if (dmac == NULL || chan == NULL)
		return DMA_NULL_ARG;

	spin_lock(&dmac->lock);

	baikal_dma_set_op(dmac, chan->dirc, TRUE);
	baikal_dma_set_ctrl(dmac, chan->dirc, chan->id, DMA_SINGLE_BLOCK);
	baikal_dma_set_xfer_size(dmac, chan->dirc, chan->id, chan->xfer_size);
	baikal_dma_set_src(dmac, chan->dirc, chan->id, desc->lli.sar.low_part);
	baikal_dma_set_dst(dmac, chan->dirc, chan->id, desc->lli.dar.low_part);
	baikal_dma_set_weight(dmac, chan->dirc, chan->id, chan->priority);

	if (chan->dirc == DMA_WRITE) {
		dma_writel(dmac, write_irq_mask, 0);
	} else { /* DMA_READ */
		dma_writel(dmac, read_irq_mask, 0);
	}

	//	baikal_dma_chan_dump_regs(dmac, chan);

	/* Check the status of the channel to make sure it's not running. Otherwise
	 * we can't start new transfer on this channel.
	 */
	baikal_dma_get_chan_status(dmac, chan->dirc, chan->id, &status);

	spin_unlock(&dmac->lock);

	if (status == CHAN_RUNNING) {
		return DMA_CHAN_BUSY;
	} else {
		return DMA_SUCCESS;
	}

}

/**
 * Resume an aborted DMA transfer.
 *
 * An undergoing DMA transfer can be terminated any time as requested by calling
 * baikal_dma_abort() function. If user want to continue the stopped transfer, this
 * function can be called.
 *
 * @note The resume function is only for debug use and user should not use this
 * feature in any real world application.
 *
 * For single block transfer, this function just re-initialize all DMA registers
 * to start the transfer from the very beginning, not really "resume" from
 * where the transfer is "paused".
 *
 * For multi block transfer, this function just simply set the Doorbell
 * register. The DMA core will fetch the data element from the linked list
 * according to the current element pointer it holds. So, the transfer is
 * roughly "resumed", but the data block stopped last time will be transferred
 * again from beginning.
 *
 */
baikal_dmareturn_t baikal_dma_resume(struct baikal_dma *dmac, struct baikal_dma_chan *edma_chan, baikal_dma_dirc dirc) 
{
	struct baikal_dma_desc *desc;
	uint32_t status;
	uint8_t chan = edma_chan->id;

	if (dirc == DMA_WRITE) {
		if (chan >= dmac->write_chans)
			return DMA_INVALID_CHAN;
	} else { /* DMA_READ */
		if (chan >= dmac->read_chans)
			return DMA_INVALID_CHAN;
	}

	/* Make sure the channel is stopped */
	baikal_dma_get_chan_status(dmac, dirc, chan, &status);

	if (status != CHAN_STOPPED) {
		return DMA_CHAN_BUSY;
	}

	if (edma_chan->element_pos.quad_part) {
		baikal_dma_set_doorbell(dmac, dirc, chan, FALSE);
	} else {
		desc = baikal_dma_chan_first_active(edma_chan);
		baikal_dma_prepare_single_block(dmac, edma_chan, desc);
	}
	return DMA_SUCCESS;
}

baikal_dmareturn_t baikal_dma_transfer_prepare_single(struct baikal_dma_chan *edma_chan, phyaddr src, phyaddr dst, uint32_t size) 
{
	struct baikal_dma_desc desc;
	if (!edma_chan)
		return DMA_NULL_ARG;
	baikal_dma_chan_setup(edma_chan, size, TRUE);
	baikal_dma_desc_fill(&desc, src, dst, size);
	return baikal_dma_prepare_single_block(to_baikal_dma(edma_chan->chan.device), edma_chan, &desc);
}

/**
 * Flush queuing DMA read/write jobs.
 *
 * Queuing jobs are those DMA transfer requests which have all context registers 
 * been configured but Doorbell is not set. This function will automatically
 * search for queuing jobs, if any, and kick off the transfer by set Doorbell.
 * If no queuing job are found, this function will report error and no DMA
 * registers will be touched.
 *
 */
baikal_dmareturn_t baikal_dma_flush_queue(struct baikal_dma *dmac) 
{
	baikal_dma_dirc dirc;
	uint8_t chan;
	uint32_t status;

	/* Check all write channels to see if there is any queuing job. */
	dirc = DMA_WRITE;
	for (chan = 0; chan < dmac->write_chans; chan++) {
		baikal_dma_get_chan_status(dmac, dirc, chan, &status);
		if (status == CHAN_QUEUING) {
			baikal_dma_set_doorbell(dmac, dirc, chan, FALSE);
		}
	}
	/* Check all read channels to see if there is any queuing job. */
	dirc = DMA_READ;
	for (; chan < dmac->read_chans + dmac->write_chans; chan++) {
		baikal_dma_get_chan_status(dmac, dirc, chan, &status);
		if (status == CHAN_QUEUING) {
			baikal_dma_set_doorbell(dmac, dirc, chan, FALSE);
		}
	}

	return DMA_SUCCESS;
}

/**
 * Clear the interrupt.
 *
 * When driver receives a interrupt from eDMA device, it must clear the
 * interrupt source to announce that the interrupt has been handled.
 *
 */
void baikal_dma_clr_irq(struct baikal_dma *dmac, baikal_dma_dirc dirc, uint8_t chan, baikal_dma_irq_type type) 
{
	baikal_dma_irq_clear_reg irq_clr;

	if (type == DMA_INT_ABORT) {
		irq_clr.abort_clear = (0x1 << chan);
	} else {
		irq_clr.done_clear = (0x1 << chan);
	}

	if (dirc == DMA_WRITE) {
		dma_writel(dmac, write_irq_clear, irq_clr.dword);
	} else {
		dma_writel(dmac, read_irq_clear, irq_clr.dword);
	}
}

/**
 * Prepare registers for multi block transfer.
 *
 * This function should be called as protected code where no other thread or
 * HW interrupt can break it.
 *
 */
static baikal_dmareturn_t baikal_dma_init_multi_block_xfer(struct baikal_dma *dmac, struct baikal_dma_chan *edma_chan) 
{
	baikal_dma_set_op(dmac, edma_chan->dirc, TRUE);
	//baikal_dma_set_ll_irq_err(dmac, edma_chan->dirc, edma_chan->id, TRUE, TRUE); // was commented out
	baikal_dma_clr_irq(dmac, edma_chan->dirc, edma_chan->id, DMA_INT_ABORT);
	baikal_dma_clr_irq(dmac, edma_chan->dirc, edma_chan->id, DMA_INT_DONE);
	baikal_dma_set_ll_irq_err(dmac, edma_chan->dirc, edma_chan->id, FALSE, TRUE);
	baikal_dma_set_ctrl(dmac, edma_chan->dirc, edma_chan->id, DMA_MULTI_BLOCK);
	baikal_dma_set_ll(dmac, edma_chan->dirc, edma_chan->id, edma_chan->llp_addr);
	baikal_dma_set_weight(dmac, edma_chan->dirc, edma_chan->id, edma_chan->weight);
	return DMA_SUCCESS;
}

static baikal_dmareturn_t baikal_dma_desc_get_data(struct baikal_dma_desc *desc, uint32_t *src, uint32_t *dst, uint32_t *xfer_size) 
{
	if (desc) {
		*src = desc->lli.sar.low_part;
		*dst = desc->lli.dar.low_part;
		*xfer_size = desc->lli.xfer_size;
		return DMA_SUCCESS;
	} else {
		return DMA_NO_QUEUING_JOBS; 
	}
}

static baikal_dmareturn_t baikal_dma_get_next_desc_data(struct baikal_dma_chan *edma_chan, uint32_t *src, uint32_t *dst, uint32_t *xfer_size) 
{
	struct baikal_dma_desc *desc;
	baikal_dmareturn_t ret;

	desc = baikal_dma_chan_first_active(edma_chan);

	if (desc) {
		ret = baikal_dma_desc_get_data(desc, src, dst, xfer_size);
		list_del_init(&desc->desc_node);
		return ret;
	}

	return DMA_NO_QUEUING_JOBS;
}

/**
 * Update linked list element.
 *
 * For multi block transfer on either read or write channel, this function
 * is called when the transfer is initialized. This function will also be
 * called each time the channel signals a Done interrupt and the transfer is
 * not finished. The HAL and the DMA core collaborate in Producer-Consumer
 * paradigm. See Linked List Operation section of the user mannual for details.
 *
 */
static baikal_dmareturn_t baikal_dma_update_ll_elements(struct baikal_dma *dmac, struct baikal_dma_chan *edma_chan) 
{
	uint8_t lle, wm, pcs, is_init, i;
	uint32_t src, dst, xfer_size;
	uint32_t *llvaddr;
	int is_wm_ele, is_last_data_ele;

	lle       = edma_chan->ll_elements;
	wm        = edma_chan->water_mark;
	pcs       = edma_chan->pcs;
	is_init   = (edma_chan->done_intr_cnt == 0);
	llvaddr   = edma_chan->llv_addr;
	src       = edma_chan->src_pos;
	dst       = edma_chan->dst_pos;

	/* Get the data of next element */
	if (baikal_dma_get_next_desc_data(edma_chan, &src, &dst, &xfer_size) != DMA_SUCCESS) {
		return DMA_FATAL;
	}
	edma_chan->feed_size += xfer_size;
	edma_chan->src_pos = src;
	edma_chan->dst_pos = dst;

	i = edma_chan->element_idx;

	/* !!! Don't flip the pcs of the first element */
	baikal_dma_set_data_ele(llvaddr, i, !pcs, FALSE, xfer_size, src, dst);

	/* Refresh the element data one by one along the linked list until it
	 * encounters watermark element or last data element
	 */
	while (TRUE) {
		i++;
		if (baikal_dma_get_next_desc_data(edma_chan, &src, &dst, &xfer_size) != DMA_SUCCESS) {
			return DMA_FATAL;
		}
		edma_chan->feed_size += xfer_size;
		is_wm_ele = (i == wm);
		is_last_data_ele = (i == (lle - 2));
		baikal_dma_set_data_ele(llvaddr, i, pcs, (is_wm_ele || is_last_data_ele), xfer_size,
				src, dst);
		if (is_last_data_ele || (!is_init && is_wm_ele))
			break;
	}

	/* Set the link element in recycling manner */
	if (is_last_data_ele) {
		baikal_dma_set_link_ele(llvaddr, lle - 1, !pcs, edma_chan->llp_addr, TRUE);
	}

	/* Now flip the pcs of the first element */
	baikal_dma_set_data_ele(llvaddr, edma_chan->element_idx, pcs, FALSE, xfer_size,
			edma_chan->src_pos, edma_chan->dst_pos);

	/* Remember where we are */
	edma_chan->src_pos = src;
	edma_chan->dst_pos = dst;
	edma_chan->element_idx = is_last_data_ele ? 0 : (i + 1);
	edma_chan->pcs = is_last_data_ele ? !pcs : pcs;
	return DMA_SUCCESS;
}

/**
 * Start DMA multi block read or write operation. This is the worker function
 * for external multi block read/write APIs.
 *
 */
static baikal_dmareturn_t baikal_dma_prepare_multi_block_transfer(struct baikal_dma *dmac, struct baikal_dma_chan *edma_chan)
{
	uint32_t status;
	baikal_dmareturn_t ret;

	if (dmac == NULL || edma_chan == NULL)
		return DMA_NULL_ARG;

	if (edma_chan->dirc == DMA_WRITE) {
		if (edma_chan->id >= dmac->write_chans)
			return DMA_INVALID_CHAN;
	} else {
		if (edma_chan->id >= dmac->read_chans)
			return DMA_INVALID_CHAN;
	}

	/* Reset/init channel info */
	edma_chan->weight = 0;
	edma_chan->pcs = 1;
	edma_chan->element_idx = 0;
	edma_chan->src_pos = 0;
	edma_chan->dst_pos = 0;
	edma_chan->done_intr_cnt = 0;
	edma_chan->abort_intr_cnt = 0;
	edma_chan->error = DMA_ERR_NONE;

	/* Initialize the elements of the linked list */
	ret = baikal_dma_update_ll_elements(dmac, edma_chan);
	if (ret != DMA_SUCCESS) {
		return ret;
	}

	/* Configure the context registers for multi block transfer. */
	baikal_dma_init_multi_block_xfer(dmac, edma_chan);

	/* Set Doorbell register */
	baikal_dma_set_doorbell(dmac, edma_chan->dirc, edma_chan->id, FALSE);

	/* Check the status of the channel to make sure it's not running. Otherwise
	 * we can't start new transfer on this channel.
	 */
	baikal_dma_get_chan_status(dmac, edma_chan->dirc, edma_chan->id, &status);
	if (status == CHAN_RUNNING) {
		return DMA_CHAN_BUSY;
	} else {
		return DMA_SUCCESS;
	}
}

static inline bool baikal_dma_interrupt_handle_error(struct baikal_dma *dmac, struct baikal_dma_chan *edma_chan)
{
	baikal_dma_err err_type = edma_chan->error;
	if (err_type == DMA_ERR_WR || err_type == DMA_ERR_RD || err_type == DMA_ERR_FETCH_LL) {
		/* Fatal errors. Can't recover. Return true for caller to invoke callback */
		return true;
	} else {
		return false; /* do not invoke callback upon return */
	}
}

static inline bool baikal_dma_interrupt_handle_done_multi_block(struct baikal_dma *dmac, struct baikal_dma_chan *edma_chan)
{
	baikal_dmareturn_t ret;
	uint32_t status;
	baikal_dma_dirc dirc = edma_chan->dirc;
	uint8_t chan = edma_chan->id;
	if (edma_chan->recycle == 0 || edma_chan->done_intr_cnt < (2 * edma_chan->recycle - 1)) {
		pr_err("Update linked list elements.");
		ret = baikal_dma_update_ll_elements(dmac, edma_chan);
		if (ret == DMA_SUCCESS) {
			baikal_dma_get_chan_status(dmac, dirc, chan, &status);
			if (status == CHAN_STOPPED) {
				pr_err("channel stopped, restart it...");
				baikal_dma_set_doorbell(dmac, dirc, chan, FALSE);
			}
		} else {
			pr_err("Failed to update elements.");
		}
		return false; /* do not invoke callback upon return */
	} else {
		return true;
	}
}

/**
 * Submit IRQ clear bits to the corresponding register.
 * Error status register is reset automatically.
 */
static inline void baikal_dma_interrupt_fast_irq_clear(struct baikal_dma *dmac, baikal_dma_dirc dirc, baikal_dma_irq_clear_reg irq_clr)
{
	if (dirc == DMA_WRITE) {
		dma_writel_native((irq_clr.dword), &(__baikal_dma_regs(dmac)->write_irq_clear));
	} else {
		dma_writel_native((irq_clr.dword), &(__baikal_dma_regs(dmac)->read_irq_clear));
	}
}

/**
 * Interrupt service routine for DMA transfer.
 */

static irqreturn_t baikal_dma_interrupt(int irq, void *dev_id)
{
	uint8_t chan;
	uint32_t id;
	uint32_t mask;
	struct baikal_dma *dmac;
	struct baikal_dma_chan *edma_chan;
	baikal_dma_dirc dirc;
	baikal_dma_irq_clear_reg irq_clr;
	bool invoke_callback;
	uint32_t time1, time2;

	time1 = read_c0_count();

	dmac = dev_id;	
	chan = irq - dmac->chan[0].irq;
	edma_chan = &dmac->chan[chan];

	/**
	 * The order is strict.
	 * 1. Mask abort and done IRQ, then read IRQ status register to determine interrupt type.
	 */
	dirc = edma_chan->dirc;
	mask = edma_chan->mask;

	if (dirc == DMA_WRITE) {
		dma_writel_native((dma_readl(dmac, write_irq_mask) | mask),
				&(__baikal_dma_regs(dmac)->write_irq_mask));
		edma_chan->irq_status.dword = dma_readl(dmac, write_irq_status.dword);
	} else {
		dma_writel_native((dma_readl(dmac, read_irq_mask) | mask),
				&(__baikal_dma_regs(dmac)->read_irq_mask));
		edma_chan->irq_status.dword = dma_readl(dmac, read_irq_status.dword);
	}

	/**
	 * 2. Check if abort occured.
	 * If yes - read error status register and set 'abort clear' bit.
	 * Set 'done clear' bit anyway.
	 * Also update counters.
	 */
	id = edma_chan->id;
	irq_clr.done_clear = (0x1 << id);

	if (edma_chan->irq_status.abort_status & (1 << id)) {
		edma_chan->has_err = baikal_dma_query_err_src(dmac, dirc, id, &edma_chan->error);
		irq_clr.abort_clear = (0x1 << id);
		/* Clear IRQ, error status register is reset automatically by hardware, but we've just saved have the error state. */
		baikal_dma_interrupt_fast_irq_clear(dmac, dirc, irq_clr);
		edma_chan->abort_intr_cnt++;
		if (edma_chan->has_err)
			invoke_callback = baikal_dma_interrupt_handle_error(dmac, edma_chan);
		else 
			invoke_callback = true;
	} else {
		/* Clear IRQ and don't mind the error status since we have no errors. */
		baikal_dma_interrupt_fast_irq_clear(dmac, dirc, irq_clr);
		edma_chan->done_intr_cnt++;
		if (edma_chan->ll_elements == 0)
			invoke_callback = true;
		else
			invoke_callback = baikal_dma_interrupt_handle_done_multi_block(dmac, edma_chan);
	}

	/**
	 * 3. Execute user callback, if defined.
	 */
	if (invoke_callback && edma_chan->callback)
		edma_chan->callback(edma_chan);

	/**
	 * 4. Unmask IRQs
	 */
	if (dirc == DMA_WRITE) {
		dma_writel_native((dma_readl(dmac, write_irq_mask) & ~mask),
				&(__baikal_dma_regs(dmac)->write_irq_mask));
	} else {
		dma_writel_native((dma_readl(dmac, read_irq_mask) & ~mask),
				&(__baikal_dma_regs(dmac)->read_irq_mask));
	}

	time2 = read_c0_count();
	if (time2 >= time1)
		time2 -= time1;
	else
		time2 = 0xFFFFFFFF - (time1 - time2) + 1;
	edma_chan->time += time2;

	return IRQ_HANDLED;
}

/**
 * Nesessary dmaengine APIs.
 */

static int baikal_dma_chan_alloc_chan_resources(struct dma_chan *chan)
{
	return 0;
}

static void baikal_dma_chan_free_chan_resources(struct dma_chan *chan)
{
}

#if 0
static int baikal_dma_chan_config(struct dma_chan *chan, struct dma_slave_config *sconfig)
{
	struct baikal_dma_chan *edma_chan = to_baikal_dma_chan(chan);

	/* Check if chan will be configured for slave transfers */
	if (!is_slave_direction(sconfig->direction))
		return -EINVAL;

	memcpy(&edma_chan->dma_sconfig, sconfig, sizeof(*sconfig));
	edma_chan->dirc = sconfig->direction;

	/*convert_burst(&dwc->dma_sconfig.src_maxburst);
	  convert_burst(&dwc->dma_sconfig.dst_maxburst);*/

	return 0;
}
#endif

	static enum dma_status
baikal_dma_chan_tx_status(struct dma_chan *chan, dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	return DMA_SUCCESS;
}

static void baikal_dma_chan_issue_pending(struct dma_chan *chan)
{
}

/**
 * Initialize error information.
 */
static void baikal_dma_init_err_info(uint32_t idx) {
	baikal_dma_err_info[DMA_SUCCESS] = "Success";
	baikal_dma_err_info[DMA_CHAN_BUSY] = "Channel is busy.";
	baikal_dma_err_info[DMA_INVALID_CHAN] = "Channel is invalid.";
	baikal_dma_err_info[DMA_INVALID_BLOCK_SIZE] = "Block size is invalid.";
	baikal_dma_err_info[DMA_INVALID_LL_LEN] = "Linked list element length is invalid.";
	baikal_dma_err_info[DMA_INVALID_WM] = "WaterMark element is invalid.";
	baikal_dma_err_info[DMA_INVALID_WEIGHT] = "Weight value is invalid.";
	baikal_dma_err_info[DMA_NULL_ARG] = "Argument is null.";
	baikal_dma_err_info[DMA_UNKNOWN_FAILURE] = "Unknown failure.";
	baikal_dma_err_info[DMA_NO_QUEUING_JOBS] = "Cannot find queuing jobs.";
	baikal_dma_err_info[DMA_NO_FEED] = "Cannot get more data from producer callback.";
	baikal_dma_err_info[DMA_FATAL] = "DMA fatal error! Unexpected input or output.";
	return;
}

/**
 * Get detailed error information of given code.
 *
 * If the HAL APIs return unseccessful code, there might be dozens of cause
 * for the error. This function call tell the user the exact cause of given
 * code.
 *
 */
char * baikal_dma_get_err_info(baikal_dmareturn_t code) {
	if (code < DMA_SUCCESS || code >= DMA_ERR_CODE_NUM)
		return "Invalid Error Code";

	if (baikal_dma_err_info[0] == 0)
		baikal_dma_init_err_info(0);

	return baikal_dma_err_info[code];
}

baikal_dmareturn_t baikal_dma_desc_submit(struct baikal_dma_chan *edma_chan, struct baikal_dma_desc *desc)
{
	unsigned long		flags;

	spin_lock_irqsave(&edma_chan->lock, flags);

	baikal_dma_chan_dump_lli(&desc->lli);

	list_add_tail(&desc->desc_node, &edma_chan->active_list);

	edma_chan->ll_elements += 1;

	spin_unlock_irqrestore(&edma_chan->lock, flags);

	return DMA_SUCCESS;
}

static void baikal_dma_off(struct baikal_dma *dmac)
{
	int i;

	/* Disable read/write */
	dma_writel(dmac, write_enb.dword, FALSE);
	dma_writel(dmac, read_enb.dword, FALSE);

	for (i = 0; i < dmac->dma.chancnt; i++)
		dmac->chan[i].initialized = false;
}

static void baikal_dma_on(struct baikal_dma *dmac)
{
	/* Enable read/write */
	dma_writel(dmac, write_enb.dword, TRUE);
	dma_writel(dmac, read_enb.dword, TRUE);

	baikal_dma_get_chan_num(dmac, &dmac->write_chans, &dmac->read_chans);
}

/* Helper function to request a write channel via dma_request_channel() */
bool baikal_dma_wr_chan_filter(struct dma_chan *chan, void *param)
{
	if (chan && chan->private == param)
		return to_baikal_dma_chan(chan)->dirc == DMA_WRITE;
	else 
		return false;
}
EXPORT_SYMBOL_GPL(baikal_dma_wr_chan_filter);

/* Helper function to request a read channel via dma_request_channel() */
bool baikal_dma_rd_chan_filter(struct dma_chan *chan, void *param)
{
	if (chan && chan->private == param)
		return to_baikal_dma_chan(chan)->dirc == DMA_READ;
	else
		return false;
}
EXPORT_SYMBOL_GPL(baikal_dma_rd_chan_filter);

void baikal_dma_chan_set_callback(struct baikal_dma_chan *chan, void (*callback) (void *))
{
	chan->callback = callback;
}
EXPORT_SYMBOL_GPL(baikal_dma_chan_set_callback);

void baikal_dma_chan_reset_callback(struct baikal_dma_chan *chan)
{
	chan->callback = NULL;
}
EXPORT_SYMBOL_GPL(baikal_dma_chan_reset_callback);

int baikal_dma_probe(struct baikal_dma_chip *chip, struct baikal_dma_platform_data *pdata)
{
	struct baikal_dma	*dmac;
	int					err;
	int					i;

	dmac = devm_kzalloc(chip->dev, sizeof(*dmac), GFP_KERNEL);
	if (!dmac)
		return -ENOMEM;

	dmac->regs = chip->regs;
	chip->dmac = dmac;

	pm_runtime_get_sync(chip->dev);

	dmac->chan = devm_kcalloc(chip->dev, BAIKAL_EDMA_TOTAL_CHANNELS, sizeof(*dmac->chan),
			GFP_KERNEL);
	if (!dmac->chan) {
		err = -ENOMEM;
		goto err_pdata;
	}

	/* Calculate all channel mask before DMA setup */
	dmac->all_chan_mask = (1 << (BAIKAL_EDMA_TOTAL_CHANNELS)) - 1;

	/* Create a pool of consistent memory blocks for hardware descriptors */
	dmac->desc_pool = dmam_pool_create("baikal_dmac_desc_pool", chip->dev,
			sizeof(struct baikal_dma_desc), 4, 0);
	if (!dmac->desc_pool) {
		dev_err(chip->dev, "No memory for descriptors dma pool\n");
		err = -ENOMEM;
		goto err_pdata;
	}

	dmac->write_chans = pdata->wr_channels;
	dmac->read_chans = pdata->rd_channels;

	INIT_LIST_HEAD(&dmac->dma.channels);

	for (i = 0; i < pdata->wr_channels + pdata->rd_channels; i++) {

		struct baikal_dma_chan *edma_chan = &dmac->chan[i];

		edma_chan->chan.device = &dmac->dma;
		edma_chan->chan.private = chip;

		baikal_dma_chan_reset_callback(edma_chan);

		list_add(&edma_chan->chan.device_node, &dmac->dma.channels);

		edma_chan->llv_addr = KSEG1ADDR((uint32_t*)kmalloc(sizeof(struct baikal_dma_lli) * MAX_LLELEMENT_NUM, GFP_KERNEL | GFP_DMA));
		memset(edma_chan->llp_addr, 0, sizeof(struct baikal_dma_lli) * MAX_LLELEMENT_NUM);
		edma_chan->llp_addr = (uint32_t*)CPHYSADDR(edma_chan->llv_addr);

		edma_chan->ch_regs = &(((baikal_dma_regs *)(dmac->regs))->ctx_regs);

		spin_lock_init(&edma_chan->lock);

		INIT_LIST_HEAD(&edma_chan->active_list);
		INIT_LIST_HEAD(&edma_chan->queue);

		/* consider channels 0...3 as write channels, 4...7 as read channels */
		if (i < BAIKAL_EDMA_WR_CHANNELS) {
			edma_chan->dirc = DMA_WRITE;
			edma_chan->id = i;
		} else {
			edma_chan->dirc = DMA_READ;
			edma_chan->id = i - dmac->write_chans;
		}

		edma_chan->mask = 0x10001 << edma_chan->id;
		edma_chan->time = 0;
		edma_chan->nollp = TRUE;
		edma_chan->block_size = MIN_BLOCK_SIZE;
		edma_chan->irq = chip->irq[i]->start;

		err = request_irq(edma_chan->irq, baikal_dma_interrupt, 0,
				chip->irq[i]->name, dmac);
		if (err)
			goto err_dma_register;
	}

	/* Set capabilities */
	dmac->dma.dev = chip->dev;

	dmac->dma.device_alloc_chan_resources = baikal_dma_chan_alloc_chan_resources;
	dmac->dma.device_free_chan_resources = baikal_dma_chan_free_chan_resources;
	//dmac->dma.device_resume = baikal_dma_chan_resume;
	dmac->dma.device_tx_status = baikal_dma_chan_tx_status;
	dmac->dma.device_issue_pending = baikal_dma_chan_issue_pending;

	/* DMA capabilities */
	dmac->dma.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	dmac->dma.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;

	spin_lock_init(&dmac->lock);

	err = dma_async_device_register(&dmac->dma);
	if (err)
		goto err_dma_register;

	dev_info(chip->dev, "Baikal PCI DMA Controller: %d write channels, %d read channels\n",
			pdata->wr_channels, pdata->rd_channels);

	pm_runtime_put_sync_suspend(chip->dev);

	return 0;

err_dma_register:
	for (i = 0; i < dmac->dma.chancnt; i++) {
		free_irq(chip->irq[i]->start, dmac);
	}

err_pdata:
	pm_runtime_put_sync_suspend(chip->dev);
	return err;
}
EXPORT_SYMBOL_GPL(baikal_dma_probe);

int baikal_dma_remove(struct baikal_dma_chip *chip)
{
	struct baikal_dma		*dmac = chip->dmac;

	pm_runtime_get_sync(chip->dev);

	baikal_dma_off(dmac);
	dma_async_device_unregister(&dmac->dma);

	for (i = 0; i < dmac->dma.chancnt; i++) {
		free_irq(chip->irq[i]->start, dmac);
	}

	pm_runtime_put_sync_suspend(chip->dev);
	return 0;
}
EXPORT_SYMBOL_GPL(baikal_dma_remove);

int baikal_dma_disable(struct baikal_dma_chip *chip)
{
	struct baikal_dma *dmac = chip->dmac;
	baikal_dma_off(dmac);
	return 0;
}
EXPORT_SYMBOL_GPL(baikal_dma_disable);

int baikal_dma_enable(struct baikal_dma_chip *chip)
{
	struct baikal_dma *dmac = chip->dmac;
	baikal_dma_on(dmac);
	return 0;
}
EXPORT_SYMBOL_GPL(baikal_dma_enable);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Baikal PCIe eDMA Controller core driver");
MODULE_AUTHOR("Pavel Parkhomenko");
