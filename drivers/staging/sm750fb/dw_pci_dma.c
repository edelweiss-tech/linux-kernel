#ifdef CONFIG_SM750_DMA
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include "sm750.h"
#include "sm750_ioctl.h"
#include <linux/dma/baikal.h>

#ifdef SMI_DMA_ASYNC	/* Use standard DMA API - async MEMCPY */
#undef SMI_DMA_ASYNC
#endif

static char smi_dma_dev[32] = "baikal-edma";

#ifdef SMI_DMA_ASYNC /* TODO: allow alternative compatible DMA */
module_param_string(dmadev, smi_dma_dev, sizeof(smi_dma_dev), 0);
MODULE_PARM_DESC(dmadev, "DMA device name");
#endif

// bool baikal_dma_wr_chan_filter(struct dma_chan *chan, void *param);

static bool smi_dma_filter(struct dma_chan *chan, void *param)
{
	char *dmadev = param;

	pr_debug("smi_dma_filter: name %s\n", chan->device->dev->driver->name);
	if (strncmp(chan->device->dev->driver->name, dmadev, strlen(dmadev)))
		return false;
	return baikal_dma_wr_chan_filter(chan, chan->private);
}

/* Prepare DMA stuff for new framebuffer - allocate channel etc. */
int smi_setup_dma(struct sm750_dev *sm750_dev)
{
	dma_cap_mask_t mask;
	struct dma_chan *dma_chan;

	dma_cap_zero(mask);
#ifdef SMI_DMA_ASYNC
	dma_cap_set(DMA_MEMCPY, mask);
#endif

	dma_chan = dma_request_channel(mask, smi_dma_filter, smi_dma_dev);
	if (!dma_chan)
		return -ENODEV;
	sm750_dev->dma_chan = dma_chan;

	return 0;
}

void smi_release_dma(struct sm750_dev *sm750_dev)
{
	dma_release_channel(sm750_dev->dma_chan);
	sm750_dev->dma_chan = NULL;
}

static void sm_check_dma_status(struct fb_info *info, uint32_t *status)
{
	struct lynxfb_par *par = info->par;
	struct sm750_dev *sm750_dev = par->dev;
#ifdef SMI_DMA_ASYNC
	enum dma_status dma_stat;

	dma_stat = dma_async_is_tx_complete(sm750_dev->dma_chan,
					  sm750_dev->cookie, NULL, NULL);
	*status = (dma_stat == DMA_IN_PROGRESS);
#else
	baikal_dma_chan_status bc_stat;
	bc_stat = baikal_dma_chan_get_status((struct baikal_dma_chan *)sm750_dev->dma_chan);
	*status = (bc_stat == CHAN_RUNNING);
#endif
}

static int sm_dma_start(struct fb_info *info, phys_addr_t src, phys_addr_t dst, uint32_t size)
{
	struct lynxfb_par *par = info->par;
	struct sm750_dev *sm750_dev = par->dev;
#ifdef SMI_DMA_ASYNC
	struct dma_async_tx_descriptor *tx;
	dma_cookie_t cookie = 0;
	struct dma_chan *chan = sm750_dev->dma_chan;
	struct dma_device *dev = chan->device;

	tx = dev->device_prep_dma_memcpy(chan, dst, src, size, 0);
	if (!tx) {
		pr_err("sm_dma_start: Can't prep tx desc\n");
		return -1;
	} else {
		cookie = tx->tx_submit(tx);
		if (dma_submit_error(cookie)) {
			pr_err("sm_dma_start: submit error\n");
			return -1;
		}
	}
	dma_async_issue_pending(chan);
	sm750_dev->cookie = cookie;
#else
	phyaddr p_src, p_dst;

	p_src.quad_part = src;
	p_dst.quad_part = dst;
	baikal_dma_transfer_prepare_single((struct baikal_dma_chan *)sm750_dev->dma_chan, p_src, p_dst, size);
	baikal_dma_transfer_start((struct baikal_dma_chan *)sm750_dev->dma_chan);
#endif
	return 0;
}

//#define CHECK_DELTA
//#define CHECK_STATUS

static long get_pfn(unsigned long uaddr, unsigned long access, unsigned long *pfn)
{
	struct vm_area_struct *vma = NULL;
	long ret = -EINVAL;

	*pfn = 0;

	down_read(&current->mm->mmap_sem);

	if((vma = find_vma(current->mm, uaddr)) == NULL) {
		goto out;
	}

	if (vma->vm_flags & access) {
		ret = follow_pfn(vma, uaddr, pfn);
	}
out: 
	up_read(&current->mm->mmap_sem);

	return ret;
}

ssize_t dw_fb_write(struct fb_info *info, const char __user *buf,
                    size_t count, loff_t *ppos);

int dw_fb_ioctl(struct fb_info *info, u_int cmd, u_long arg)
{
	uint32_t status;

	switch (cmd) {
		case FBIO_DW_GET_STAT_DMA_TRANSFER:
			{
			u32 __user *argp = (u32 __user *) arg;
			sm_check_dma_status(info, &status);
			if (copy_to_user(argp, &status, sizeof(uint32_t)))
	                        return -EFAULT;
			return 0;
			}
		case FBIO_DW_DMA_WRITE:
			{
			fb_dma_req_t __user *argp = (fb_dma_req_t __user *) arg;
			return dw_fb_write(info, argp->from, argp->size, &argp->fb_off);
			}
		default:
			return -EINVAL;
	}
}

ssize_t dw_fb_write(struct fb_info *info, const char __user *buf,
					size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	phys_addr_t dst;
	int err = 0;
	unsigned long pfn;
	phys_addr_t pbuf;
	unsigned long total_size;
	unsigned long dma_size;
	u16 lead = 0, tail = 0;
#ifdef CHECK_STATUS
	int i;
#endif /* CHECK_STATUS */
#if defined(CHECK_DELTA) || defined(CHECK_STATUS)
	uint32_t status;
#endif /* DELTA || STATUS */
#ifdef CHECK_DELTA
        volatile unsigned long jiffies1, jiffies2;
#endif /* CHECK_DELTA */

	if (info->state != FBINFO_STATE_RUNNING)
		return -EPERM;

	total_size = info->screen_size;

	if (total_size == 0)
		total_size = info->fix.smem_len;

	if (p > total_size)
		return -EFBIG;

	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}

	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;
		count = total_size - p;
	}

	dst = info->fix.smem_start + p;

	if (info->fbops->fb_sync)
		info->fbops->fb_sync(info);

	dma_cache_wback_inv((uint32_t)buf, count);

	/* Odd address = can't DMA. Align */
	if (dst & 3) {
		lead = 4 - (dst & 3);
		if (copy_from_user((char *)dst, buf, lead))
			return -EFAULT;
		buf += lead;
		dst += lead;
	}
	/* DMA resolution is 32 bits */
	if ((count - lead) & 3)
		tail = (count - lead) & 3;

	/* DMA the data */
	dma_size = count - lead - tail;

	/* Get physical address based on an user-virtual address. */
	if (get_pfn((unsigned long)buf, VM_WRITE, &pfn) == 0) {
		pbuf = (pfn << PAGE_SHIFT) | (((uint32_t)buf) & ~PAGE_MASK);
	} else {
		pr_err("%s: cannot get pfn\n", __FUNCTION__);
		return -EFAULT;
	}

#ifdef CHECK_DELTA
	printk("%s: dma_size = %lx buf=0x%p pbuf=0x%p dst=0x%p\n", __FUNCTION__, dma_size, buf, pbuf, dst);
	jiffies1 = read_c0_count();
#endif /* CHECK_DELTA */

	err = sm_dma_start(info, pbuf /* MEM */, dst /* PCI */, dma_size /*size*/);

#ifdef CHECK_STATUS
	for (i=0; i < 100000; i++) {
		sm_check_dma_status(info, &status);
		if (!status)
			break;
	}
#endif /* CHECK_STATUS */

#ifdef CHECK_DELTA
	jiffies2 = read_c0_count();
	printk("%s: Status read = %d delta = %ld, size = 0x%lx,\n", __FUNCTION__, status,
                (jiffies2 - jiffies1), dma_size);
#endif /* CHECK_DELTA */

#if 0
	/* Disable it, DW PCI allows size-unaligned transactions. */

	dst += dma_size;
	buf += dma_size;
	/* Copy any leftover data */
	if (tail && copy_from_user(dst, buf, tail))
		return -EFAULT;
#endif
	if  (!err)
		*ppos += count;
	return (err) ? err : count;
}
#endif /*CONFIG_SM750_DMA*/
