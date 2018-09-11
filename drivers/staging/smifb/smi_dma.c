/*
 * DMA support for SMI drm driver.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/freezer.h>
#include <linux/kthread.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>

#include <linux/dma/baikal.h>
#include <uapi/video/smifb.h>

#include "smi_drv.h"

#define smfb_err(smi_fb, ...) dev_err(smi_fb->base.dev->dev, __VA_ARGS__)

struct {
	struct baikal_dma_chan *chan;
	struct smi_framebuffer *smi_fb;
} chan_to_smi[4];

static void dma_done_callback(void *arg)
{
	struct baikal_dma_chan *chan = (struct baikal_dma_chan *)arg;
	struct smi_framebuffer *smi_fb;
	int i;

	for (i = 0; i < 4; i++) {
		if (chan_to_smi[i].chan == chan) {
			smi_fb = chan_to_smi[i].smi_fb;
			wake_up(&smi_fb->wait_qh);
			break;
		}
	}
}

static void add_chan_map(struct smi_framebuffer *smi_fb, struct baikal_dma_chan *chan)
{
	int i;
	for (i = 0; i < 4; i++) {
		if (chan_to_smi[i].chan == NULL) {
			chan_to_smi[i].chan = chan;
			chan_to_smi[i].smi_fb = smi_fb;
			baikal_dma_chan_set_callback(chan, dma_done_callback);
			break;
		}
	}
}

static void remove_chan_map(struct smi_framebuffer *smi_fb)
{
	int i;
	for (i = 0; i < 4; i++) {
		if (chan_to_smi[i].smi_fb == smi_fb) {
			baikal_dma_chan_reset_callback(chan_to_smi[i].chan);
			chan_to_smi[i].chan = NULL;
			chan_to_smi[i].smi_fb = NULL;
		}
	}
}

static void smi_dma_start_xfer(struct smi_framebuffer *smi_fb)
{
	void *kmap_va;
	unsigned long size = smi_fb->end_off - smi_fb->start_off;
	dma_addr_t dst, src;
	struct dma_chan *chan = smi_fb->dma_chan;
	unsigned long start_page, num_pages, start_off;
	struct smi_bo *src_bo, *dst_bo;
	int ret;

	src_bo = gem_to_smi_bo(smi_fb->obj);
	dst_bo = smi_fb->vram_bo;

	start_page = smi_fb->start_off >> PAGE_SHIFT;
	num_pages = ((smi_fb->end_off - 1) >> PAGE_SHIFT) - start_page + 1;
	start_off = smi_fb->start_off & (PAGE_SIZE - 1);

	src = PFN_PHYS(page_to_pfn(src_bo->bo.ttm->pages[start_page])) + start_off;
	dst = dst_bo->bo.mem.bus.base + dst_bo->bo.mem.bus.offset +
		smi_fb->start_off;
	smi_fb->start_off = smi_fb->end_off = 0;
	mutex_unlock(&smi_fb->mutex); // unlock early to allow user side updating start_off/end_off

	ret = ttm_bo_kmap(&src_bo->bo, start_page, num_pages, &src_bo->kmap);
	if (ret) {
		smfb_err(smi_fb, "ttm_bo_kmap %lx:%lx error %d\n", start_page, num_pages, ret);
		return;
	}
	kmap_va = src_bo->kmap.virtual;

	dma_cache_sync(NULL, kmap_va + start_off, size, DMA_TO_DEVICE);
	ttm_bo_kunmap(&src_bo->kmap);

#if 0 /* async DMA API */
	tx = dev->device_prep_dma_memcpy(chan, dst, src, size, 0);
	if (!tx) {
		smfb_err(smi_fb, "Can't prep tx desc\n");
	} else {
		cookie = tx->tx_submit(tx);
		if (dma_submit_error(cookie))
			smfb_err(smi_fb, "Submit error\n");
	}
	dma_async_issue_pending(chan);
	smi_fb->cookie = cookie;
#else /* Baikal DMA */
	phyaddr p_src, p_dst;

	p_src.quad_part = src;
	p_dst.quad_part = dst;
	baikal_dma_transfer_prepare_single((struct baikal_dma_chan *)chan,
					   p_src, p_dst, size);
	baikal_dma_transfer_start((struct baikal_dma_chan *)chan);
#endif
	mutex_lock(&smi_fb->mutex);
}

static int smi_syncer_thread(void *arg)
{
	struct smi_framebuffer *smi_fb = arg;
	DEFINE_WAIT(wait);

	while (!kthread_should_stop()) {
		mutex_lock(&smi_fb->mutex);
#if 0 /* async DMA */
		status = dma_async_is_tx_complete(smi_fb->dma_chan,
						  smi_fb->cookie, NULL, NULL);
		if (status != DMA_IN_PROGRESS && smi_fb->end_off)
			smi_dma_start_xfer(smi_fb);
#else /* Baikal DMA */
		baikal_dma_chan_status bc_stat;
		if (smi_fb->end_off) {
			bc_stat = baikal_dma_chan_get_status((struct baikal_dma_chan *)smi_fb->dma_chan);
			if (bc_stat != CHAN_RUNNING) {
				smi_dma_start_xfer(smi_fb);
}
		}
#endif

		mutex_unlock(&smi_fb->mutex);
		wait_event_freezable_timeout(smi_fb->wait_qh,
					     smi_fb->end_off, HZ/10);
	}
	return 0;
}

static char smi_dma_dev[32] = "baikal-edma";
//module_param_string(dmadev, smi_dma_dev, sizeof(smi_dma_dev), 0);
//MODULE_PARM_DESC(dmadev, "DMA device name");

static bool smi_dma_filter(struct dma_chan *chan, void *param)
{
	char *dmadev = param;
	if (strncmp(chan->device->dev->driver->name, dmadev, strlen(dmadev)))
		return false;
	return baikal_dma_wr_chan_filter(chan, chan->private);
}

/* Prepare DMA stuff for new framebuffer - allocate channel etc. */
int smi_setup_dma(struct smi_framebuffer *smi_fb)
{
	dma_cap_mask_t mask;
	struct dma_chan *dma_chan;
	int rc;

	if (smi_fb->has_dma) {
		pr_err("smi_setup_dma: DMA already set up\n");
		return 0;
	}
	dma_cap_zero(mask);

	dma_chan = dma_request_channel(mask, smi_dma_filter, smi_dma_dev);
	if (!dma_chan)
		return -ENODEV;

	smi_fb->dma_chan = dma_chan;
	mutex_init(&smi_fb->mutex);
	init_waitqueue_head(&smi_fb->wait_qh);
	if (smi_fb->is_user) {
		smi_fb->syncer_thread = kthread_run(smi_syncer_thread, smi_fb, "smi_syncd");
		if (IS_ERR(smi_fb->syncer_thread)) {
			smfb_err(smi_fb, "Can't start syncer thread\n");
			rc = PTR_ERR(smi_fb->syncer_thread);
			smi_fb->syncer_thread = NULL;
			dma_release_channel(smi_fb->dma_chan);
			smi_fb->dma_chan = NULL;
			return rc;
		}
		add_chan_map(smi_fb, (struct baikal_dma_chan *)dma_chan);
	}
	smi_fb->has_dma = true;

	return 0;
}

void smi_stop_dma(struct smi_framebuffer *smi_fb)
{
	if (!smi_fb->has_dma)
		return;
	if (smi_fb->syncer_thread) {
		kthread_stop(smi_fb->syncer_thread);
		remove_chan_map(smi_fb);
	}
	dma_release_channel(smi_fb->dma_chan);
	smi_fb->has_dma = 0;
}

static int sm_check_dma_status(struct fb_info *info, uint32_t *status)
{
	struct smi_fbdev *par = info->par;
	struct smi_framebuffer *smi_fb = &par->gfb;
	baikal_dma_chan_status bc_stat;

	bc_stat = baikal_dma_chan_get_status((struct baikal_dma_chan *)smi_fb->dma_chan);
	*status = (bc_stat == CHAN_RUNNING);
	return 0;
}

static int sm_dma_start(struct fb_info *info, phys_addr_t src, phys_addr_t dst, uint32_t size)
{
	struct smi_fbdev *par = info->par;
	struct smi_framebuffer *smi_fb = &par->gfb;
	phyaddr p_src, p_dst;

	p_src.quad_part = src;
	p_dst.quad_part = dst;
	baikal_dma_transfer_prepare_single((struct baikal_dma_chan *)smi_fb->dma_chan, p_src, p_dst, size);
	baikal_dma_transfer_start((struct baikal_dma_chan *)smi_fb->dma_chan);
	return 0;
}

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
	struct smi_fbdev *par = info->par;
	struct smi_framebuffer *smi_fb = &par->gfb;
	uint32_t status;
	int rc;

	if (!smi_fb->has_dma) {
		rc = smi_setup_dma(smi_fb);
		if (rc && (cmd == FBIO_DW_GET_STAT_DMA_TRANSFER ||
			   cmd == FBIO_DW_DMA_WRITE))
			return rc;
	}

	switch (cmd) {
	case FBIO_DW_GET_STAT_DMA_TRANSFER:
		{
			u32 __user *argp = (u32 __user *) arg;
			rc = sm_check_dma_status(info, &status);
			if (copy_to_user(argp, &status, sizeof(uint32_t)))
	                        return -EFAULT;
			return rc;
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

	err = sm_dma_start(info, pbuf /* MEM */, dst /* PCI */, dma_size /*size*/);

	if  (!err)
		*ppos += count;
	return (err) ? err : count;
}
