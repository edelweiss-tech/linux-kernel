#ifdef CONFIG_SM750_DMA
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include "sm750_ioctl.h"

void DWGetChanWrStatus(u8 chan, u32 *status);
int DWDmaSingleBlockWrite(u8 chan, void *src, void *dst,
			  u32 size, u8 wei, int queue);

#define DMA_WRITE_CHAN		2

//#define CHECK_DELTA
//#define CHECK_STATUS

static long get_pfn(unsigned long uaddr, unsigned long access,
		    unsigned long *pfn)
{
	struct vm_area_struct *vma = NULL;
	long ret = -EINVAL;

	*pfn = 0;

	down_read(&current->mm->mmap_sem);

	vma = find_vma(current->mm, uaddr);
	if (!vma)
		goto out;

	if (vma->vm_flags & access)
		ret = follow_pfn(vma, uaddr, pfn);
out:
	up_read(&current->mm->mmap_sem);

	return ret;
}

ssize_t dw_fb_write(struct fb_info *info, const char __user *buf,
		    size_t count, loff_t *ppos);

int dw_fb_ioctl(struct fb_info *info, u_int cmd, u_long arg)
{
	u32 status;

	switch (cmd) {
	case FBIO_DW_GET_STAT_DMA_TRANSFER:
		{
		u32 __user *argp = (u32 __user *)arg;

		DWGetChanWrStatus(DMA_WRITE_CHAN /*chan*/, &status);
		if (copy_to_user(argp, &status, sizeof(u32)))
			return -EFAULT;
		return 0;
		}
	case FBIO_DW_DMA_WRITE:
		{
		fb_dma_req_t __user *argp = (fb_dma_req_t __user *)arg;

		return dw_fb_write(info, argp->from, argp->size,
				   &argp->fb_off);
		}
	default:
		return -EINVAL;
	}
}

ssize_t dw_fb_write(struct fb_info *info, const char __user *buf,
		    size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	void *dst;
	int err = 0;
	unsigned long pfn;
	void *pbuf;
	unsigned long total_size;
	unsigned long dma_size;
	u16 lead = 0, tail = 0;
#ifdef CHECK_STATUS
	int i;
#endif /* CHECK_STATUS */
#if defined(CHECK_DELTA) || defined(CHECK_STATUS)
	u32 status;
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

	dst = (void __force *)(info->screen_base + p);

	if (info->fbops->fb_sync)
		info->fbops->fb_sync(info);

	dma_cache_wback_inv((u32)buf, count);

	/* Odd address = can't DMA. Align */
	if ((unsigned long)dst & 3) {
		lead = 4 - ((unsigned long)dst & 3);
		if (copy_from_user(dst, buf, lead))
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
		pbuf = (void *)(((pfn << PAGE_SHIFT) | (((u32)buf)
				& ~PAGE_MASK)));
	} else {
		pr_err("%s: cannot get pfn\n", __func__);
		return -EFAULT;
	}

#ifdef CHECK_DELTA
	pr_info("%s: dma_size = %lx buf=0x%p pbuf=0x%p dst=0x%p\n", __func__,
		dma_size, buf, pbuf, dst);
	jiffies1 = read_c0_count();
#endif /* CHECK_DELTA */

	err = DWDmaSingleBlockWrite(DMA_WRITE_CHAN /*chan*/,
				    (void *)pbuf /* MEM */,
				    (void *)CPHYSADDR((u32)dst) /* PCI */,
				    dma_size /*size*/, 0 /*wei*/, 0 /*queue*/);

#ifdef CHECK_STATUS
	for (i = 0; i < 100000; i++) {
		DWGetChanWrStatus(DMA_WRITE_CHAN /*chan*/, &status);
		if (status == DW_PCI_DMA_DONE)
			break;
	}
#endif /* CHECK_STATUS */

#ifdef CHECK_DELTA
	jiffies2 = read_c0_count();
	pr_info("%s: Status read = %d delta = %ld, size = 0x%lx,\n", __func__,
		status, (jiffies2 - jiffies1), dma_size);
#endif /* CHECK_DELTA */

	if  (!err)
		*ppos += count;
	return (err) ? err : count;
}
#endif/*CONFIG_SM750_DMA*/
