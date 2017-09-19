/*
 * Baikal-T SOC platform support code. EFUSE driver.
 *
 * Copyright (C) 2014-2016 Baikal Electronics JSC
 * 
 * Author:
 *   Georgiy Vlasov <Georgy.Vlasov@baikalelectronics.ru>
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

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <asm/io.h>
#include "efuse.h"

#define VERSION	"1.00"

#define DEBUG_EFUSE FALSE
#define STATIC_PART_OF_MAC 0x4ca515 /* CHECKIT this filed can change in the next revisions*/

/* Current EFuse format */
typedef struct
{
	/* Field Name */   
	u32 Locks       ; 
	 u8 Version     ; 
	 u8 Fab     : 4 ; 
	 u8 Process : 4 ; 
	 u8 LotId       ; 
	 u8 Revision    ; 
	u32 SerialNum   ; 
	u32 CornerId: 4 ; 
	u32 CPUFreq : 4 ; 
	u32 Pad:24      ; 
	u32 Reserved[28]; 
}EFUSE_Structure;

typedef struct
{
	struct  device *dev;
	EFUSE_Structure *EFUSE_Format;
	void    __iomem *efuse;
	dev_t   first;                         /* Variable for the first device number        */
	struct  cdev c_dev;                    /* Variable for the character device structure */
	struct  class *cl;                     /* Variable for the device class               */
	int     is_device_open;
}be_efuse;

be_efuse *be_apb_efuse;

/* API for other kernel space progs */

u32 be_efuse_getLocks(void)
{
	return be_apb_efuse->EFUSE_Format->Locks;
}

u8 be_efuse_getVersion(void)
{
	return be_apb_efuse->EFUSE_Format->Version;
}

u8 be_efuse_getFab(void)
{
	return be_apb_efuse->EFUSE_Format->Fab;
}

u8 be_efuse_getProcess(void)
{
	return be_apb_efuse->EFUSE_Format->Process;
}

u8 be_efuse_getLotID(void)
{
	return be_apb_efuse->EFUSE_Format->LotId;
}

u8 be_efuse_getRevision(void)
{
	return be_apb_efuse->EFUSE_Format->Revision;
}

u32 be_efuse_getSerialNum(void)
{
	return be_apb_efuse->EFUSE_Format->SerialNum;
}

u32 be_efuse_getCornerID(void)
{
	return be_apb_efuse->EFUSE_Format->CornerId;
}

u32 be_efuse_getCPUFreq(void)
{
	return be_apb_efuse->EFUSE_Format->CPUFreq;
}

u32 be_efuse_getPad(void)
{
	return be_apb_efuse->EFUSE_Format->Pad;
}

u64 be_efuse_getMAC(u8 id)
{ 
	u64 ret;
	u32 devId = be_apb_efuse->EFUSE_Format->SerialNum << 2;
   	devId |= (id & 0x3);
   	ret = ((u64)(be_apb_efuse->EFUSE_Format->Revision) & 0xFFFFFF) << 24;
   	ret = STATIC_PART_OF_MAC;
   	ret = ret << 24;
   	return  (ret | (devId & 0xFFFFFF)); 
}

void	get_efuse(u32 *Raw)
{
	be_apb_efuse->EFUSE_Format->Locks = *Raw;
#ifdef DEBUG_EFUSE
	printk(KERN_INFO "Locks field = %x\n",be_apb_efuse->EFUSE_Format->Locks);
#endif

	be_apb_efuse->EFUSE_Format->Version = *(Raw + 1) >> 24;
#ifdef DEBUG_EFUSE
	printk(KERN_INFO "Version field = %x\n",be_apb_efuse->EFUSE_Format->Version);
#endif

	be_apb_efuse->EFUSE_Format->Fab= ((*(Raw + 1) >> 16) & 0xF0) >> 4;
#ifdef DEBUG_EFUSE 
	printk(KERN_INFO "Fab field = %x\n",be_apb_efuse->EFUSE_Format->Fab);
#endif

	be_apb_efuse->EFUSE_Format->Process = (*(Raw + 1) >> 16) & 0x0F;
#ifdef DEBUG_EFUSE
	printk(KERN_INFO "Process field = %x\n",be_apb_efuse->EFUSE_Format->Fab);
#endif

	be_apb_efuse->EFUSE_Format->LotId = *(Raw + 1) >> 8;
#ifdef DEBUG_EFUSE
	printk(KERN_INFO "Lotid field = %x\n",be_apb_efuse->EFUSE_Format->LotId);
#endif

	be_apb_efuse->EFUSE_Format->Revision = *(Raw + 1);
#ifdef DEBUG_EFUSE
	printk(KERN_INFO "Revision field = %x\n",be_apb_efuse->EFUSE_Format->Revision);
#endif

	be_apb_efuse->EFUSE_Format->SerialNum = *(Raw + 2);
#ifdef DEBUG_EFUSE
	printk(KERN_INFO "Serial field = %x\n",be_apb_efuse->EFUSE_Format->SerialNum);
#endif

	be_apb_efuse->EFUSE_Format->CornerId = (*(Raw + 3) >> 28);
#ifdef DEBUG_EFUSE
	printk(KERN_INFO "Corner field = %x\n",be_apb_efuse->EFUSE_Format->CornerId);
#endif

	be_apb_efuse->EFUSE_Format->CPUFreq = (*(Raw + 3) >> 24) & 0x0F;
#ifdef DEBUG_EFUSE
	printk(KERN_INFO "CpuFrequency field = %x\n",be_apb_efuse->EFUSE_Format->CPUFreq);
#endif

	be_apb_efuse->EFUSE_Format->Pad = (*(Raw + 3)) & 0x00FFFFFF;
#ifdef DEBUG_EFUSE
	printk(KERN_INFO "Pad field = %x\n",be_apb_efuse->EFUSE_Format->Pad);
#endif
}

int read_EFUSE(void)
{
	/* dump from efuse to stucture RAW */
	u32 *Raw;
	int i = 0;
	u32 reg;
	u32 addr;
	Raw = (u32 *) kmalloc(sizeof(u32) * 32,GFP_KERNEL);
		if(Raw == NULL) {
	 		printk (KERN_INFO "EFUSE dump allocation failure.");
	 		return -ENOMEM;
		}
	addr = 0xFFFFFFE0;
	while ((addr <= 0xFFFFFFFF) && (addr > 0xFFFFFF00))    /* Second condition for protection against looping */
	{
		/* 1)writing addr of string which we will read */
		reg = 0xFFFFFFFF;
		reg &= addr; //reading from zero addr
		iowrite32(reg, (u32 *)(be_apb_efuse->efuse) + EFUSE_ADDR/4); /* push data to the register */
		/* 2)set read mode*/
		reg = ioread32((u32 *)(be_apb_efuse->efuse) + EFUSE_MODES/4); /* pull register */
		reg |= (1<<0); /* set 0 bit in 1 */
		reg &= ~(1<<1); /* set 1 bit in 0 , it's a read mode */
		iowrite32(reg, (u32 *)(be_apb_efuse->efuse) + EFUSE_MODES/4); /* push data into the register */
		/* 3)set enable reg */
		reg = ioread32((u32 *)(be_apb_efuse->efuse) + EFUSE_ENABLE/4); 
		reg |= (1<<0); 
		iowrite32(reg, (u32 *)(be_apb_efuse->efuse) + EFUSE_ENABLE/4);
		/* 4)delay for waiting preadu signal */
		udelay(2);
		/* 5)set power down mode */
		reg = ioread32((u32 *)(be_apb_efuse->efuse) + EFUSE_ENABLE/4);
		reg &= ~(1<<0);
		iowrite32(reg, (u32 *)(be_apb_efuse->efuse) + EFUSE_ENABLE/4);
		/* 6)pull from reg efuse_rdata */
		Raw[i] = ioread32((u32 *)(be_apb_efuse->efuse) + EFUSE_RDATA/4);
		i++;
		addr++;
	}
	/* 7)close work session with efuse */
	reg = ioread32((u32 *)(be_apb_efuse->efuse) + EFUSE_MODES/4);
	reg &= ~(1<<0); /* set power down mode */
	iowrite32(reg, (u32 *)(be_apb_efuse->efuse) + EFUSE_MODES/4);
	/* set enable */
	reg = ioread32((u32 *)(be_apb_efuse->efuse) + EFUSE_ENABLE/4);
	reg |= (1<<0);
	iowrite32(reg, (u32 *)(be_apb_efuse->efuse) + EFUSE_ENABLE/4);
	udelay(2);
	/* set power down mode */
	reg = ioread32((u32 *)(be_apb_efuse->efuse) + EFUSE_ENABLE/4);
	reg &= ~(1<<0);
	iowrite32(reg, (u32 *)(be_apb_efuse->efuse) + EFUSE_ENABLE/4);
	/* parse dump from efuse */
	get_efuse(Raw);
	kfree(Raw);
	return 0;
}

static int baikal_efuse_open(struct inode *i, struct file *f)
{
	if (be_apb_efuse->is_device_open)
		return -EBUSY;

	be_apb_efuse->is_device_open++;
#ifdef DEBUG_EFUSE
	printk(KERN_DEBUG "baikal_efuse_driver has been opened");
#endif
	return 0;
}

static int baikal_efuse_close(struct inode *i, struct file *f)
{
	be_apb_efuse->is_device_open--;
#ifdef DEBUG_EFUSE
	printk(KERN_DEBUG "baikal_efuse_driver has been closed\n");
#endif
	return 0;
}

static ssize_t baikal_efuse_read(struct file *f, char __user *buf, size_t
  len, loff_t *off)
{
#ifdef DEBUG_EFUSE
	u64 res;
	printk(KERN_DEBUG "baikal_efuse_driver read function has been used\n");
	res = be_efuse_getMAC(Gb_ETHERNET_0);
	printk(KERN_DEBUG "MAC0 for 1Gb ETH # 0=%llx", res);
	res = be_efuse_getMAC(Gb_ETHERNET_1);
	printk(KERN_DEBUG "MAC1 for 1Gb ETH # 1=%llx \n",res);
	res = be_efuse_getMAC(xGb_ETHERNET);
	printk(KERN_DEBUG "MAC2 for 10 Gb ETH=%llx \n",res);
#endif	
	return 0;
}


static struct file_operations efuse_driver_fops =
{
	.owner = THIS_MODULE,
	.open = baikal_efuse_open,
	.release = baikal_efuse_close,
	.read = baikal_efuse_read
};

static int be_efuse_probe(struct platform_device *pdev)
{
	struct resource *res;

	be_apb_efuse = devm_kzalloc(&pdev->dev, sizeof(*be_apb_efuse), GFP_KERNEL);
	if (!be_apb_efuse)
		return -ENOMEM;

	be_apb_efuse->is_device_open = 0; /* init dev_open */
	be_apb_efuse->dev = &pdev->dev;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	be_apb_efuse->efuse = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(be_apb_efuse->efuse))
		return PTR_ERR(be_apb_efuse->efuse);

	dev_info(&pdev->dev, "Baikal Efuse Driver\n");
	dev_info(&pdev->dev, "Version " VERSION "\n");

	if (alloc_chrdev_region(&(be_apb_efuse->first), 0, 1, "baikal_efuse_driver") < 0) /* register number of efuse device */ 
	{
    		return -1;
	}

	if ((be_apb_efuse->cl = class_create(THIS_MODULE, "efuse")) == NULL) /* create device class */
  	{
    		unregister_chrdev_region(be_apb_efuse->first, 1);
    		return -1;
	}

	if (device_create(be_apb_efuse->cl, NULL, be_apb_efuse->first, NULL, "efuse_driver") == NULL) /* create device with name efuse_driver */
	{
    		class_destroy(be_apb_efuse->cl);
    		unregister_chrdev_region(be_apb_efuse->first, 1);
    		return -1;
	}

	cdev_init(&(be_apb_efuse->c_dev), &efuse_driver_fops);
	if (cdev_add(&(be_apb_efuse->c_dev), be_apb_efuse->first, 1) == -1)
	{
    		device_destroy(be_apb_efuse->cl, be_apb_efuse->first);
    		class_destroy(be_apb_efuse->cl);
    		unregister_chrdev_region(be_apb_efuse->first, 1);
    		return -1;
	}

#ifdef DEBUG_EFUSE
	printk(KERN_DEBUG "<Major, Minor>: <%d, %d>\n", MAJOR(be_apb_efuse->first), MINOR(be_apb_efuse->first));
#endif

	be_apb_efuse->EFUSE_Format = (EFUSE_Structure *) kmalloc(sizeof(EFUSE_Structure),GFP_KERNEL);
	if(be_apb_efuse->EFUSE_Format == NULL) 
		{
	 		printk (KERN_INFO "EFUSE structure allocation failure.");
	 		return -ENOMEM;
		}

	if (read_EFUSE() != 0) /* read all efuse memory to the dump sructure and parse it */
		{
			printk (KERN_INFO "EFUSE read procedure failure");
	 		return -1;
		}

	printk(KERN_INFO "baikal_efuse_driver has been loaded\n");
	return 0;
}

static int be_efuse_remove(struct platform_device *pdev)
{
	cdev_del(&(be_apb_efuse->c_dev));
	device_destroy(be_apb_efuse->cl, be_apb_efuse->first);
	class_destroy(be_apb_efuse->cl);
	unregister_chrdev_region(be_apb_efuse->first, 1);
	iounmap(be_apb_efuse->efuse);
	kfree(be_apb_efuse->EFUSE_Format);
	printk(KERN_INFO "baikal_efuse_driver has been unloaded\n");
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id be_apb_of_match[] = {
	{ .compatible = "baikal,efuse", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, be_apb_of_match);
#endif

static struct platform_driver be_efuse_driver = {
	.probe		= be_efuse_probe,
	.remove		= be_efuse_remove,
	.driver		= {
		.name	= "baikal_efuse",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(be_apb_of_match),
#endif /* CONFIG_OF */
	},
};

module_platform_driver(be_efuse_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Georgiy Vlasov <Georgy.Vlasov@baikalelectronics.ru>");
MODULE_DESCRIPTION("baikal_efuse_driver");
