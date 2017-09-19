/*
 * Baikal-T SOC platform support code. APB Terminator driver.
 *
 * Copyright (C) 2014  Baikal Electronics OJSC
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

#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/sysfs.h>

#define VERSION	"1.02"

#define BE_APB_IRQ_CTRL		0x00
#define BE_APB_FAULT_ADDR	0x04
#define BE_APB_FAULT_TEST	0x10

#define BE_APB_IRQ_MASK		(1 << 1)
#define BE_APB_IRQ_PEND		(1 << 0)

struct be_apb {
	struct device *dev;
	void __iomem *regs;
	int	irq;
	unsigned int count;
	unsigned int addr;
};

#ifdef CONFIG_SYSFS
static ssize_t show_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct be_apb *apb = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", apb->count);
}
static DEVICE_ATTR(errors, S_IWUSR | S_IRUGO, show_count, NULL);

static ssize_t show_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct be_apb *apb = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%08x\n", apb->addr);
}
static DEVICE_ATTR(addr, S_IWUSR | S_IRUGO, show_addr, NULL);

static ssize_t show_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "Test APB exception\n");
}
static ssize_t store_test(struct device *dev, struct device_attribute *attr,
                 const char *buf, size_t count)
{
	struct be_apb *apb = dev_get_drvdata(dev);
	/* Dummy write */
	writel(0, apb->regs + BE_APB_FAULT_TEST);
	/* Never occurs */
	return count;
}
static DEVICE_ATTR(test, S_IWUSR | S_IRUGO, show_test, store_test);

static void be_apb_sysfs_init(struct device *dev)
{
	int ret;
	/* Errors count */
	ret = sysfs_create_file(&dev->kobj, &dev_attr_errors.attr);
	if (ret)
		return;
	/* Last error address */
	ret = sysfs_create_file(&dev->kobj, &dev_attr_addr.attr);
	if (ret)
		goto __err2;
	/* Test entry */
	ret = sysfs_create_file(&dev->kobj, &dev_attr_test.attr);
	if (ret)
		goto __err1;
	return;
__err1:
	sysfs_remove_file(&dev->kobj, &dev_attr_addr.attr);
__err2:
	sysfs_remove_file(&dev->kobj, &dev_attr_errors.attr);
}

static void be_apb_sysfs_remove(struct device *dev)
{
	sysfs_remove_file(&dev->kobj, &dev_attr_errors.attr);
	sysfs_remove_file(&dev->kobj, &dev_attr_addr.attr);
	sysfs_remove_file(&dev->kobj, &dev_attr_test.attr);
}
#else
static void be_apb_sysfs_init(struct device *dev) {}
static void be_apb_sysfs_remove(struct device *dev) {}
#endif

static irqreturn_t be_apb_irq(int irq, void *data)
{
	struct be_apb *apb = (struct be_apb *)data;
	/* Get fault address */
	apb->addr = readl(apb->regs + BE_APB_FAULT_ADDR);
	/* Alert */
	dev_crit_ratelimited(apb->dev,
		"Peripherial Bus IOMEM access error handled at %08x\n", apb->addr);
	/* Increase counter (in irq handler it is atomic) */
	apb->count += 1;
	/* Unmask and clear IRQ */
	writel(BE_APB_IRQ_MASK, apb->regs + BE_APB_IRQ_CTRL);
	/* Return success */
	return IRQ_HANDLED;
}

static int be_apb_probe(struct platform_device *pdev)
{
	struct be_apb *apb;
	struct resource *res;
	int ret;

	apb = devm_kzalloc(&pdev->dev, sizeof(*apb), GFP_KERNEL);
	if (!apb)
		return -ENOMEM;

	apb->dev = &pdev->dev;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	apb->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(apb->regs))
		return PTR_ERR(apb->regs);

	/* Try to get IRQ resource */
	apb->irq = platform_get_irq(pdev, 0);
	if (apb->irq < 0)
		return -EIO;

	/* Request IRQ */
	ret = request_irq(apb->irq, be_apb_irq,
					IRQF_SHARED, "be-apb", (void *)apb);
	if (ret)
		return ret;

	/* Unmask and clear IRQ */
	writel(BE_APB_IRQ_MASK, apb->regs + BE_APB_IRQ_CTRL);
	dev_set_drvdata(&pdev->dev, apb);
	/* Register sysfs entries */
	be_apb_sysfs_init(&pdev->dev);

	dev_info(&pdev->dev, "Baikal Peripheral Bus Error handler\n");
	dev_info(&pdev->dev, "Version " VERSION "\n");

	return 0;
}

static int be_apb_remove(struct platform_device *pdev)
{
	struct be_apb *apb = platform_get_drvdata(pdev);
	/* Free IRQ resource */
	free_irq(apb->irq, (void *)apb);
	/* Free sysfs */
	be_apb_sysfs_remove(apb->dev);
	/* Return success */
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id be_apb_of_match[] = {
	{ .compatible = "be,apb-ehb", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, be_apb_of_match);
#endif

static struct platform_driver be_apb_driver = {
	.probe		= be_apb_probe,
	.remove		= be_apb_remove,
	.driver		= {
		.name	= "be-apb",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(be_apb_of_match),
#endif /* CONFIG_OF */
	},
};
module_platform_driver(be_apb_driver);
MODULE_VERSION(VERSION);
MODULE_AUTHOR("Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal Electronics APB Terminator Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:be_apb");
