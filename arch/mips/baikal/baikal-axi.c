/*
 * Baikal-T SOC platform support code. AXI Terminator driver.
 *
 * Copyright (C) 2014-2016 Baikal Electronics JSC
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
#include <linux/device.h>		/* dev_err */
#include <linux/module.h>
#include <linux/of_platform.h>	/* open firmware functioons */
#include <linux/sysfs.h>		/* sysfs functions */

#define VERSION	"1.02"

#define BE_AXI_ADDRL_OFS		0x00
#define BE_AXI_ADDRH_OFS		0x04

#define BE_AXI_ADDRH_MASK		0xff
#define BE_AXI_ADDRH_SHFT		24
#define BE_AXI_TYPE_MASK		0x01
#define BE_AXI_TYPE_SHFT		23

#define BE_MSG_NOERROR			"No interconnect errors detected"
#define BE_MSG_SLAVE_ERROR		"Slave returns internal error"
#define BE_MSG_DECODE_ERROR		"No slave at selected address"

struct be_axi {
	struct device *dev;
	void __iomem *regs;
	int	irq;
	unsigned int count;
	unsigned long long addr;
	unsigned int type;
};

#ifdef CONFIG_SYSFS
static ssize_t show_count(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct be_axi *axi = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", axi->count);
}
static DEVICE_ATTR(count, S_IWUSR | S_IRUGO, show_count, NULL);

static ssize_t show_addr(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct be_axi *axi = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%08llx\n", axi->addr);
}
static DEVICE_ATTR(addr, S_IWUSR | S_IRUGO, show_addr, NULL);

static ssize_t show_type(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct be_axi *axi = dev_get_drvdata(dev);

	if (!axi->count)
		return scnprintf(buf, PAGE_SIZE, "%s\n", BE_MSG_NOERROR);
	return scnprintf(buf, PAGE_SIZE, "%s\n", axi->type ? BE_MSG_DECODE_ERROR : BE_MSG_SLAVE_ERROR);
}
static DEVICE_ATTR(type, S_IWUSR | S_IRUGO, show_type, NULL);

static ssize_t show_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "Test interconnect error "
				"(0 - Slave internal error, 1 - No slave error)\n");
}
static ssize_t store_test(struct device *dev, struct device_attribute *attr,
                 const char *buf, size_t count)
{
	struct be_axi *axi = dev_get_drvdata(dev);

	/* Dummy byte read */
	if (*buf == '0')
		readb(axi->regs + BE_AXI_ADDRL_OFS);
	if (*buf == '1')
		readb(axi->regs + 1);
	return count;
}
static DEVICE_ATTR(test, S_IWUSR | S_IRUGO, show_test, store_test);

static void be_axi_sysfs_init(struct device *dev)
{
	int ret;
	/* Errors count */
	ret = sysfs_create_file(&dev->kobj, &dev_attr_count.attr);
	if (ret)
		return;
	/* Last error address */
	ret = sysfs_create_file(&dev->kobj, &dev_attr_addr.attr);
	if (ret)
		goto __err3;
	/* Last error type */
	ret = sysfs_create_file(&dev->kobj, &dev_attr_type.attr);
	if (ret)
		goto __err2;
	/* Test entry */
	ret = sysfs_create_file(&dev->kobj, &dev_attr_test.attr);
	if (ret)
		goto __err1;
	return;
__err1:
	sysfs_remove_file(&dev->kobj, &dev_attr_type.attr);
__err2:
	sysfs_remove_file(&dev->kobj, &dev_attr_addr.attr);
__err3:
	sysfs_remove_file(&dev->kobj, &dev_attr_count.attr);
}

static void be_axi_sysfs_remove(struct device *dev)
{
	sysfs_remove_file(&dev->kobj, &dev_attr_count.attr);
	sysfs_remove_file(&dev->kobj, &dev_attr_addr.attr);
	sysfs_remove_file(&dev->kobj, &dev_attr_type.attr);
	sysfs_remove_file(&dev->kobj, &dev_attr_test.attr);
}
#else
static void be_axi_sysfs_init(struct device *dev) {}
static void be_axi_sysfs_remove(struct device *dev) {}
#endif

static irqreturn_t be_axi_irq(int irq, void *data)
{
	struct be_axi *axi = (struct be_axi *)data;
	unsigned long long addr;
	/* Get low part of fault address */
	axi->addr = readl(axi->regs + BE_AXI_ADDRL_OFS);
	/* Get high part of fault address */
	addr = readl(axi->regs + BE_AXI_ADDRH_OFS);
	/* Add high bits to fault address */
	axi->addr |= ((addr >> BE_AXI_ADDRH_SHFT) & BE_AXI_ADDRH_MASK) << 32;
	/* Get fault type */
	axi->type = (addr >> BE_AXI_TYPE_SHFT) & BE_AXI_TYPE_MASK;
	/* Alert */
	dev_crit_ratelimited(axi->dev, "Interconnect: %s (handled at %08llx)\n",
		axi->type ? BE_MSG_DECODE_ERROR : BE_MSG_SLAVE_ERROR, axi->addr);
	/* Increase counter (in irq handler it is atomic) */
	axi->count += 1;
	/* Return success */
	return IRQ_HANDLED;
}

static int be_axi_probe(struct platform_device *pdev)
{
	struct be_axi *axi;
	struct resource *res;
	int ret;

	axi = devm_kzalloc(&pdev->dev, sizeof(*axi), GFP_KERNEL);
	if (!axi)
		return -ENOMEM;

	axi->dev = &pdev->dev;
	
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	axi->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(axi->regs))
		return PTR_ERR(axi->regs);

	/* Try to get IRQ resource */
	axi->irq = platform_get_irq(pdev, 0);
	if (axi->irq < 0)
		return -EIO;

	/* Request IRQ */
	ret = request_irq(axi->irq, be_axi_irq,
			IRQF_SHARED, "be-axi", (void *)axi);
	if (ret)
		return ret;

	dev_set_drvdata(&pdev->dev, axi);
	/* Register sysfs entries */
	be_axi_sysfs_init(&pdev->dev);

	dev_info(&pdev->dev, "Baikal Interconnect Error handler\n");
	dev_info(&pdev->dev, "Version " VERSION "\n");

	return 0;
}

static int be_axi_remove(struct platform_device *pdev)
{
	struct be_axi *axi = platform_get_drvdata(pdev);
	/* Free IRQ resource */
	free_irq(axi->irq, axi);
	/* Free sysfs */
	be_axi_sysfs_remove(axi->dev);
	/* Return success */
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id be_axi_of_match[] = {
	{ .compatible = "be,axi-ehb", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, be_axi_of_match);
#endif

static struct platform_driver be_axi_driver = {
	.probe		= be_axi_probe,
	.remove		= be_axi_remove,
	.driver		= {
		.name	= "be-axi",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(be_axi_of_match),
#endif /* CONFIG_OF */
	},
};
module_platform_driver(be_axi_driver);
MODULE_VERSION(VERSION);
MODULE_AUTHOR("Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal Electronics Interconnect error handler driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:be_axi");
