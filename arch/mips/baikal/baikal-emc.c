/*
 * Baikal-T SOC platform support code. Extended memory
 * controller driver.
 *
 * Copyright (C) 2016  Baikal Electronics JSC
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

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/sysfs.h>

#define VERSION	"1.01"

#define EMC_MSTR				(0x00)	/* Master Register */
#define EMC_STAT				(0x04)	/* Operating Mode Status Register */
#define EMC_ECCCFG0				(0x70)	/* ECC Configuration Register */
#define EMC_ECCCFG1				(0x74)	/* ECC Configuration Register */
#define EMC_ECCSTAT				(0x78)	/* ECC Status Register */
#define EMC_ECCCLR				(0x7c)	/* ECC Clear Register */
#define EMC_ECCERRCNT			(0x80)	/* ECC Error Counter Register */
#define EMC_ECCCADDR0			(0x84)	/* ECC Corrected Error Address Register 0 */
#define EMC_ECCCADDR1			(0x88)	/* ECC Corrected Error Address Register 1 */
#define EMC_ECCUADDR0			(0xa4)	/* ECC Uncorrected Error Address Register 0 */
#define EMC_ECCUADDR1			(0xa8)	/* ECC Uncorrected Error Address Register 1 */
#define EMC_CRCPARCTL0			(0xc0)	/* CRC Parity Control Register0 */
#define EMC_CRCPARCTL1			(0xc4)	/* CRC Parity Control Register1 */
#define EMC_CRCPARCTL2			(0xc8)	/* CRC Parity Control Register2 */
#define EMC_CRCPARSTAT			(0xcc)	/* CRC Parity Status Register */

#define EMC_ECCCLR_UNCORR_CNT	(1 << 3) 
#define EMC_ECCCLR_CORR_CNT		(1 << 2) 
#define EMC_ECCCLR_UNCORR_ERR	(1 << 1) 
#define EMC_ECCCLR_CORR_ERR		(1 << 0) 

#define EMC_CRCPARCTL0_CLR_CNT	(1 << 2)
#define EMC_CRCPARCTL0_CLR_ERR	(1 << 1)
#define EMC_CRCPARCTL0_EN		(1 << 0)
#define EMC_CRCPARCTL1_PAR_EN	(1 << 8)
#define EMC_CRCPARCTL1_CRC_EN	(1 << 4)

#define	DRAM_TYPE_DDR3			(1 << 0)
#define	DRAM_TYPE_MDDR			(1 << 1)
#define	DRAM_TYPE_LPDDR2		(1 << 2)
#define	DRAM_TYPE_LPDDR3		(1 << 3)
#define	DRAM_TYPE_DDR4			(1 << 4)

struct be_emc_info {
	unsigned char cfg;
	unsigned char ranks;
	unsigned char burst;
	unsigned char dqdiv;
	unsigned char gdown;
	unsigned char timing;
	unsigned char bchop;
	unsigned char bmode;
	unsigned char ecc;
	unsigned char type;
};

struct be_emc {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	int	irq_dfi, irq_ecr, irq_euc;
	struct be_emc_info info;
};

#ifdef CONFIG_SYSFS
static ssize_t show_emc_stat(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct be_emc *emc = platform_get_drvdata(pdev);
	unsigned int reg1, reg2;

	reg1 = readl(emc->regs + EMC_CRCPARSTAT);
	reg2 = readl(emc->regs + EMC_ECCERRCNT);

	return scnprintf(buf, PAGE_SIZE, 
				"Parity or CRC errors    : %u\n"
				"Correctable ECC errors  : %u\n" 
				"Uncorrectable ECC errors: %u\n", 
				reg1 & 0xffff, reg2 & 0xffff, reg2 >> 16);
}
static DEVICE_ATTR(stat, S_IWUSR | S_IRUGO, show_emc_stat, NULL);

static ssize_t show_emc_rate(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct be_emc *emc = platform_get_drvdata(pdev);

	return scnprintf(buf, PAGE_SIZE, "%lu\n", clk_get_rate(emc->clk));
}
static DEVICE_ATTR(rate, S_IWUSR | S_IRUGO, show_emc_rate, NULL);

static void be_emc_sysfs_init(struct device *dev)
{
	int ret;

	ret = sysfs_create_file(&dev->kobj, &dev_attr_stat.attr);
	if (ret)
		return;

	ret = sysfs_create_file(&dev->kobj, &dev_attr_rate.attr);
	if (ret)
		goto __err;
__err:
	sysfs_remove_file(&dev->kobj, &dev_attr_stat.attr);
}

static void be_emc_sysfs_remove(struct device *dev)
{
	sysfs_remove_file(&dev->kobj, &dev_attr_stat.attr);
	sysfs_remove_file(&dev->kobj, &dev_attr_rate.attr);
}
#else
static void be_emc_sysfs_init(struct device *dev) {}
static void be_emc_sysfs_remove(struct device *dev) {}
#endif

static irqreturn_t be_emc_irq(int irq, void *data)
{
	struct be_emc *emc = (struct be_emc *)data;
	unsigned int reg1, reg2;

	if (irq == emc->irq_dfi) {
		dev_crit_ratelimited(emc->dev, "Parity or CRC error is "
			"detected on the DFI interface\n");

		reg1 = readl(emc->regs + EMC_CRCPARCTL0);
		reg1 |= EMC_CRCPARCTL0_CLR_ERR;
		writel(reg1, emc->regs + EMC_CRCPARCTL0);

		return IRQ_HANDLED;
	}

	if (irq == emc->irq_ecr) {
		dev_warn_ratelimited(emc->dev, "Correctable ECC error is "
			"detected\n");

		reg1 = readl(emc->regs + EMC_ECCCADDR0);
		reg2 = readl(emc->regs + EMC_ECCCADDR1);

		dev_warn_ratelimited(emc->dev, "RANK=%d ROW=%d BGRP=%d "
			"BANK=%d COL=%d\n", (reg1 >> 24) & 0x03, reg1 & 0x3ffff,
			(reg2 >> 24) & 0x03, (reg2 >> 16) & 0x07, reg2 & 0x0fff);

		writel(EMC_ECCCLR_CORR_ERR,	emc->regs + EMC_ECCCLR);

		return IRQ_HANDLED;
	}

	if (irq == emc->irq_euc) {
		dev_crit_ratelimited(emc->dev, "Uncorrectable ECC error is "
			"detected\n");

		reg1 = readl(emc->regs + EMC_ECCUADDR0);
		reg2 = readl(emc->regs + EMC_ECCUADDR1);

		dev_crit_ratelimited(emc->dev, "RANK=%d ROW=%d BGRP=%d "
			"BANK=%d COL=%d\n", (reg1 >> 24) & 0x03, reg1 & 0x3ffff,
			(reg2 >> 24) & 0x03, (reg2 >> 16) & 0x07, reg2 & 0x0fff);

		writel(EMC_ECCCLR_UNCORR_ERR, emc->regs + EMC_ECCCLR);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static const char * be_emc_ddr_type(unsigned char type)
{
	switch (type) {
	case DRAM_TYPE_DDR3:
		return "DDR3";
	case DRAM_TYPE_MDDR:
		return "mDDR";
	case DRAM_TYPE_LPDDR2:
		return "LPDDR2";
	case DRAM_TYPE_LPDDR3:
		return "LPDDR3";
	case DRAM_TYPE_DDR4:
		return "DDR4";
	default:
		return "DDR";
	}
}

static void be_emc_init(struct be_emc *emc)
{
	struct be_emc_info *info = &emc->info;
	unsigned int reg;

	reg = readl(emc->regs + EMC_MSTR);

	info->cfg = 4 << ((reg >> 30) & 0x03);
	info->ranks = (((reg >> 24) & 0x0f) + 1) >> 1;
	info->burst = 1 << ((reg >> 16) & 0x0f);
	info->dqdiv = 1 << ((reg >> 12) & 0x03);
	info->gdown = (reg >> 11) & 0x01;
	info->timing = 1 << ((reg >> 10) & 0x01);
	info->bchop = (reg >> 9) & 0x01;
	info->bmode = (reg >> 8) & 0x01;
	info->type = reg & 0x1f;

	reg = readl(emc->regs + EMC_ECCCFG0);
	info->ecc = (reg & 0x07) ? 1 : 0;

	dev_info(emc->dev, "%s x%d ranks:%d ecc:%s burst:%d bus:DQ/%d "
		"timing:%dT bchop:%d bmode:%d\n", be_emc_ddr_type(info->type),
		info->cfg, info->ranks, info->ecc ? "on":"off", 
		info->burst, info->dqdiv, info->timing, info->bchop, info->bmode);
	
	/* Enable DFI irq and clear status */
	reg = readl(emc->regs + EMC_CRCPARCTL0);
	reg |= (EMC_CRCPARCTL0_CLR_CNT | EMC_CRCPARCTL0_CLR_ERR |
			EMC_CRCPARCTL0_EN);
	writel(reg, emc->regs + EMC_CRCPARCTL0);
	wmb();

	writel(EMC_ECCCLR_CORR_CNT | EMC_ECCCLR_CORR_ERR |
		EMC_ECCCLR_UNCORR_CNT | EMC_ECCCLR_UNCORR_ERR,
		emc->regs + EMC_ECCCLR);
	wmb();
}

static int be_emc_probe(struct platform_device *pdev)
{
	struct be_emc *emc;
	struct resource *res;
	int ret;

	emc = devm_kzalloc(&pdev->dev, sizeof(*emc), GFP_KERNEL);
	if (!emc)
		return -ENOMEM;

	emc->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	emc->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(emc->regs))
		return PTR_ERR(emc->regs);

	emc->clk = devm_clk_get(&pdev->dev, "ddrclk");
	if (IS_ERR(emc->clk))
		return PTR_ERR(emc->clk);
	
	clk_prepare_enable(emc->clk);

	/* dfi_alert_err_intr */
	emc->irq_dfi = platform_get_irq(pdev, 0);
	if (emc->irq_dfi < 0)
		return -EIO;

	/* Request IRQ */
	ret = request_irq(emc->irq_dfi, be_emc_irq,
			IRQF_SHARED, "emc-dfi", (void *)emc);
	if (ret)
		return ret;

	/* Try to get recovered error IRQ */
	emc->irq_ecr = platform_get_irq(pdev, 1);
	if (emc->irq_ecr < 0)
		goto __main;

	/* ecc_corrected_err_intr */
	ret = request_irq(emc->irq_ecr, be_emc_irq,
			IRQF_SHARED, "emc-ecr", (void *)emc);
	if (ret)
		return ret;

	/* ecc_uncorrected_err_intr */
	emc->irq_euc = platform_get_irq(pdev, 2);
	if (emc->irq_euc < 0)
		goto __main;

	/* Request IRQ */
	ret = request_irq(emc->irq_euc, be_emc_irq,
			IRQF_SHARED, "emc-euc", (void *)emc);
	if (ret)
		return ret;

__main:
	/* Report info */
	dev_info(&pdev->dev, "Baikal Extended Memory controller\n");
	dev_info(&pdev->dev, "Version " VERSION "\n");

	be_emc_init(emc);

	platform_set_drvdata(pdev, emc);

	/* Register sysfs entries */
	be_emc_sysfs_init(&pdev->dev);

	return 0;
}

static int be_emc_remove(struct platform_device *pdev)
{
	struct be_emc *emc = platform_get_drvdata(pdev);

	if (emc->irq_dfi)
		free_irq(emc->irq_dfi, (void *)emc);

	if (emc->irq_ecr)
		free_irq(emc->irq_ecr, (void *)emc);

	if (emc->irq_euc)
		free_irq(emc->irq_euc, (void *)emc);

	if (emc->clk)
		 devm_clk_put(emc->dev, emc->clk);

	be_emc_sysfs_remove(&pdev->dev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id be_emc_of_match[] = {
	{ .compatible = "be,memory-controller", },
	{ .compatible = "be,emc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, be_emc_of_match);
#endif

static struct platform_driver be_emc_driver = {
	.probe		= be_emc_probe,
	.remove		= be_emc_remove,
	.driver		= {
		.name	= "be-emc",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(be_emc_of_match),
#endif /* CONFIG_OF */
	},
};

module_platform_driver(be_emc_driver);
MODULE_VERSION(VERSION);
MODULE_AUTHOR("Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal extended memory controller driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:be_emc");
