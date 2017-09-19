/*
 * Synopsis DesignWare RTC driver.
 *
 * Copyright (C) 2013 Baikal Electronics.
 *
 * Author: Alexey Malahov <Alexey.Malahov@baikalelectronics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <linux/of.h>

/* RTC registers. */
#define RTC_CCVR		(0x0) /* Current Counter Value Register. */
#define RTC_CMR			(0x4) /* Current Match Register. */
#define RTC_CLR 		(0x8) /* Current Load Register. */
#define RTC_CCR			(0xc) /* Counter Control Register. */
#define RTC_STAT		(0x10) /* Interrupt Status Register. */
#define RTC_RSTAT		(0x14) /* Interrupt Raw Status Register. */
#define RTC_EOI			(0x18) /* End of Interrupt Register. */
#define RTC_COMP_VERSION	(0x1c) /* Component Version Register. */
#define RTC_COMP_VERSION_VAL	0x3230332a

/* RTC_CCR bits. */
#define RTC_CCR_WEN	(1 << 3)	/* Wrap enable. */
#define RTC_CCR_EN	(1 << 2)	/* Counter enable. */
#define RTC_CCR_IMASK	(1 << 1)	/* Interrupt masked. */
#define RTC_CCR_IEN	(1 << 0)	/* Interrupt enabled. */

/* rtc_clk frequency by default. 1kHz. */
#define	RTC_CLK_FREQ_DEFAULT	1000

/* default RTC epoch. */
#define RTC_EPOCH_DEFAULT	1970 /* Jan 1 1970 00:00:00 */

struct dw_rtc {
	struct rtc_device	*rtc_dev;
	void __iomem		*base;
	uint32_t		freq;
	uint32_t		epoch;
	unsigned long		epoch_seconds;
	uint32_t		irq;
};

static int dw_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct dw_rtc *rtc = dev_get_drvdata(dev);
	uint32_t reg;

	if (!rtc)
		return -ENODEV;

	/* Read the control register. */
	reg = readl(rtc->base + RTC_CCR);

	if (enabled) {
		/* Check the pending interrupt. */
		if (readl(rtc->base + RTC_RSTAT)) {
			/*
			 *  According to the databook the rtc_en bit of the RTC_CCR
			 *  register should be set to 0 to clear the interrupt.
 			 */
			reg &= ~RTC_CCR_EN;
			writel(reg, rtc->base + RTC_CCR);

			reg |= RTC_CCR_EN;
		}

		/* Unmask and enable an interrupt. */
		reg &= ~RTC_CCR_IMASK;
		reg |= RTC_CCR_IEN;
	} else {
		/* Disable and mask interrupt. */
		reg &= ~RTC_CCR_IEN;
		reg |= RTC_CCR_IMASK;
	}

	writel(reg, rtc->base + RTC_CCR);

	dev_dbg(dev, "%s: Set RTC_CCR to 0x%x\n", __FUNCTION__, reg);
	return 0;
}

static int dw_read_reg(struct device *dev, int reg, struct rtc_time *tm)
{
	struct dw_rtc *rtc = dev_get_drvdata(dev);
	unsigned long seconds;

	if (!rtc)
		return -ENODEV;

	/* Get elapsed seconds. */
	seconds = readl(rtc->base + reg) / rtc->freq;

	dev_dbg(dev, "Read %ld sec from 0x%x register, epoch secs =%ld\n",
			seconds, reg, rtc->epoch_seconds);

	rtc_time_to_tm((seconds + rtc->epoch_seconds), tm);

	dev_dbg(dev, "%s: Read time: year=%d, mon=%d, mday=%d, hour=%d, min=%d sec=%d\n",
				__FUNCTION__, tm->tm_year, tm->tm_mon, tm->tm_mday,
				tm->tm_hour, tm->tm_min, tm->tm_sec);
	return 0;
}

static int dw_set_reg(struct device *dev, int reg, struct rtc_time *tm)
{
	struct dw_rtc *rtc = dev_get_drvdata(dev);
	unsigned long seconds;

	if (!rtc)
		return -ENODEV;

	dev_dbg(dev, "%s: Set time: year=%d, mon=%d, mday=%d, hour=%d, min=%d sec=%d\n",
				__FUNCTION__, tm->tm_year, tm->tm_mon, tm->tm_mday,
				tm->tm_hour, tm->tm_min, tm->tm_sec);

	/* Check time passed. */
	if (rtc_valid_tm(tm) != 0) {
		dev_err(dev, "time is unvalid.\n");
		return -EINVAL;
	}

	seconds = mktime(tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
			tm->tm_hour, tm->tm_min, tm->tm_sec);

	dev_dbg(dev, "%s: sec=%ld epoch_sec=%ld delta=%ld\n", __FUNCTION__, seconds, rtc->epoch_seconds,
			seconds - rtc->epoch_seconds);

	seconds -= rtc->epoch_seconds;

	if ((seconds >= 0) && ((uint64_t) seconds * rtc->freq) < ULONG_MAX) {
		dev_dbg(dev, "Write %ld sec to 0x%x register\n", seconds, reg);
		writel((rtc->freq * seconds), rtc->base + reg);
	} else {
		dev_err(dev, "Change the RTC epoch: %d year\n", rtc->epoch);
		return -EINVAL;
	}

	return 0;
}


static irqreturn_t dw_irq(int irq, void *dev_id)
{
	struct dw_rtc *rtc = dev_id;

	/* Clear the interrupt. */
	readl(rtc->base + RTC_EOI);

	rtc_update_irq(rtc->rtc_dev, 1, (RTC_AF | RTC_IRQF));
	return IRQ_HANDLED;
}

static int dw_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	/* Read time from RTC_CMR. */
	return dw_read_reg(dev, RTC_CMR, &alm->time);
}

static int dw_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	int ret;

	/* Set time to RTC_CMR. */
	if ((ret = dw_set_reg(dev, RTC_CMR, &alm->time)) != 0) {
		return ret;
	}

	/* Does alarm need to be enalbed? */
	if (alm->enabled) {
		dw_alarm_irq_enable(dev, alm->enabled);
	}

	return 0;
}

static int dw_read_time(struct device *dev, struct rtc_time *tm)
{
	/* Read time from RTC_CCVR. */
	return dw_read_reg(dev, RTC_CCVR, tm);
}

static int dw_set_time(struct device *dev, struct rtc_time *tm)
{
	/* Set time to RTC_CLR. */
	return dw_set_reg(dev, RTC_CLR, tm);
}

static int dw_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct dw_rtc *rtc = dev_get_drvdata(dev);
	
	if (!rtc)
		return -ENODEV;

	switch (cmd) {
	case RTC_EPOCH_READ:
		return put_user(rtc->epoch, (unsigned long __user *)arg);
	case RTC_EPOCH_SET:
		/* Doesn't support before 1900 */
		if (arg < 1900)
			return -EINVAL;
		rtc->epoch = arg;
		rtc->epoch_seconds = mktime(rtc->epoch, 1, 1, 0, 0, 0);
		break;
	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}


static int dw_proc(struct device *dev, struct seq_file *seq)
{
	uint32_t reg;
	struct dw_rtc *rtc = dev_get_drvdata(dev);
	
	if (!rtc)
		return -ENODEV;

	reg = readl(rtc->base + RTC_CCR);
	seq_printf(seq, "Wrap enable\t: %s\n", reg & RTC_CCR_WEN ? "yes" : "no");
	seq_printf(seq, "Counter enable\t: %s\n", reg & RTC_CCR_EN ? "yes" : "no");
	seq_printf(seq, "Interrupt masked\t: %s\n", reg & RTC_CCR_IMASK ? "yes" : "no");
	seq_printf(seq, "Interrupt enabled\t: %s\n", reg & RTC_CCR_IEN ? "yes" : "no");

	seq_printf(seq, "RTC epoch\t\t: %d year\n", rtc->epoch);
	seq_printf(seq, "RTC base\t\t: 0x%p\n", rtc->base);
	seq_printf(seq, "RTC freq\t\t: %d\n", rtc->freq);

	reg = readl(rtc->base + RTC_CCVR);
	seq_printf(seq, "RTC_CCVR\t\t: 0x%x\n", reg);

	reg = readl(rtc->base + RTC_CMR);
	seq_printf(seq, "RTC_CMR \t\t: 0x%x\n", reg);

	return 0;
}

static const struct rtc_class_ops dw_ops = {
	.ioctl			= dw_ioctl,
	.read_time		= dw_read_time,
	.set_time		= dw_set_time,
	.read_alarm		= dw_read_alarm,
	.set_alarm		= dw_set_alarm,
	.proc			= dw_proc,
	.alarm_irq_enable 	= dw_alarm_irq_enable,
};

static int dw_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct dw_rtc *rtc;
        struct device_node *np = pdev->dev.of_node;
        const uint32_t *iprop;

	/* Allocate resources. */
	if (!(rtc = devm_kzalloc(&pdev->dev, sizeof(struct dw_rtc), GFP_KERNEL))) {
		dev_err(&pdev->dev, "cannot allocate memory\n");
		return -ENOMEM;
	}

	if (!(res = platform_get_resource(pdev, IORESOURCE_MEM, 0))) {
		dev_err(&pdev->dev, "cannot get resource\n");
		return -ENXIO;
	}

	if (!(rtc->base = devm_ioremap(&pdev->dev, res->start, resource_size(res)))) {
		dev_err(&pdev->dev, "cannot ioremap\n");
		return -EIO;
	}

	dev_info(&pdev->dev, "rtc_base = %p rtc_version = 0x%x\n", rtc->base,
				readl(rtc->base + RTC_COMP_VERSION));


	rtc->rtc_dev = devm_rtc_device_register(&pdev->dev, pdev->name,
						&dw_ops, THIS_MODULE);

	if (IS_ERR(rtc->rtc_dev)) {
		dev_err(&pdev->dev, "cannot register rtc device\n");
		return PTR_ERR(rtc->rtc_dev);
	}

	/* Clear pending interrupts. */
	writel(0, rtc->base + RTC_CCR);

	/* Enable counter. */
	writel(RTC_CCR_EN, rtc->base + RTC_CCR);

	if ((rtc->irq = platform_get_irq(pdev, 0)) <= 0) {
		dev_err(&pdev->dev, "cannot get irq.\n");
		return -ENXIO;
	}

	dev_info(&pdev->dev, "rtc_irq = %d\n", rtc->irq);

	if (devm_request_irq(&pdev->dev, rtc->irq, dw_irq, 0, "dw_rtc", rtc) < 0) {
		dev_err(&pdev->dev, "cannot request irq%d\n", rtc->irq);
		return -EIO;
	}

	platform_set_drvdata(pdev, rtc);

	np = pdev->dev.of_node;
	iprop = of_get_property(np, "rt-clk-frequency", NULL);
	if (iprop)
		rtc->freq = be32_to_cpup(iprop);
	else
		rtc->freq = RTC_CLK_FREQ_DEFAULT;

	dev_info(&pdev->dev, "rtc_clk freq = %d Hz\n", rtc->freq);

	/* Set RTC epoch. */
	rtc->epoch = RTC_EPOCH_DEFAULT;
	rtc->epoch_seconds = mktime(rtc->epoch, 1, 1, 0, 0, 0);


	/* Disable irq. */
	dw_alarm_irq_enable(&pdev->dev, 0);

	dev_info(&pdev->dev, "Synopsis DesignWare Real Time Clock\n");

	return 0;

}

static int dw_remove(struct platform_device *pdev)
{
        struct dw_rtc *rtc = platform_get_drvdata(pdev);

        if (!rtc)
                return 0;

	/* Disable irq. */
	dw_alarm_irq_enable(&pdev->dev, 0);

        platform_set_drvdata(pdev, NULL);

        return 0;
}


//static SIMPLE_DEV_PM_OPS(dw_pm_ops, dw_suspend, dw_resume); //TODO

static const struct of_device_id rtc_dt_ids[] = {
	{ .compatible = "snps,dw-rtc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rtc_dt_ids);

static struct platform_driver dw_rtcdrv = {
	.probe          = dw_probe,
	.remove         = dw_remove,
	.driver         = {
		.name   = "dw-rtc",
		.owner  = THIS_MODULE,
//		.pm     = &dw_rtc_pm_ops,
		.of_match_table = of_match_ptr(rtc_dt_ids),
	},
};

module_platform_driver(dw_rtcdrv);

MODULE_AUTHOR(" Alexey Malahov <Alexey.Malahov@baikalelectronics.com>");
MODULE_DESCRIPTION("Synopsis DesignWare RTC driver.");
MODULE_LICENSE("GPL");
