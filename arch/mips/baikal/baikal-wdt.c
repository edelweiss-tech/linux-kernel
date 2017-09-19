/*
 * Baikal-T SOC platform support code. AXI Terminator driver.
 *
 * Copyright (C) 2014-2016 Baikal Electronics JSC
 * 
 * Author:
 *   Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>
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

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqchip/mips-gic.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/percpu.h>
#include <linux/platform_device.h>
#include <linux/smp.h>

#define VERSION			"1.01"
#define WDT_SECONDS		30
#define WDT_TIMEOUT		(HZ / 2)
#define WDT_MODE_DEF	GIC_WD_CTRL_TYPE_PIT

struct {
	struct clk			*clk;
	unsigned int 		irq;
	unsigned int 		freq;
	unsigned int 		next;
	unsigned int 		initial;
} be_wdt;

static inline void be_wdt_set_next_heartbeat(void)
{
	be_wdt.next = jiffies + WDT_SECONDS * HZ;
}

static void be_wdt_update(void *data)
{
	gic_write_wd_initial(be_wdt.initial);	
}

static void be_wdt_keepalive(void)
{
	on_each_cpu(be_wdt_update, NULL, 1);
}

static irqreturn_t be_wdt_interrupt(int irq, void *dev_id)
{
	be_wdt_update(NULL);

	return IRQ_HANDLED;
}

struct irqaction be_wdt_irqaction = {
	.handler = be_wdt_interrupt,
	.flags = IRQF_PERCPU | IRQF_TIMER,
	.name = "watchdog",
};

static void be_wdt_mode(void *data)
{
	unsigned int *mode = (unsigned int *)data;
	gic_write_wd_ctrl(*mode);
}

static void be_wdt_enable(void *data)
{
	unsigned int mode = gic_read_wd_ctrl();

	mode = WDT_MODE_DEF << GIC_WD_CTRL_TYPE_SHFT;
	mode |=  GIC_WD_CTRL_WAIT_MSK | GIC_WD_CTRL_START_MSK;

	enable_percpu_irq(be_wdt.irq, IRQ_TYPE_NONE);
	on_each_cpu(be_wdt_mode, (void *)&mode, 1);
}

static void be_wdt_disable(void *data)
{
	unsigned int mode = gic_read_wd_ctrl();

	mode &=  ~GIC_WD_CTRL_START_MSK;

	disable_percpu_irq(be_wdt.irq);
	on_each_cpu(be_wdt_mode, (void *)&mode, 1);
}

static int be_wdt_cpu_notifier(struct notifier_block *nb, unsigned long action,
				void *data)
{
	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_STARTING:
		be_wdt_enable(data);
		break;
	case CPU_DYING:
		be_wdt_disable(data);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block be_wdt_cpu_nb = {
	.notifier_call = be_wdt_cpu_notifier,
};

static const char * be_wdt_mode_str(unsigned int mode)
{
	switch (mode) {
	case GIC_WD_CTRL_TYPE_SC:
		return "Secondary Count";
	case GIC_WD_CTRL_TYPE_OT:
		return "One Tick";
	case GIC_WD_CTRL_TYPE_PIT:
		return "Programm Interval";
	default:
		return "Unknown Mode";
	}
}

static int be_wdt_drv_probe(struct platform_device *pdev)
{
	int ret;

	be_wdt.clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(be_wdt.clk))
		return PTR_ERR(be_wdt.clk);

	ret = clk_prepare_enable(be_wdt.clk);
	if (ret)
		return ret;
	be_wdt.freq = clk_get_rate(be_wdt.clk);

	be_wdt.irq = platform_get_irq(pdev, 0);
	if (be_wdt.irq < 0)
		goto err_clk;

	be_wdt_irqaction.percpu_dev_id = (void *)&be_wdt;
	ret = setup_percpu_irq(be_wdt.irq, &be_wdt_irqaction);
	if (ret < 0)
		goto err_irq;

	ret = register_cpu_notifier(&be_wdt_cpu_nb);
	if (ret < 0)
		dev_warn(&pdev->dev, "Unable to register CPU notifier\n");

	be_wdt.initial = WDT_SECONDS * 10000000 / be_wdt.freq;
	be_wdt_set_next_heartbeat();

	dev_info(&pdev->dev, "Baikal Electronics GIC Watchdog Driver\n");
	dev_info(&pdev->dev, "Version " VERSION "\n");
	dev_info(&pdev->dev, "Mode: %s Timer\n", be_wdt_mode_str(WDT_MODE_DEF));

	return 0;

err_irq:
	remove_percpu_irq(be_wdt.irq, &be_wdt_irqaction);
err_clk:
	clk_disable_unprepare(be_wdt.clk);

	return ret;
}

static int be_wdt_drv_remove(struct platform_device *pdev)
{
	clk_disable_unprepare(be_wdt.clk);

	disable_percpu_irq(be_wdt.irq);

	remove_percpu_irq(be_wdt.irq, &be_wdt_irqaction);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int be_wdt_suspend(struct device *dev)
{
	clk_disable_unprepare(be_wdt.clk);

	return 0;
}

static int be_wdt_resume(struct device *dev)
{
	int ret;

	ret = clk_prepare_enable(be_wdt.clk);

	if (ret)
		return ret;

	be_wdt_keepalive();

	return 0;
}

static SIMPLE_DEV_PM_OPS(be_wdt_pm_ops, be_wdt_suspend, be_wdt_resume);
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_OF
static const struct of_device_id be_wdt_of_match[] = {
	{ .compatible = "be,gic-wdt", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, be_wdt_of_match);
#endif

static struct platform_driver be_wdt_driver = {
	.probe		= be_wdt_drv_probe,
	.remove		= be_wdt_drv_remove,
	.driver		= {
		.name	= "be-wdt",
		.of_match_table = of_match_ptr(be_wdt_of_match),
#ifdef CONFIG_PM_SLEEP
		.pm	= &be_wdt_pm_ops,
#endif
	},
};

module_platform_driver(be_wdt_driver);

MODULE_VERSION(VERSION);
MODULE_AUTHOR("Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal Electronics GIC Watchdog Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:be_wdt");
