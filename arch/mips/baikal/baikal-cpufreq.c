/*
 * Baikal-T SOC platform support code. 
 * CPU Frequency Scaling driver.
 *
 * Copyright (C) 2016 Baikal Electronics JSC
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
#include <linux/clk-provider.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define VERSION	"1.01a"

#define PLL_FREQ_MAX	1200000	/* KHz */
#define PLL_FREQ_MIN	200000	/* KHz */
#define PLL_FREQ_STEP	25000	/* KHz */

struct be_cpufreq {
	struct device *dev;
	struct clk *clk;			/* CPU clk */
	unsigned int max;			/* KHz */
	unsigned int min;			/* KHz */
	unsigned int latency;		/* uS  */
};

static int be_cpufreq_notifier(struct notifier_block *nb,
				 unsigned long val, void *data)
{
	struct be_cpufreq *cpufreq = (struct be_cpufreq *)data;

	if (val == CPUFREQ_POSTCHANGE)
		cpufreq->latency = loops_per_jiffy;

	return NOTIFY_OK;
}

static struct notifier_block be_cpufreq_notifier_block = {
	.notifier_call = be_cpufreq_notifier,
};

static int be_cpufreq_target(struct cpufreq_policy *policy,
			       unsigned int index)
{
	struct be_cpufreq *cpufreq = (struct be_cpufreq *)policy->driver_data;
	unsigned int old_freq, new_freq;

	old_freq = policy->cur;
	new_freq = policy->freq_table[index].frequency;

	dev_info(cpufreq->dev, "%u KHz --> %u KHz\n", old_freq, new_freq);
	clk_set_rate(cpufreq->clk, new_freq * 1000);

	return 0;
}

static int be_cpufreq_init(struct cpufreq_policy *policy)
{
	struct be_cpufreq *cpufreq = (struct be_cpufreq *)policy->driver_data;
	struct cpufreq_frequency_table *freq_tbl;
	unsigned int steps, freq;
	int i, ret ;

	steps = (PLL_FREQ_MAX - PLL_FREQ_MIN) / PLL_FREQ_STEP;

	freq_tbl = devm_kzalloc(cpufreq->dev, sizeof(*freq_tbl) * (steps + 1),
					GFP_KERNEL);

	if (!freq_tbl) {
		dev_err(cpufreq->dev,
			"Failed to alloc cpufreq frequency table\n");
		return -ENOMEM;
	}

	freq = PLL_FREQ_MIN;
	for (i = 0; i < steps; ++i) {
		if ((freq < cpufreq->min) || (freq > cpufreq->max))
			freq_tbl[i].frequency = CPUFREQ_ENTRY_INVALID;
		else
			freq_tbl[i].frequency = freq;
		dev_info(cpufreq->dev, "CPUFreq index %d: frequency %d KHz\n", i,
			freq_tbl[i].frequency);
		freq += PLL_FREQ_STEP;
	}
	freq_tbl[steps].frequency = CPUFREQ_TABLE_END;

	policy->clk = cpufreq->clk;
	ret = cpufreq_generic_init(policy, freq_tbl, 0);
	if (ret)
		kfree(freq_tbl);

	return ret;
}

static int be_cpufreq_exit(struct cpufreq_policy *policy)
{
	kfree(policy->freq_table);
	return 0;
}

static struct cpufreq_driver be_cpufreq_driver = {
	.name		= "cpufreq-baikal",
	.flags		= CPUFREQ_STICKY | CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify		= cpufreq_generic_frequency_table_verify,
	.target_index	= be_cpufreq_target,
	.get		= cpufreq_generic_get,
	.init		= be_cpufreq_init,
	.exit		= be_cpufreq_exit,
	.attr		= cpufreq_generic_attr,
};

static int be_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_notifier(&be_cpufreq_notifier_block,
				    CPUFREQ_TRANSITION_NOTIFIER);
	cpufreq_unregister_driver(&be_cpufreq_driver);

	return 0;
}

static int be_cpufreq_probe(struct platform_device *pdev)
{
	struct be_cpufreq *cpufreq;
	struct device_node *np;
	struct device *dev = &pdev->dev;
	int ret;

	cpufreq = devm_kzalloc(&pdev->dev, sizeof(*cpufreq), GFP_KERNEL);
	if (!cpufreq)
		return -ENOMEM;

	cpufreq->dev = dev;

	cpufreq->clk = devm_clk_get(dev, "cpuclk");
	if (IS_ERR(cpufreq->clk)) {
		dev_err(dev, "Unable to get CPU clock\n");
		return PTR_ERR(cpufreq->clk);
	}

	np = of_node_get(dev->of_node);
	if (!np) {
		dev_err(dev, "Failed to find DT node\n");
		return -ENOENT;
	}

	if(of_property_read_u32(np, "be,clk-freq-min", &cpufreq->min))
		cpufreq->min = PLL_FREQ_MIN;

	if(of_property_read_u32(np, "be,clk-freq-max", &cpufreq->max))
		cpufreq->max = PLL_FREQ_MAX;

	be_cpufreq_driver.driver_data = (void *)cpufreq;

	ret = cpufreq_register_driver(&be_cpufreq_driver);
	if (ret) {
		dev_err(dev, "Failed to register cpufreq driver\n");
		return ret;
	}

	ret = cpufreq_register_notifier(&be_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
	if (ret) {
		dev_err(dev, "Failed to register cpufreq notifier\n");
		cpufreq_unregister_driver(&be_cpufreq_driver);
		return ret;
	}

	platform_set_drvdata(pdev, cpufreq);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id be_cpufreq_of_match[] = {
	{ .compatible = "be,cpufreq", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, be_cpufreq_of_match);
#endif

static struct platform_driver be_cpufreq_platdrv = {
	.probe		= be_cpufreq_probe,
	.remove		= be_cpufreq_remove,
	.driver		= {
		.name	= "be-cpufreq",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(be_cpufreq_of_match),
#endif /* CONFIG_OF */
	},

};

module_platform_driver(be_cpufreq_platdrv);

MODULE_VERSION(VERSION);
MODULE_AUTHOR("Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal CPUFreq driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:be_cpufreq");
