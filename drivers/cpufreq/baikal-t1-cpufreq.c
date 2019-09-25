/*
 * Baikal-T SOC platform support code. 
 * CPU Frequency Scaling driver.
 *
 * Copyright (C) 2018 Baikal Electronics JSC
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published bythe Free Software Foundation; either version 2
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
#include <linux/irqchip/mips-gic.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define VERSION	"2.0"

#define PLL_FREQ_MAX	1300000	/* KHz */
#define PLL_FREQ_MIN	200000	/* KHz */
#define PLL_FREQ_STEP	25000	/* KHz */

struct be_cpufreq {
	struct device *dev;
	void    __iomem *cpufreq_dev;
	struct clk *clk;			/* CPU clk */
	struct clk *coreclk;			/* Core PLL parent of CPU clk*/
	unsigned int max_freq;			/* KHz */
	unsigned int min_freq;			/* KHz */
	unsigned int latency;			/* uS  */
};
static struct be_cpufreq *cpufreq;

static int be_cpufreq_notifier(struct notifier_block *nb,
				 unsigned long val, void *data)
{
	struct cpufreq_freqs *freqs = data;
	/* Change freq in /proc/cpuinfo */
	if (val == CPUFREQ_POSTCHANGE)
		cpu_data[freqs->cpu].udelay_val = freqs->new * 5;

	return NOTIFY_OK;
}

static struct notifier_block be_cpufreq_notifier_block = {
	.notifier_call = be_cpufreq_notifier,
};

static int be_cpufreq_target(struct cpufreq_policy *policy,
			       unsigned int index)
{
	unsigned int reg;
	struct cpufreq_freqs freqs;

	freqs.old = policy->cur;
	freqs.new = policy->freq_table[index].frequency;

	dev_info(cpufreq->dev,"%u KHz --> %u KHz\n", freqs.old, freqs.new);

	reg = ioread32((u32 *)(cpufreq->cpufreq_dev)); /* pull register */
	pr_debug( "Core PLL CTL reg BEFORE = %x",reg);

	clk_set_rate(policy->clk, freqs.new * 1000);

	reg = ioread32((u32 *)(cpufreq->cpufreq_dev)); /* pull register */
	pr_debug( "Core PLL CTL reg AFTER = %x",reg);
	
	policy->cur = freqs.new;

	/* Change freq in /proc/cpuinfo */
	cpu_data[0].udelay_val= clk_get_rate(policy->clk) / 1000 * 5;
	cpu_data[1].udelay_val=clk_get_rate(policy->clk) / 1000 * 5;

	dev_info(cpufreq->dev, "Frequency changing procedure completed\n");

	return 0;
}

static int be_cpufreq_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *freq_tbl;
	unsigned int steps, freq;
	int i, ret;

	steps = (cpufreq->max_freq - cpufreq->min_freq) / PLL_FREQ_STEP;

	freq_tbl = kzalloc(sizeof(*freq_tbl) * (steps + 1),
					GFP_KERNEL);

	if (!freq_tbl) {
		dev_err(cpufreq->dev,
			"Failed to alloc cpufreq frequency table\n");
		return -ENOMEM;
	}

	freq = cpufreq->min_freq;;
	for (i = 0; i <= steps; ++i) {
		if ((freq < cpufreq->min_freq) || (freq > cpufreq->max_freq))
			freq_tbl[i].frequency = CPUFREQ_ENTRY_INVALID;
		else
			freq_tbl[i].frequency = freq;
		pr_debug("CPUFreq index %d: frequency %d KHz\n", i,
			freq_tbl[i].frequency);
		freq += PLL_FREQ_STEP;
	}
	freq_tbl[steps + 1].frequency = CPUFREQ_TABLE_END;

	policy->driver_data = (void *) cpufreq;
	policy->clk = cpufreq->coreclk;
	policy->cur = clk_get_rate(policy->clk) / 1000;
	ret = cpufreq_generic_init(policy, freq_tbl, 1000);
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
	struct device_node *np;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	cpufreq = devm_kzalloc(&pdev->dev,sizeof(*cpufreq), GFP_KERNEL);
	if (!cpufreq)
		return -ENOMEM;

	cpufreq->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	cpufreq->cpufreq_dev = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(cpufreq->cpufreq_dev))
		return PTR_ERR(cpufreq->cpufreq_dev);

	cpufreq->clk = devm_clk_get(dev, "cpuclk");
	if (IS_ERR(cpufreq->clk)) {
		dev_err(dev, "Unable to get CPU clock\n");
		return PTR_ERR(cpufreq->clk);
	}

	cpufreq->coreclk = clk_get_parent(cpufreq->clk);
		if (IS_ERR(cpufreq->coreclk)) {
			dev_err(dev, "Unable to get COREPLL which is a parent of CPU clock\n");
			return PTR_ERR(cpufreq->coreclk);
		}

	np = of_find_node_by_name(NULL,"core_pll");

	if (!np) {
		dev_err(dev, "Failed to find DT node\n");
		return -ENOENT;
	}

	if (of_property_read_u32_index(np, "clock-frequency-range", 0,
		&cpufreq->min_freq))
		cpufreq->min_freq = PLL_FREQ_MIN * 1000;
	cpufreq->min_freq = cpufreq->min_freq / 1000;

	if (of_property_read_u32_index(np, "clock-frequency-range", 1,
		&cpufreq->max_freq))
		cpufreq->max_freq = PLL_FREQ_MAX * 1000;
	cpufreq->max_freq = cpufreq->max_freq / 1000;


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
MODULE_AUTHOR("Georgy Vlasov <Georgy.Vlasov@baikalelectronics.ru>");
MODULE_AUTHOR("Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal CPUFreq driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:be_cpufreq");
