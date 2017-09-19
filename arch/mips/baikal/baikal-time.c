/*
 * Baikal-T SOC platform support code.
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

#include <linux/init.h>
#include <linux/sched_clock.h>
#include <linux/clk-provider.h>		/* of_clk_init */
#include <linux/clocksource.h> 		/* clocksource_of_init */
#include <linux/clk.h>			/* of_clk_get */
#include <linux/of.h>
#include <linux/irqchip/mips-gic.h>

#include <asm/bootinfo.h>
#include <asm/time.h>

#include <linux/irqchip/mips-gic.h>
#include <asm/mach-baikal/hardware.h>
#include "common.h"

static unsigned long __init plat_get_dev_clk(const char *name)
{
	struct device_node *np;
	struct clk *clk;
	int ret;

	/* Get node */
	np = of_find_compatible_node(NULL, NULL, name);
	if (!np)
		return 0;

	/* Get node clock index 0 */
	clk = of_clk_get(np, 0);
	if (IS_ERR(clk))
		return 0;

	/* Prepare and enable clock */
	ret = clk_prepare_enable(clk);
	if (!ret)
		return clk_get_rate(clk);

	if (of_property_read_u32(np, "clock-frequency", &ret))
		return 0;

	return ret;
}

/*
 * Platform timers initialization
 */
void __init plat_time_init(void)
{
	/* Init system clocks */
	of_clk_init(NULL);

	if (!of_have_populated_dt())
		pr_info("No device tree!!!!\n");

	/* Init clocksources */
	clocksource_probe();

	/* Set architectural timer frequency */
	mips_hpt_frequency = plat_get_dev_clk("mti,p5600");

	/* Check frequency */
	if (!mips_hpt_frequency) {
		pr_warn("No CPU clock frequency defined.\n");
		mips_hpt_frequency = CPU_FREQ / CPU_CLK_DIV;
	}

	/* Report CPU clock frequency */
	pr_info("CPU timer frequency: %u MHz\n",
		(unsigned int)(mips_hpt_frequency / 1000000));

	/*
	 * Use deterministic values for initial counter interrupt
	 * so that calibrate delay avoids encountering a counter wrap.
	 */
	write_c0_count(0);
	write_c0_compare(0xffff);
}
