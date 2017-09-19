/*
 * clk-baikal.c - Baikal Electronics clock driver.
 *
 * Copyright (C) 2015,2016 Baikal Electronics JSC
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
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <asm/setup.h>

#define VERSION	"1.03"

#define BE_CLK_ENABLE_MASK 		(1 << 0)
#define BE_CLK_RESET_MASK 		(1 << 1)
#define BE_CLK_SET_MASK 		(1 << 2)
#define BE_CLK_BYPASS_MASK 		(1 << 30)
#define BE_CLK_LOCK_MASK 		(1 << 31)

#define BE_CLKR_SHFT			2
#define BE_DIV_SHFT				4
#define BE_CLKF_SHFT			8
#define BE_CLKOD_SHFT			21

#define BE_CLK_DIV_MAX_WIDTH	17
#define BE_CLK_DIV_MASK			(((1 << BE_CLK_DIV_MAX_WIDTH) - 1) \
 									<< BE_DIV_SHFT)

#define BE_RD_CLKR(SRC)			(((SRC) & 0x000000FC) >> BE_CLKR_SHFT)
#define BE_RD_CLKF(SRC)			(((SRC) & 0x001FFF00) >> BE_CLKF_SHFT)
#define BE_RD_CLKOD(SRC)		(((SRC) & 0x01E00000) >> BE_CLKOD_SHFT)

#define BE_CLKR_VAL(NR)			((NR - 1) << BE_CLKR_SHFT)
#define BE_CLKF_VAL(NF)			((NF - 1) << BE_CLKF_SHFT)
#define BE_CLKOD_VAL(OD)		((OD - 1) << BE_CLKOD_SHFT)

#define BE_PLL_CLK_VAL(NR, NF, OD)	\
	(BE_CLKR_VAL(NR) | BE_CLKF_VAL(NF) | BE_CLKOD_VAL(OD))

#define BE_PLL_DIV_MASK 		0x01FFFFFC
#define BE_PLL_LATENCY			100000000 /* ns */
#define BE_PLL_FREQ_STEP		25000000

static DEFINE_SPINLOCK(clk_lock);

struct be_clk_pll {
	struct clk_hw   hw;
	void __iomem    *reg;
	spinlock_t      *lock;
	const char      *name;
	unsigned int 	latency; /* ns */
	unsigned int    min, max, step;
};
#define to_be_clk_pll(_hw) container_of(_hw, struct be_clk_pll, hw)

/*
 * Common functions
 */
static inline unsigned int be_clk_read(void *csr)
{
        return readl(csr);
}

static inline void be_clk_write(unsigned int data, void *csr)
{
        return writel(data, csr);
}

static int be_clk_pll_reset(struct clk_hw *hw)
{
	struct be_clk_pll *pllclk = to_be_clk_pll(hw);
	unsigned int reg, count;

	reg = be_clk_read(pllclk->reg);
	reg |= BE_CLK_RESET_MASK;
	be_clk_write(reg, pllclk->reg);
	wmb();

	count = 50;
	do {
		udelay(pllclk->latency / 1000);
		reg = be_clk_read(pllclk->reg);
	} while (!(reg & BE_CLK_LOCK_MASK) && --count);

	if (!(reg & BE_CLK_LOCK_MASK))
		return -ETIMEDOUT;

	return 0;
}

static int be_clk_pll_is_enabled(struct clk_hw *hw)
{
	struct be_clk_pll *pllclk = to_be_clk_pll(hw);
	unsigned int reg;

	reg = be_clk_read(pllclk->reg);

	return !!(reg & BE_CLK_ENABLE_MASK);
}

static int be_clk_pll_enable(struct clk_hw *hw)
{
	struct be_clk_pll *pllclk = to_be_clk_pll(hw);
	unsigned int reg;

	reg = be_clk_read(pllclk->reg);
	reg |= BE_CLK_ENABLE_MASK;
	be_clk_write(reg, pllclk->reg);
	wmb();

	return 0;
}

static unsigned long be_clk_pll_recalc_rate(struct clk_hw *hw,
                                unsigned long parent_rate)
{
	struct be_clk_pll *pllclk = to_be_clk_pll(hw);
	unsigned long fref, fout;
	unsigned int reg, nr, nf, od;

	/* Read pll ctrl reg */
	reg = be_clk_read(pllclk->reg);
	/* Fetch pll parameters */
	nr = BE_RD_CLKR(reg) + 1;
	nf = BE_RD_CLKF(reg) + 1;
	od = BE_RD_CLKOD(reg) + 1;
	/* ref dividers */
	fref = parent_rate / nr / od;
	/* pll multiplier */
	fout = fref * nf;

	return fout;
}

long be_clk_pll_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
	struct be_clk_pll *pllclk = to_be_clk_pll(hw);

	if (!pllclk->max) {
		rate = be_clk_pll_recalc_rate(hw, *parent_rate);
		pllclk->max = rate;
		pllclk->min = rate;
	}

	if (rate > pllclk->max)
		return pllclk->max;

	if (rate < pllclk->min)
		return pllclk->min;

	return pllclk->min + ((rate - pllclk->min) / pllclk->step) *
		pllclk->step;
}

int be_clk_pll_set_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	struct be_clk_pll *pllclk = to_be_clk_pll(hw);
	unsigned int reg, nf, od, mul;

	mul = (rate / parent_rate) & 0x7f;
	od = ((68 / mul) << 1) & 0x3f; 
	nf = (mul * od) & 0x1fff;

	reg = be_clk_read(pllclk->reg);
	reg &= ~BE_PLL_DIV_MASK;
	reg |= 	BE_PLL_CLK_VAL(1, nf, od);

	be_clk_write(reg, pllclk->reg);
	wmb();

	return be_clk_pll_reset(hw);
}

const struct clk_ops be_clk_pll_ops = {
		.enable      = be_clk_pll_enable,
		.is_enabled  = be_clk_pll_is_enabled,
		.recalc_rate = be_clk_pll_recalc_rate,
		.round_rate  = be_clk_pll_round_rate,
		.set_rate    = be_clk_pll_set_rate,
};

static __init int be_clk_pll_setup(struct device_node *np,
	struct be_clk_pll *pmuclk)
{

	if (of_property_read_u32(np, "clock-latency",
				&pmuclk->latency))
		pmuclk->latency = BE_PLL_LATENCY;

	if (of_property_read_u32_index(np, "clock-frequency-range", 0,
		&pmuclk->min))
		pmuclk->min = 0;
	if (of_property_read_u32_index(np, "clock-frequency-range", 1,
		&pmuclk->max))
		pmuclk->max = 0;
	if (of_property_read_u32_index(np, "clock-frequency-range", 2,
		&pmuclk->step) || !pmuclk->step)
		pmuclk->step = BE_PLL_FREQ_STEP;

	if (pmuclk->min > pmuclk->max)
		return -EINVAL;

	return 0;
}

static __init void be_pllclk_init(struct device_node *np)
{
	struct clk *clk;
	struct clk_init_data init;
	struct be_clk_pll *pmuclk;
	const char *clk_name = np->name;
	const char *parent_name;
	void *res;

	/* allocate the APM clock structure */
	pmuclk = kzalloc(sizeof(*pmuclk), GFP_KERNEL);
	if (!pmuclk) {
		pr_err("PMU: Could not allocate clock %s\n", np->full_name);
		return;
	}

	res = of_iomap(np, 0);
	if (res == NULL) {
		pr_err("PMU: Unable to map CSR register for %s\n", np->full_name);
		goto __err;
	}

	if (be_clk_pll_setup(np, pmuclk)) {
		pr_err("PMU: Unable setup clock %s\n", np->full_name);
		goto __err;
	}

	/* Get clock name */
	of_property_read_string(np, "clock-output-names", &clk_name);
	if (!clk_name)
		clk_name = np->full_name;

	/* Set clock init parameters */
	init.name = clk_name;
	init.ops = &be_clk_pll_ops;
	init.flags = CLK_SET_RATE_NO_REPARENT | CLK_IGNORE_UNUSED;
	parent_name = of_clk_get_parent_name(np, 0);
	init.parent_names = &parent_name;
	init.num_parents = 1;

	/* Baikal pll parameters */
	pmuclk->reg = res;
	pmuclk->lock = &clk_lock;
	pmuclk->hw.init = &init;
	pmuclk->name = clk_name;

	clk = clk_register(NULL, &pmuclk->hw);
	if (IS_ERR(clk)) {
		pr_err("PMU: could not register clk %s\n", clk_name);
		goto __err;
	}

	of_clk_add_provider(np, of_clk_src_simple_get, clk);
	clk_register_clkdev(clk, clk_name, NULL);

	pr_debug("PMU: Add %s PLL clock \n", clk_name);

	return;

__err:
	kfree(pmuclk);
}
CLK_OF_DECLARE(be_pll_clock, "be,pmu-pll-clock", be_pllclk_init);

struct be_dev_params {
	unsigned int 	width;		/* Divider width */
	unsigned int    nobypass; 	/* Disable clock div=1 */
};

struct be_clk {
	struct clk_hw   hw;
	const char      *name;
	spinlock_t      *lock;
	void __iomem    *reg;
	struct be_dev_params params;
};

#define to_be_clk(_hw) container_of(_hw, struct be_clk, hw)

static int be_clk_enable(struct clk_hw *hw)
{
	struct be_clk *pclk = to_be_clk(hw);
	unsigned long flags = 0;
	unsigned int data;
	/* Lock clock */
	if (pclk->lock)
		spin_lock_irqsave(pclk->lock, flags);
	/* If clock valid */
	if (pclk->reg != NULL) {
		/* Debug info */
		pr_debug("%s clock enabled\n", pclk->name);
		/* Get CSR register */
		data = be_clk_read(pclk->reg);
		/* Enable the clock */
		data |= BE_CLK_ENABLE_MASK;
		/* Set CSR register */
		be_clk_write(data, pclk->reg);
		/* Debug info */
		pr_debug("%s clock PADDR base 0x%08lX clk value 0x%08X\n",
			pclk->name, __pa(pclk->reg), data);
	}
	/* Unlock clock */
	if (pclk->lock)
		spin_unlock_irqrestore(pclk->lock, flags);
	/* Return success */
	return 0;
}

static void be_clk_disable(struct clk_hw *hw)
{
	struct be_clk *pclk = to_be_clk(hw);
	unsigned long flags = 0;
	unsigned int data;
	/* Lock clock */
	if (pclk->lock)
		spin_lock_irqsave(pclk->lock, flags);
	/* If clock valid */
	if (pclk->reg != NULL) {
		/* Debug info */
		pr_debug("%s clock disabled\n", pclk->name);
		/* Get CSR register */
		data = be_clk_read(pclk->reg);
		/* Disable the clock */
		data &= ~BE_CLK_ENABLE_MASK;
		/* Set CSR register */
		be_clk_write(data, pclk->reg);
		/* Debug info */
		pr_debug("%s clock PADDR base 0x%08lX clk value 0x%08X\n",
			pclk->name, __pa(pclk->reg), data);
	}
	/* Unlock clock */
	if (pclk->lock)
		spin_unlock_irqrestore(pclk->lock, flags);
}

static int be_clk_is_enabled(struct clk_hw *hw)
{
	struct be_clk *pclk = to_be_clk(hw);
	unsigned int data = 0;

	/* If clock valid */
	if (pclk->reg != NULL) {
		/* Debug info */
		pr_debug("%s clock checking\n", pclk->name);
		/* Get CSR register */
		data = be_clk_read(pclk->reg);
		/* Debug info */
		pr_debug("%s clock PADDR base 0x%08lX clk value 0x%08X\n",
			pclk->name, __pa(pclk->reg), data);
		/* Debug info */
		pr_debug("%s clock is %sabled\n", pclk->name,
			data & BE_CLK_ENABLE_MASK ? "en" : "dis");
	}
	/* Enabled and not controlled */
	else
		return 1;
	return data & BE_CLK_ENABLE_MASK ? 1 : 0;
}

static unsigned long be_clk_recalc_rate(struct clk_hw *hw,
                                unsigned long parent_rate)
{
	struct be_clk *pclk = to_be_clk(hw);
	unsigned int data;

	/* If clock valid */
	if ((pclk->reg != NULL) &&
	    (pclk->params.width != 0)) {
		/* Get CSR register */
		data = be_clk_read(pclk->reg);
		/* Apply global mask and shift data */
		data = (data & BE_CLK_DIV_MASK) >> BE_DIV_SHFT;
		/* Apply divider width mask */
		data &= (1 << pclk->params.width) - 1;
		/* Debug info */
		pr_debug("%s clock recalc rate %ld parent %ld\n",
				pclk->name, parent_rate / data, parent_rate);
		return parent_rate / data;
	} else {
		pr_debug("%s clock recalc rate %ld parent %ld\n",
			pclk->name, parent_rate, parent_rate);
		return parent_rate;
	}
}

static int be_clk_set_rate(struct clk_hw *hw, unsigned long rate,
								unsigned long parent_rate)
{
	struct be_clk *pclk = to_be_clk(hw);
	unsigned long flags = 0;
	unsigned int data;
	unsigned int divider;
	/* Lock clock */
	if (pclk->lock)
		spin_lock_irqsave(pclk->lock, flags);
	/* If clock valid */
	if ((pclk->reg != NULL) &&
	    (pclk->params.width != 0)) {
		/* Let's compute the divider */
		if (rate > parent_rate)
			rate = parent_rate;
		/* Calc divider rounded down */
		divider = parent_rate / rate;
		/* Apply divider width mask */
		divider &= (1 << pclk->params.width) - 1;
		/* Why so may be ? */
		if (!divider)
			divider = 1;
		/* Check nobypass flag */
		if ((divider == 1) && pclk->params.nobypass)
			divider = 2;
		/* Get current state */
		data = be_clk_read(pclk->reg);
		/* Clear divide field */
		data &= ~BE_CLK_DIV_MASK;
		/* Set new divider */
		data |= divider << BE_DIV_SHFT;
		/* Set new value */
		be_clk_write(data, pclk->reg);
		/* Set restart pulse */
		data |= BE_CLK_SET_MASK;
		/* Restart divider */
		be_clk_write(data, pclk->reg);
		/* Debug info */
		pr_debug("%s clock set rate %ld\n", pclk->name,
				parent_rate / divider);
	} else {
		/* bypass mode */
		divider = 1;
	}
	/* Unlock clock */
	if (pclk->lock)
		spin_unlock_irqrestore(pclk->lock, flags);
	/* Return new rate */
	return parent_rate / divider;
}

static long be_clk_round_rate(struct clk_hw *hw, unsigned long rate,
                                unsigned long *prate)
{
	struct be_clk *pclk = to_be_clk(hw);
	unsigned long parent_rate = *prate;
	unsigned int divider;
	/* If clock valid */
	if (pclk->reg) {
		/* Let's compute the divider */
		if (rate > parent_rate)
			rate = parent_rate;
		/* Calc divider rounded down */
		divider = parent_rate / rate;
	} else {
		divider = 1;
	}
	/* Return actual freq */
	return parent_rate / divider;
}

const struct clk_ops be_clk_ops = {
        .enable = be_clk_enable,
        .disable = be_clk_disable,
        .is_enabled = be_clk_is_enabled,
        .recalc_rate = be_clk_recalc_rate,
        .set_rate = be_clk_set_rate,
        .round_rate = be_clk_round_rate,
};

static struct clk *be_register_clk(struct device *dev,
				const char *name, const char *parent_name,
				struct be_dev_params *params, void __iomem *reg,
				spinlock_t *lock)
{
	struct be_clk *pmuclk;
	struct clk *clk;
	struct clk_init_data init;
	int rc;

	/* Allocate the APM clock structure */
	pmuclk = kzalloc(sizeof(*pmuclk), GFP_KERNEL);
	if (!pmuclk) {
		/* Error */
		pr_err("%s: could not allocate PMU clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}
	/* Setup clock init structure */
	init.name = name;
	init.ops = &be_clk_ops;
	init.flags = 0;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;
	/* Setup IP clock structure */
	pmuclk->reg = reg;
	pmuclk->name = name;
	pmuclk->lock = lock;
	pmuclk->hw.init = &init;
	pmuclk->params = *params;

	/* Register the clock */
	clk = clk_register(dev, &pmuclk->hw);
	if (IS_ERR(clk)) {
		/* Error */
		pr_err("%s: could not register clk %s\n", __func__, name);
		/* Free memory */
		kfree(pmuclk);
		return clk;
	}

	/* Register the clock for lookup */
	rc = clk_register_clkdev(clk, name, NULL);
	if (rc != 0) {
		/* Error */
		pr_err("%s: could not register lookup clk %s\n",
			__func__, name);
	}
	return clk;
}

static void __init be_devclk_init(struct device_node *np)
{
	const char *clk_name = np->full_name;
	struct clk *clk;
	struct be_dev_params params;
	void *reg;
	int rc;

	/* Check if the entry is disabled */
	if (!of_device_is_available(np))
		return;

	/* Remap ctrl reg mem */
	reg = of_iomap(np, 0);
	if (reg == NULL) {
		/* Error */
		pr_err("Unable to map CSR register for %s\n", np->full_name);
		return;
	}
	/* Check nobypass property */
	params.nobypass = of_property_read_bool(np, "nobypass");
	/* Get divider width */
	if (of_property_read_u32(np, "divider-width", &params.width))
				params.width = BE_CLK_DIV_MAX_WIDTH;
	/* Get clock name */
	of_property_read_string(np, "clock-output-names", &clk_name);
	/* Register clock */
	clk = be_register_clk(NULL, clk_name, of_clk_get_parent_name(np, 0),
						&params, reg, &clk_lock);
	/* Check error */
	if (IS_ERR(clk))
		goto err;
	/* Debug error */
	pr_debug("Add %s clock\n", clk_name);
	/* Add clock provider */
	rc = of_clk_add_provider(np, of_clk_src_simple_get, clk);
	if (rc != 0)
		pr_err("%s: could register provider clk %s\n", __func__,
				np->full_name);
	return;
err:
	if (reg)
		iounmap(reg);
}
CLK_OF_DECLARE(be_dev_clock, "be,pmu-device-clock", be_devclk_init);

MODULE_VERSION(VERSION);
MODULE_AUTHOR("Dmitry Dunaev");
MODULE_DESCRIPTION("Baikal Electronics clock driver");
MODULE_LICENSE("GPL");
