/*
 * I2C (ddc) support fot sm750
 *
 * Copyright (c) 2017 T-Platforms
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2.
 */

#include <linux/fb.h>
#include "ddk750_reg.h"
#include "sm750.h"

static void sm750_ddc_setscl(void *data, int val)
{
	struct sm750_ddc *ddc = data;
	u32 reg, mask;

	mask = 1 << ddc->i2c_scl_pin;
	reg = readl(ddc->regs + GPIO_DATA_DIRECTION);
	if (val) {
		/* set direction to input to allow slave to stretch clock */
		if (reg & mask)
			writel(reg & ~mask, ddc->regs + GPIO_DATA_DIRECTION);
	} else {
		if (!(reg & mask))
			writel(reg | mask, ddc->regs + GPIO_DATA_DIRECTION);
	}
}
 
static void sm750_ddc_setsda(void *data, int val)
{
	struct sm750_ddc *ddc = data;
	u32 reg, mask;

	mask = 1 << ddc->i2c_sda_pin;

	reg = readl(ddc->regs + GPIO_DATA_DIRECTION);
	if (val) {
		if (reg & mask)
			writel(reg & ~mask, ddc->regs + GPIO_DATA_DIRECTION);
	} else {
		if (!(reg & mask))
			writel(reg | mask, ddc->regs + GPIO_DATA_DIRECTION);
	}
}
 
static int sm750_ddc_getscl(void *data)
{
	struct sm750_ddc *ddc = data;
	u32 reg, mask;

	mask = 1 << ddc->i2c_scl_pin;
	reg = readl(ddc->regs + GPIO_DATA_DIRECTION);
	if (reg & mask)
		writel(reg & ~mask, ddc->regs + GPIO_DATA_DIRECTION);

	reg = readl(ddc->regs + GPIO_DATA);
	return !!(reg & mask);
}
 
static int sm750_ddc_getsda(void *data)
{
	struct sm750_ddc *ddc = data;
	u32 reg, mask;

	mask = 1 << ddc->i2c_sda_pin;
	reg = readl(ddc->regs + GPIO_DATA_DIRECTION);
	if (reg & mask)
		writel(reg & ~mask, ddc->regs + GPIO_DATA_DIRECTION);

	reg = readl(ddc->regs + GPIO_DATA);
	return !!(reg & mask);
}
 
static int sm750_ddc_pre_xfer(struct i2c_adapter *adap)
{
	struct sm750_ddc *ddc = container_of(adap, struct sm750_ddc, ddc_adapter);
	u32 reg, mask;

	mask = (1 << ddc->i2c_scl_pin) | (1 << ddc->i2c_sda_pin);
	reg = readl(ddc->regs + GPIO_MUX);
	/* make sure GPIO MUX is not broken */
	if (reg & mask) { 
		pr_debug("sm750_ddc_pre_xfer: MUX = %08x\n", reg);
		reg &= ~mask;
		writel(reg, ddc->regs + GPIO_MUX);
	}

	/* Set direction to IN. */
	reg = readl(ddc->regs + GPIO_DATA);
	reg &= ~mask;
	writel(reg, ddc->regs + GPIO_DATA);

	/*
	 * Set data to 0, so we can operate with direction only - 
	 * set IN for 1 or set OUT for 0.
	 */
	reg = readl(ddc->regs + GPIO_DATA_DIRECTION);
	reg &= ~mask;
	writel(reg, ddc->regs + GPIO_DATA_DIRECTION);

	return 0;
}

char *sm750_ddc_read_edid(struct i2c_adapter *adap)
{
	struct i2c_client *i2c_cl;
	char *edid_buf;
	int i, c;

	i2c_cl = i2c_new_dummy(adap, 0x50);
	if (IS_ERR_OR_NULL(i2c_cl))
		return NULL;
	edid_buf = kmalloc(256, GFP_KERNEL);
	if (!edid_buf)
		goto out_client;

	for (i = 0; i < 256; i++) {
		c = i2c_smbus_read_byte_data(i2c_cl, i);
		if (c < 0) {
			if (i < 128) {
				goto out_free;
			} else {
				memset(edid_buf + 128, 128, 0);
				break;
			}
		}
		edid_buf[i] = c;
	}

	i2c_unregister_device(i2c_cl);
	return edid_buf;

out_free:
	kfree(edid_buf);
out_client:
	i2c_unregister_device(i2c_cl);
	return NULL;
}

int sm750_ddc_init(struct sm750_ddc *ddc)
{
	u32 reg, mask;
	int ret;

	mask = (1 << ddc->i2c_scl_pin) | (1 << ddc->i2c_sda_pin);
	reg = readl(ddc->regs + GPIO_MUX);
	reg &= ~mask;
	writel(reg, ddc->regs + GPIO_MUX);
	ddc->ddc_adapter.owner = THIS_MODULE;
	ddc->ddc_adapter.class = I2C_CLASS_DDC;
	ddc->ddc_adapter.algo_data = &ddc->ddc_algo;
	ddc->ddc_algo.setscl = sm750_ddc_setscl;
	ddc->ddc_algo.setsda = sm750_ddc_setsda;
	ddc->ddc_algo.getscl = sm750_ddc_getscl;
	ddc->ddc_algo.getsda = sm750_ddc_getsda;
	ddc->ddc_algo.pre_xfer = sm750_ddc_pre_xfer;
	ddc->ddc_algo.udelay = 10;
	ddc->ddc_algo.timeout = 20;
	ddc->ddc_algo.data = ddc;

	i2c_set_adapdata(&ddc->ddc_adapter, ddc);
	ret = i2c_bit_add_bus(&ddc->ddc_adapter);
	if (ret == 0) {
		ddc->ddc_registered = 1;
		pr_debug("registered ddc bus %s (pins %d/%d, regs %p)\n",
			ddc->ddc_adapter.name, ddc->i2c_scl_pin,
			ddc->i2c_sda_pin, ddc->regs);
	}

	return ret;
}

int sm750_setup_ddc(struct sm750_dev *sm750_dev)
{
	int ret = 0;
	struct sm750_ddc *ddc_p;

	/* DDC0 - primary (panel/DVI output) */
	ddc_p = &sm750_dev->ddc[0];
	strlcpy(ddc_p->ddc_adapter.name, "sm750 DVI i2c",
		sizeof(ddc_p->ddc_adapter.name));
	ddc_p->i2c_scl_pin = 30;
	ddc_p->i2c_sda_pin = 31;
	ddc_p->regs = sm750_dev->pvReg;
	ddc_p->ddc_adapter.dev.parent = &sm750_dev->pdev->dev;
	ret = sm750_ddc_init(ddc_p);
	if (ret)
		return ret;

	ddc_p = &sm750_dev->ddc[1];
	strlcpy(ddc_p->ddc_adapter.name, "sm750 CRT i2c",
		sizeof(ddc_p->ddc_adapter.name));
	ddc_p->i2c_scl_pin = 17;
	ddc_p->i2c_sda_pin = 18;
	ddc_p->regs = sm750_dev->pvReg;
	ddc_p->ddc_adapter.dev.parent = &sm750_dev->pdev->dev;
	ret = sm750_ddc_init(ddc_p);

	return ret;
}

void sm750_remove_ddc(struct sm750_dev *sm750_dev)
{
	struct sm750_ddc *ddc_p;

	ddc_p = &sm750_dev->ddc[0];
	if (ddc_p->ddc_registered) {
		i2c_del_adapter(&ddc_p->ddc_adapter);
		ddc_p->ddc_registered = 0;
	}
	ddc_p = &sm750_dev->ddc[1];
	if (ddc_p->ddc_registered) {
		i2c_del_adapter(&ddc_p->ddc_adapter);
		ddc_p->ddc_registered = 0;
	}
}

