/*
 * Platform driver for the Baikal SoC PCIe eDMA Controller
 *
 * Copyright (C) 2007-2008 Atmel Corporation
 * Copyright (C) 2010-2011 ST Microelectronics
 * Copyright (C) 2013 Intel Corporation
 * Copyright (C) 2017-2018 Baikal Electronics JSC
 *
 * Some parts of this driver are derived from the original dw_dmac.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>

#include <linux/platform_data/dma-baikal.h>
#include "internal.h"

#ifdef CONFIG_MIPS_BAIKAL
struct baikal_dma_chip *baikal_dma_chip;
#endif /* CONFIG_MIPS_BAIKAL */

#define DRV_NAME	"baikal-edma"

#ifdef CONFIG_OF
	static struct baikal_dma_platform_data *
baikal_dma_parse_dt(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct baikal_dma_platform_data *pdata;
	u32 tmp;

	if (!np) {
		dev_err(&pdev->dev, "Missing DT data\n");
		return NULL;
	}

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	return pdata;
}
#else
	static inline struct baikal_dma_platform_data *
baikal_dma_parse_dt(struct platform_device *pdev)
{
	return NULL;
}
#endif

static int edma_probe(struct platform_device *pdev)
{
	struct baikal_dma_chip *chip;
	struct device *dev = &pdev->dev;
	struct resource *mem;
	struct baikal_dma_platform_data *pdata;
	int err;
	int i;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chip->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(chip->regs))
		return PTR_ERR(chip->regs);

	err = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (err)
		return err; 

	pdata = dev_get_platdata(dev);
	if (!pdata)
		pdata = baikal_dma_parse_dt(pdev);

	pdata->wr_channels = BAIKAL_EDMA_WR_CHANNELS;
	pdata->rd_channels = BAIKAL_EDMA_RD_CHANNELS;

	for (i = 0; i < BAIKAL_EDMA_TOTAL_CHANNELS; i++) {
		if (!(chip->irq[i] = platform_get_resource(pdev, IORESOURCE_IRQ, i))) {
			dev_err(&pdev->dev, "There is no IRQ resource specified for channel %d.\n", i);
			return -EINVAL;
		}
	}

	chip->dev = dev;

	pm_runtime_enable(&pdev->dev);

	err = baikal_dma_probe(chip, pdata);
	if (err)
		goto err_baikal_dma_probe;

	/* save a reference to the DMA chip into drvdata */
	/* for PCI devices to retrieve it later */
	/* this will probably work on platforms with traditional PCI software stack */
	/* such as Baikal-M ARM based SoC*/
	platform_set_drvdata(pdev, chip);
#ifdef CONFIG_MIPS_BAIKAL
	/* this is a workaround for legacy PCI stack on Baikal-T1 MIPS based SoC */
	baikal_dma_chip = chip;
#endif /* CONFIG_MIPS_BAIKAL */

	return 0;

err_baikal_dma_probe:
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(chip->clk);
	return err;
}

static int edma_remove(struct platform_device *pdev)
{
	struct baikal_dma_chip *chip = platform_get_drvdata(pdev);

	baikal_dma_remove(chip);
	pm_runtime_disable(&pdev->dev);
	clk_disable_unprepare(chip->clk);

	return 0;
}

static void edma_shutdown(struct platform_device *pdev)
{
	struct baikal_dma_chip *chip = platform_get_drvdata(pdev);

	baikal_dma_disable(chip);
	clk_disable_unprepare(chip->clk);
}

#ifdef CONFIG_OF
static const struct of_device_id edma_of_id_table[] = {
	{ .compatible = "be,baikal-edma" },
	{}
};
MODULE_DEVICE_TABLE(of, baikal_dma_of_id_table);
#endif

#ifdef CONFIG_PM_SLEEP

static int edma_suspend_late(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct baikal_dma_chip *chip = platform_get_drvdata(pdev);

	baikal_dma_disable(chip);
	clk_disable_unprepare(chip->clk);

	return 0;
}

static int edma_resume_early(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct baikal_dma_chip *chip = platform_get_drvdata(pdev);

	clk_prepare_enable(chip->clk);
	return baikal_dma_enable(chip);
}

#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops edma_dev_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(edma_suspend_late, edma_resume_early)
};

static struct platform_driver edma_driver = {
	.probe		= edma_probe,
	.remove		= edma_remove,
	.shutdown       = edma_shutdown,
	.driver = {
		.name	= DRV_NAME,
		.pm	= &edma_dev_pm_ops,
		.of_match_table = of_match_ptr(edma_of_id_table),
	},
};

static int __init edma_init(void)
{
	return platform_driver_register(&edma_driver);
}
subsys_initcall(edma_init);

static void __exit edma_exit(void)
{
	platform_driver_unregister(&edma_driver);
}
module_exit(edma_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Baikal Electronics PCIe eDMA Controller platform driver");
MODULE_ALIAS("platform:" DRV_NAME);
