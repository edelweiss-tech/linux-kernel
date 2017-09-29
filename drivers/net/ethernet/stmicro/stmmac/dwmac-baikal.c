/*
 * Baikal Electronics SoCs DWMAC glue layer
 *
 * Copyright (C) 2015,2016 Baikal Electronics JSC
 * Author:
 *   Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "stmmac.h"
#include "stmmac_platform.h"
#include "dwmac_dma.h"

#define GMAC_GPIO	0x000000e0	/* GPIO register */
#define GMAC_GPIO_GPO0	(1 << 8)	/* 0-output port */

#ifdef CONFIG_MACH_BAIKAL_BFK2
static struct stmmac_dma_ops baikal_dma_ops;

static int baikal_dwmac_dma_reset(void __iomem *ioaddr)
{
	u32 value = readl(ioaddr + DMA_BUS_MODE);
	int limit;

	/* DMA SW reset */
	value |= DMA_BUS_MODE_SFT_RESET;
	writel(value, ioaddr + DMA_BUS_MODE);

	/* Clear PHY reset */
	value = readl(ioaddr + GMAC_GPIO);
	value |= GMAC_GPIO_GPO0;
	writel(value, ioaddr + GMAC_GPIO);

	limit = 10;
	while (limit--) {
		if (!(readl(ioaddr + DMA_BUS_MODE) & DMA_BUS_MODE_SFT_RESET))
			break;
		mdelay(10);
	}
	if (limit < 0)
		return -EBUSY;

	return 0;
}
#endif

static int dwmac_baikal_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	if (pdev->dev.of_node) {
		plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
		if (IS_ERR(plat_dat)) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			return PTR_ERR(plat_dat);
		}
	} else {
		plat_dat = dev_get_platdata(&pdev->dev);
		if (!plat_dat) {
			dev_err(&pdev->dev, "no platform data provided\n");
			return  -EINVAL;
		}

		/* Set default value for multicast hash bins */
		plat_dat->multicast_filter_bins = HASH_TABLE_SIZE;

		/* Set default value for unicast filter entries */
		plat_dat->unicast_filter_entries = 1;
	}

	plat_dat->has_gmac = 1;
	plat_dat->enh_desc = 1;
	plat_dat->tx_coe = 1;
	plat_dat->rx_coe = 1;

	dev_info(&pdev->dev, "Baikal Electronics DWMAC glue driver\n");

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);

#ifdef CONFIG_MACH_BAIKAL_BFK2
	if (!ret) {
		/* change dma_reset method to baikal specific */
		struct net_device *ndev = dev_get_drvdata(&pdev->dev);
		struct stmmac_priv *priv = netdev_priv(ndev);
		if (priv && priv->hw && priv->hw->dma) {
			dev_info(&pdev->dev, "Updating dma_reset (was %p)\n", priv->hw->dma->reset);
			memcpy(&baikal_dma_ops, priv->hw->dma, sizeof(baikal_dma_ops));
			baikal_dma_ops.reset = baikal_dwmac_dma_reset;
			priv->hw->dma = &baikal_dma_ops;
		}
	}
#endif

	return ret;
}

static const struct of_device_id dwmac_baikal_match[] = {
	{ .compatible = "be,dwmac-3.710"},
	{ .compatible = "be,dwmac"},
	{ }
};
MODULE_DEVICE_TABLE(of, dwmac_baikal_match);

static struct platform_driver dwmac_baikal_driver = {
	.probe  = dwmac_baikal_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name           = "baikal-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = of_match_ptr(dwmac_baikal_match),
	},
};
module_platform_driver(dwmac_baikal_driver);

MODULE_DESCRIPTION("Baikal dwmac glue driver");
MODULE_LICENSE("GPL v2");
