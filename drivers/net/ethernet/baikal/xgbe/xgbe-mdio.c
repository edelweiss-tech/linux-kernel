/*
 *
 * This file is available to you under your choice of the following two
 * licenses:
 *
 * License 1: GPLv2
 *
 * Copyright (c) 2014 Advanced Micro Devices, Inc.
 *
 * This file is free software; you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or (at
 * your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *     The Synopsys DWC ETHER XGMAC Software Driver and documentation
 *     (hereinafter "Software") is an unsupported proprietary work of Synopsys,
 *     Inc. unless otherwise expressly agreed to in writing between Synopsys
 *     and you.
 *
 *     The Software IS NOT an item of Licensed Software or Licensed Product
 *     under any End User Software License Agreement or Agreement for Licensed
 *     Product with Synopsys or any supplement thereto.  Permission is hereby
 *     granted, free of charge, to any person obtaining a copy of this software
 *     annotated with this license and the Software, to deal in the Software
 *     without restriction, including without limitation the rights to use,
 *     copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *     of the Software, and to permit persons to whom the Software is furnished
 *     to do so, subject to the following conditions:
 *
 *     The above copyright notice and this permission notice shall be included
 *     in all copies or substantial portions of the Software.
 *
 *     THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS"
 *     BASIS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *     TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *     PARTICULAR PURPOSE ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS
 *     BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *     CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *     SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *     INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *     CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *     ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *     THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * License 2: Modified BSD
 *
 * Copyright (c) 2014 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Advanced Micro Devices, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * This file incorporates work covered by the following copyright and
 * permission notice:
 *     The Synopsys DWC ETHER XGMAC Software Driver and documentation
 *     (hereinafter "Software") is an unsupported proprietary work of Synopsys,
 *     Inc. unless otherwise expressly agreed to in writing between Synopsys
 *     and you.
 *
 *     The Software IS NOT an item of Licensed Software or Licensed Product
 *     under any End User Software License Agreement or Agreement for Licensed
 *     Product with Synopsys or any supplement thereto.  Permission is hereby
 *     granted, free of charge, to any person obtaining a copy of this software
 *     annotated with this license and the Software, to deal in the Software
 *     without restriction, including without limitation the rights to use,
 *     copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *     of the Software, and to permit persons to whom the Software is furnished
 *     to do so, subject to the following conditions:
 *
 *     The above copyright notice and this permission notice shall be included
 *     in all copies or substantial portions of the Software.
 *
 *     THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS"
 *     BASIS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *     TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 *     PARTICULAR PURPOSE ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS
 *     BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *     CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *     SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *     INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *     CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *     ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *     THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/kmod.h>
#include <linux/mdio.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/bitops.h>
#include <linux/jiffies.h>
#include <linux/clk.h>

#include "xgbe.h"
#include "xgbe-common.h"

static int be_xgbe_an_enable_kr_training(struct xgbe_prv_data *pdata)
{
	int ret;
	DBGPR("%s\n", __FUNCTION__);
	return 0;	// ???

	ret = XMDIO_READ(pdata, MDIO_MMD_PMAPMD, MDIO_PMA_10GBR_PMD_CTRL);
	if (ret < 0)
		return ret;

	ret |= 0x02;
	XMDIO_WRITE(pdata, MDIO_MMD_PMAPMD, MDIO_PMA_10GBR_PMD_CTRL, ret);

	return 0;
}

static int be_xgbe_phy_pcs_power_cycle(struct xgbe_prv_data *pdata)
{
	int ret;
	DBGPR("%s\n", __FUNCTION__);
	return 0;

	ret = XMDIO_READ(pdata, MDIO_MMD_PCS, MDIO_CTRL1);
	if (ret < 0)
		return ret;

	ret |= MDIO_CTRL1_LPOWER;
	XMDIO_WRITE(pdata, MDIO_MMD_PCS, MDIO_CTRL1, ret);

	usleep_range(75, 100);

	ret &= ~MDIO_CTRL1_LPOWER;
	XMDIO_WRITE(pdata, MDIO_MMD_PCS, MDIO_CTRL1, ret);

	return 0;
}

static int be_xgbe_phy_xgmii_mode(struct xgbe_prv_data *pdata)
{
	int ret;
	DBGPR("%s\n", __FUNCTION__);
	/* Enable KR training */
	ret = be_xgbe_an_enable_kr_training(pdata);
	if (ret < 0)
		return ret;

	/* Set PCS to KR/10G speed */
	ret = XMDIO_READ(pdata, MDIO_MMD_PCS, MDIO_CTRL2);
	if (ret < 0)
		return ret;

	ret &= ~MDIO_PCS_CTRL2_TYPE;
	ret |= MDIO_PCS_CTRL2_10GBR;
	XMDIO_WRITE(pdata, MDIO_MMD_PCS, MDIO_CTRL2, ret);

	ret = XMDIO_READ(pdata, MDIO_MMD_PCS, MDIO_CTRL1);
	if (ret < 0)
		return ret;

	ret &= ~MDIO_CTRL1_SPEEDSEL;
	ret |= MDIO_CTRL1_SPEED10G;
	XMDIO_WRITE(pdata, MDIO_MMD_PCS, MDIO_CTRL1, ret);

	ret = be_xgbe_phy_pcs_power_cycle(pdata);
	if (ret < 0)
	return ret;
	/* TBD */
	//priv->mode = BE_XGBE_MODE_KR;

	return 0;
}

static int __maybe_unused be_xgbe_phy_soft_reset(struct xgbe_prv_data *pdata)
{
	int count, ret;
	DBGPR("%s\n", __FUNCTION__);

	ret = XMDIO_READ(pdata, MDIO_MMD_PCS, MDIO_CTRL1);
	if (ret < 0)
		return ret;

	ret |= MDIO_CTRL1_RESET;
	XMDIO_WRITE(pdata, MDIO_MMD_PCS, MDIO_CTRL1, ret);

	count = 50;
	do {
		msleep(20);
		ret = XMDIO_READ(pdata, MDIO_MMD_PCS, MDIO_CTRL1);
		if (ret < 0)
			return ret;
	} while ((ret & MDIO_CTRL1_RESET) && --count);

	if (ret & MDIO_CTRL1_RESET)
		return -ETIMEDOUT;

	return 0;
}

static int be_xgbe_phy_config_aneg(struct xgbe_prv_data *pdata)
{
	int reg;

	set_bit(XGBE_LINK_INIT, &pdata->dev_state);
	pdata->link_check = jiffies;

	DBGPR("%s\n", __FUNCTION__);
	reg = XMDIO_READ(pdata, MDIO_MMD_AN, MDIO_CTRL1);
	if (reg < 0)
		return reg;
	/* Disable autonegotiation with tranceiver */
	/************ !!! WORKAROUND !!! ***********/
	/* Disable autonegotiation in any case */
	//if (priv->xmit) {
		reg &= ~MDIO_AN_CTRL1_ENABLE;
		pdata->phy.autoneg = AUTONEG_DISABLE;
	//}
	//else {
	//	reg |= MDIO_AN_CTRL1_ENABLE;
	//	phydev->autoneg = AUTONEG_ENABLE;
	//}

	XMDIO_WRITE(pdata, MDIO_MMD_AN, MDIO_CTRL1, reg);

	return 0;
}

static int be_xgbe_phy_config_init(struct xgbe_prv_data *pdata)
{
	int ret = 0;
	DBGPR("%s\n", __FUNCTION__);
	/* Initialize supported features */
	pdata->phy.supported = SUPPORTED_Autoneg;
	pdata->phy.supported |= SUPPORTED_Pause | SUPPORTED_Asym_Pause;
	pdata->phy.supported |= SUPPORTED_Backplane | SUPPORTED_10000baseKX4_Full;
	pdata->phy.supported |= SUPPORTED_10000baseKR_Full | SUPPORTED_10000baseR_FEC;
	switch (pdata->speed_set) {
	case BE_XGBE_PHY_SPEEDSET_1000_10000:
		pdata->phy.supported |= SUPPORTED_1000baseKX_Full;
		break;
	case BE_XGBE_PHY_SPEEDSET_2500_10000:
		pdata->phy.supported |= SUPPORTED_2500baseX_Full;
		break;
	}

	if (pdata->phydev)
		pdata->phy.supported |= SUPPORTED_10000baseT_Full;

	pdata->phy.advertising = pdata->phy.supported;
	pdata->phy.pause_autoneg = 0;
	pdata->phy.tx_pause = 0;
	pdata->phy.rx_pause = 0;

	if (pdata->ext_clk) {
		DBGPR("%s EXT CLOCK\n", __FUNCTION__);
		/* Switch XGMAC PHY PLL to use extrnal ref clock from pad */
		ret = XMDIO_READ(pdata, MDIO_MMD_PMAPMD, VR_XS_PMA_MII_Gen5_MPLL_CTRL);
		ret &= ~(VR_XS_PMA_MII_Gen5_MPLL_CTRL_REF_CLK_SEL_bit);
		XMDIO_WRITE(pdata, MDIO_MMD_PMAPMD, VR_XS_PMA_MII_Gen5_MPLL_CTRL, ret);
		wmb();
		/* Turn off internal XGMAC PHY clock */
		clk_disable_unprepare(pdata->sysclk);
	}

	/* Make vendor specific soft reset */
	ret = XMDIO_READ(pdata, MDIO_MMD_PCS, VR_XS_PCS_DIG_CTRL1);
	ret |= VR_XS_PCS_DIG_CTRL1_VR_RST_Bit;
	XMDIO_WRITE(pdata, MDIO_MMD_PCS, VR_XS_PCS_DIG_CTRL1, ret);
	wmb();

	/* Wait reset finish */
	do {
		usleep_range(500, 600);
		ret = XMDIO_READ(pdata, MDIO_MMD_PCS, VR_XS_PCS_DIG_CTRL1);
	} while ( (ret & VR_XS_PCS_DIG_CTRL1_VR_RST_Bit) != 0 );


			DBGPR("%s %x\n", __FUNCTION__, ret);
	/*
	 * Wait for the RST (bit 15) of the “SR XS or PCS MMD Control1” Register is 0.
	 * This bit is self-cleared when Bits[4:2] in VR XS or PCS MMD Digital
	 * Status Register are equal to 3’b100, that is, Tx/Rx clocks are stable
	 * and in Power_Good state.
	 */
	do {
		usleep_range(500, 600);
		ret = XMDIO_READ(pdata, MDIO_MMD_PCS, SR_XC_or_PCS_MMD_Control1);
	} while ( (ret & SR_XC_or_PCS_MMD_Control1_RST_Bit) != 0 );

	/*
	 * This bit is self-cleared when Bits[4:2] in VR XS or PCS MMD Digital
	 * Status Register are equal to 3’b100, that is, Tx/Rx clocks are stable
	 * and in Power_Good state.
	 */
	do {
		usleep_range(500, 600);
		ret = XMDIO_READ(pdata, MDIO_MMD_PCS, DWC_GLBL_PLL_MONITOR);
	} while ( (ret & SDS_PCS_CLOCK_READY_mask) != SDS_PCS_CLOCK_READY_bit );

	/* Turn off and clear interrupts */
	XMDIO_WRITE(pdata, MDIO_MMD_AN, MDIO_AN_INTMASK, 0);
	XMDIO_WRITE(pdata, MDIO_MMD_AN, MDIO_AN_INT, 0);
	wmb();

	be_xgbe_phy_config_aneg(pdata);

	ret = be_xgbe_phy_xgmii_mode(pdata);
	if (ret < 0)
		return ret;

	return 0;
}

static int be_xgbe_phy_aneg_done(struct xgbe_prv_data *pdata)
{
	int reg;
	DBGPR("%s\n", __FUNCTION__);
	reg = XMDIO_READ(pdata, MDIO_MMD_AN, MDIO_STAT1);
	if (reg < 0)
		return reg;

	return (reg & MDIO_AN_STAT1_COMPLETE) ? 1 : 0;
}

static int be_xgbe_phy_update_link(struct xgbe_prv_data *pdata)
{
	int new_state = 0;

	if (pdata->phy.link) {
		/* Flow control support */
		pdata->pause_autoneg = pdata->phy.pause_autoneg;

		if (pdata->tx_pause != pdata->phy.tx_pause) {
			new_state = 1;
			pdata->hw_if.config_tx_flow_control(pdata);
			pdata->tx_pause = pdata->phy.tx_pause;
		}

		if (pdata->rx_pause != pdata->phy.rx_pause) {
			new_state = 1;
			pdata->hw_if.config_rx_flow_control(pdata);
			pdata->rx_pause = pdata->phy.rx_pause;
		}

		/* Speed support */
		if (pdata->phy_speed != pdata->phy.speed) {
			new_state = 1;
			pdata->phy_speed = pdata->phy.speed;
		}

		if (pdata->phy_link != pdata->phy.link) {
			new_state = 1;
			pdata->phy_link = pdata->phy.link;
		}
	} else if (pdata->phy_link) {
		new_state = 1;
		pdata->phy_link = 0;
		pdata->phy_speed = SPEED_UNKNOWN;
	}

	return 0;
}

static void be_xgbe_phy_read_status(struct xgbe_prv_data *pdata)
{

	int reg, link_aneg;

	pdata->phy.link = 1;

	if (test_bit(XGBE_LINK_ERR, &pdata->dev_state)) {
		netif_carrier_off(pdata->netdev);

		pdata->phy.link = 0;
		goto update_link;
	}

	link_aneg = (pdata->phy.autoneg == AUTONEG_ENABLE);

	/* Get the link status. Link status is latched low, so read
	 * once to clear and then read again to get current state
	 */

	/* Check tranceiver status */
	if (pdata->phydev) {
		pdata->phydev->drv->read_status(pdata->phydev);
		if (!pdata->phydev->link){
			pdata->phydev->link = 0;
			pdata->phy.link &= pdata->phydev->link;
			DBGPR("%s PHY %x\n", __FUNCTION__, pdata->phy.link);
		}
	}
	reg = XMDIO_READ(pdata, MDIO_MMD_PCS, MDIO_STAT1);
	pdata->phy.link &= (reg & MDIO_STAT1_LSTATUS) ? 1 : 0;
	DBGPR("%s PCS %x\n", __FUNCTION__, reg);

	reg = XMDIO_READ(pdata, MDIO_MMD_PMAPMD, MDIO_STAT1);
	DBGPR("%s PMA %x\n", __FUNCTION__, reg);
	pdata->phy.link &= (reg & MDIO_STAT1_LSTATUS) ? 1 : 0;

	if (pdata->phy.link) {
		DBGPR("%s link on\n", __FUNCTION__);
		if (link_aneg && !be_xgbe_phy_aneg_done(pdata)) {
			//xgbe_check_link_timeout(pdata);
			return ;
		}

		//xgbe_phy_status_aneg(pdata);

		if (test_bit(XGBE_LINK_INIT, &pdata->dev_state))
			clear_bit(XGBE_LINK_INIT, &pdata->dev_state);

		netif_carrier_on(pdata->netdev);
	} else {
		DBGPR("%s link off\n", __FUNCTION__);
		if (test_bit(XGBE_LINK_INIT, &pdata->dev_state)) {
			//xgbe_check_link_timeout(pdata);

			if (link_aneg)
				return;
		}

		//xgbe_phy_status_aneg(pdata);

		netif_carrier_off(pdata->netdev);
	}

update_link:
	be_xgbe_phy_update_link(pdata);
}

static int be_xgbe_phy_resume(struct xgbe_prv_data *pdata)
{
	int ret;
	DBGPR("%s\n", __FUNCTION__);
	//mutex_lock(&phydev->lock);

	ret = XMDIO_READ(pdata, MDIO_MMD_PCS, MDIO_CTRL1);
	if (ret < 0)
		return 1;

	ret &= ~MDIO_CTRL1_LPOWER;
	XMDIO_WRITE(pdata, MDIO_MMD_PCS, MDIO_CTRL1, ret);

	ret = 0;

//unlock:
	//mutex_unlock(&phydev->lock);

	return ret;
}

static int be_xgbe_phy_suspend(struct xgbe_prv_data *pdata)
{
	int ret;

	DBGPR("%s\n", __FUNCTION__);
//	mutex_lock(&phydev->lock);

	ret = XMDIO_READ(pdata, MDIO_MMD_PCS, MDIO_CTRL1);
	if (ret < 0)
		return 1;

	ret |= MDIO_CTRL1_LPOWER;
	XMDIO_WRITE(pdata, MDIO_MMD_PCS, MDIO_CTRL1, ret);

	ret = 0;

//unlock:
//	mutex_unlock(&phydev->lock);

	return ret;
}

static void be_xgbe_phy_stop(struct xgbe_prv_data *pdata)
{
	netif_dbg(pdata, link, pdata->netdev, "stopping PHY\n");

	/* Disable auto-negotiation */
	//xgbe_disable_an(pdata);

	/* Disable auto-negotiation interrupts */
	XMDIO_WRITE(pdata, MDIO_MMD_AN, MDIO_AN_INTMASK, 0);

//	devm_free_irq(pdata->dev, pdata->an_irq, pdata);

	pdata->phy.link = 0;
	netif_carrier_off(pdata->netdev);

	be_xgbe_phy_update_link(pdata);
}

static int be_xgbe_xmit_probe(struct xgbe_prv_data *pdata)
{

	struct device_node *xmit_node;
	struct phy_device *phydev;
	struct device *dev = pdata->dev;
	int ret;
	/* Retrieve the xmit-handle */

	xmit_node = of_parse_phandle(dev->of_node, "phy-handle", 0);
	if (!xmit_node) {
		dev_info(dev, "no phy-handle, work in KR/KX mode\n");
		return -ENODEV;
	}
	phydev = of_phy_find_device(xmit_node);
	if (!phydev)
		return -EINVAL;
	ret = phy_init_hw(phydev);
	if (ret < 0)
		return ret;

	phydev->speed = SPEED_10000;
	phydev->duplex = DUPLEX_FULL;
	phydev->dev.of_node = xmit_node;

	pdata->phydev = phydev;
	/* refcount is held by phy_attach_direct() on success */
	put_device(&phydev->dev);

#if 0
	/* Add sysfs link to netdevice */
	ret = sysfs_create_link(&(xmit->bus->dev.kobj), &(phydev->attached_dev->dev.kobj), "traceiver");
	if (ret)
		dev_warn(&xmit->dev, "sysfs link to netdevice failed\n");
#endif
	return 0;
}

void xgbe_init_function_ptrs_phy(struct xgbe_phy_if *phy_if)
{
	phy_if->phy_init        = be_xgbe_phy_config_init;

	phy_if->phy_reset       = be_xgbe_phy_soft_reset;

	phy_if->phy_probe       = be_xgbe_xmit_probe;
	phy_if->phy_stop        = be_xgbe_phy_stop;
	phy_if->phy_suspend     = be_xgbe_phy_suspend;
	phy_if->phy_resume      = be_xgbe_phy_resume;

	phy_if->phy_status      = be_xgbe_phy_read_status;
	phy_if->phy_config_aneg = be_xgbe_phy_config_aneg;
}