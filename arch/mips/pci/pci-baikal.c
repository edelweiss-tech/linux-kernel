/*
 *  Baikal-T SOC platform support code.
 *
 *  Copyright (C) 2015-2017 Baikal Electronics JSC.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 *  BAIKAL MIPS boards specific PCI support.
 */
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>	/* dev_err */
#include <linux/module.h>
#include <linux/of_platform.h>	/* open firmware functioons */
#include <linux/delay.h>

#include <asm/mips-cm.h>
#include <asm/mips-boards/generic.h>
#include "pci-baikal.h"

static struct resource dw_mem_resource = {
	.name	= "DW PCI MEM",
	.start	= PHYS_PCIMEM_BASE_ADDR,
	.end	= PHYS_PCIMEM_LIMIT_ADDR,
	.flags	= IORESOURCE_MEM,
};

static struct resource dw_io_resource = {
	.name	= "DW PCI I/O",
	.start	= PHYS_PCIIO_BASE_ADDR,
	.end	= PHYS_PCIIO_LIMIT_ADDR,
	.flags	= IORESOURCE_IO,
};

static struct resource dw_busn_resource = {
	.name	= "DW PCI busn",
	.start	= PCIE_ROOT_BUS_NUM, /* It's going to be overwritten anyway */
	.end	= 255,
	.flags	= IORESOURCE_BUS,
};

extern struct pci_ops dw_pci_ops;
static int dw_pcie_get_busn(void);

static struct pci_controller dw_controller = {
	.pci_ops	= &dw_pci_ops,
	.io_resource	= &dw_io_resource,
	.mem_resource	= &dw_mem_resource,
	.busn_resource	= &dw_busn_resource,
	.get_busno  = dw_pcie_get_busn,
};

#ifdef CONFIG_PCI_MSI
static int dw_msi_irq;
#endif /* CONFIG_PCI_MSI */

static int dw_aer_irq;

#define PCIE_PHY_RETRIES	1000000
#define PCIE_ERROR_VALUE	0xFFFFFFFF
#define PHY_ALL_LANES		0xF
#define PHY_LANE0			0x1

#define OK					0
#define ERROR				-1
#define ERROR_MISMATCH1		0x0010
#define ERROR_MISMATCH2		0x0020
#define ERROR_MISMATCH3		0x0040
#define ERROR_MISMATCH4		0x0080
#define ERROR_MISMATCH5		0x0100
#define ERROR_MISMATCH6		0x0200
#define ERROR_MISMATCH7		0x0400
#define ERROR_MISMATCH8		0x0800

/* Retrieve the secondary bus number of the RC */
static int dw_pcie_get_busn(void)
{
	return PCIE_ROOT_BUS_NUM;
}

void baikal_find_vga_mem_init(void);

static uint32_t dw_pcie_phy_read(uint32_t phy_addr)
{
	uint32_t reg;
	int i;

	/* Set lane0 for reading values. */
	WRITE_PCIE_REG(PCIE_BK_MGMT_SEL_LANE, PHY_LANE0);

	/* Write the address of the PHY register. */
	WRITE_PCIE_REG(PCIE_BK_MGMT_CTRL, (phy_addr & BK_MGMT_CTRL_ADDR_MASK)
			| BK_MGMT_CTRL_READ);

	for (i = 0; i < PCIE_PHY_RETRIES; i++) {
		reg = READ_PCIE_REG(PCIE_BK_MGMT_CTRL);
		if (reg & BK_MGMT_CTRL_DONE) {
			/* Read data register. */
			reg = READ_PCIE_REG(PCIE_BK_MGMT_READ_DATA);
			pr_debug("%s: phy_addr=0x%x val=0x%x\n", __func__,
				 phy_addr, reg);
			return reg;
		}
	}

	pr_err("%s: timeout expired for phy_addr=0x%x\n", __func__, phy_addr);

	/* return error */
	return PCIE_ERROR_VALUE;
}

static uint32_t dw_pcie_phy_write(uint32_t phy_addr, uint32_t val)
{
	uint32_t reg;
	int i;

	pr_debug("%s: phy_addr=0x%x val=0x%x\n", __func__, phy_addr, val);

	/* Set line. */
	WRITE_PCIE_REG(PCIE_BK_MGMT_SEL_LANE, PHY_ALL_LANES);

	/* Write value to data register. */
	WRITE_PCIE_REG(PCIE_BK_MGMT_WRITE_DATA, val);

	/* Write the address of the PHY register. */
	WRITE_PCIE_REG(PCIE_BK_MGMT_CTRL, (phy_addr & BK_MGMT_CTRL_ADDR_MASK)
			| BK_MGMT_CTRL_WRITE);

	for (i = 0; i < PCIE_PHY_RETRIES; i++) {
		reg = READ_PCIE_REG(PCIE_BK_MGMT_CTRL);
		if (reg & BK_MGMT_CTRL_DONE)
			return OK;
	}

	pr_err("%s: timeout expired for phy_addr=0x%x\n", __func__, phy_addr);

	/* return error */
	return PCIE_ERROR_VALUE;
}

void dw_set_iatu_region(int dir, int index, int base_addr, int limit_addr,
			int target_addr, int tlp_type)
{
	WRITE_PCIE_REG(PCIE_IATU_VIEWPORT_OFF, ((index << REGION_INDEX_SHIFT)
			| (dir << REGION_DIR_SHIFT)));
	WRITE_PCIE_REG(PCIE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0,
			(base_addr << LWR_BASE_RW_SHIFT));
	WRITE_PCIE_REG(PCIE_IATU_UPR_BASE_ADDR_OFF_OUTBOUND_0, 0);
	WRITE_PCIE_REG(PCIE_IATU_LIMIT_ADDR_OFF_OUTBOUND_0,
			(limit_addr << LIMIT_ADDR_RW_SHIFT));
	WRITE_PCIE_REG(PCIE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0,
			(target_addr << LWR_TARGET_RW_SHIFT));
	WRITE_PCIE_REG(PCIE_IATU_UPR_TARGET_ADDR_OFF_OUTBOUND_0, 0);
	WRITE_PCIE_REG(PCIE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0,
		(tlp_type << IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_SHIFT));
	WRITE_PCIE_REG(PCIE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0,
			IATU_REGION_CTRL_2_OFF_OUTBOUND_0_REGION_EN);

	wmb();
}

#ifdef CONFIG_MACH_BAIKAL_BFK
#define PLL_WAIT_RETRIES 1000
static int dw_init_pll(const unsigned int pmu_register)
{
	uint32_t reg;
	int i = 0;

	/* Wait for LOCK bit in BK_PMU_COREPLL_CTL */
	while (!(READ_PMU_REG(BK_PMU_COREPLL_CTL) & BK_PMU_LOCK_BIT)) {
		if ((i++) == PLL_WAIT_RETRIES) {
			return ERROR;
		}
	}
	/* Set EN & RST bit in pmu_register */
	reg = READ_PMU_REG(pmu_register);
	reg |= BK_PMU_EN_BIT | BK_PMU_RST_BIT;
	WRITE_PMU_REG(pmu_register, reg);

	/* Wait for LOCK bit in pmu_register */
	i = 0;
	while (!(READ_PMU_REG(pmu_register) & BK_PMU_LOCK_BIT)) {
		if ((i++) == PLL_WAIT_RETRIES) {
			return ERROR;
		}
	}

	return OK;
}
#endif /* CONFIG_MACH_BAIKAL_BFK */

static int dw_pcie_init(void)
{
	volatile uint32_t reg;
	int i, st = 0;
	uint32_t pmu_pcie_rstc_mask = 0;

	/* PMU PCIe init. */

#ifdef CONFIG_MACH_BAIKAL_BFK
	/* Init PCIe PLL only for Baikal-T CPU  */
	/* 2. Start BK_PMU_PCIEPLL_CTL. */
	dw_init_pll(BK_PMU_PCIEPLL_CTL);
#endif /* CONFIG_MACH_BAIKAL_BFK */

	/* 3. Read value of BK_PMU_AXI_PCIE_M_CTL, set EN bit. */
	reg = READ_PMU_REG(BK_PMU_AXI_PCIE_M_CTL);
	reg |= PMU_AXI_PCIE_M_CTL_EN;
	WRITE_PMU_REG(BK_PMU_AXI_PCIE_M_CTL, reg);

	/* 4. Read value of BK_PMU_AXI_PCIE_S_CTL, set EN bit. */
	reg = READ_PMU_REG(BK_PMU_AXI_PCIE_S_CTL);
	reg |= PMU_AXI_PCIE_S_CTL_EN;
	WRITE_PMU_REG(BK_PMU_AXI_PCIE_S_CTL, reg);

	/*
	 * 5. Manage RESET* bits
	 * (PHY_RESET, PIPE_RESET, CORE_RST, PWR_RST, STICKY_RST, NONSTICKY_RST)
	 */

	reg = READ_PMU_REG(BK_PMU_PCIE_RSTC);
	if (reg & PMU_PCIE_RSTC_REQ_PHY_RST)
		pmu_pcie_rstc_mask |= PMU_PCIE_RSTC_PHY_RESET;
	if (reg & PMU_PCIE_RSTC_REQ_CORE_RST)
		pmu_pcie_rstc_mask |= PMU_PCIE_RSTC_CORE_RST;
	if (reg & PMU_PCIE_RSTC_REQ_STICKY_RST)
		pmu_pcie_rstc_mask |= PMU_PCIE_RSTC_STICKY_RST;
	if (reg & PMU_PCIE_RSTC_REQ_NON_STICKY_RST)
		pmu_pcie_rstc_mask |= PMU_PCIE_RSTC_NONSTICKY_RST;
	WRITE_PMU_REG(BK_PMU_PCIE_RSTC, reg | pmu_pcie_rstc_mask);
	usleep_range(10, 20);
	reg = READ_PMU_REG(BK_PMU_PCIE_RSTC);
	reg &= ~pmu_pcie_rstc_mask;
	WRITE_PMU_REG(BK_PMU_PCIE_RSTC, reg);
	reg = READ_PMU_REG(BK_PMU_PCIE_RSTC);
	pr_debug("%s: PCIE_RSTC after reset: %08x\n", __func__, reg);

	/* 3.1 Set DBI2 mode, dbi2_cs = 0x1 */
	reg = READ_PMU_REG(BK_PMU_PCIE_GENC);
	reg |= PMU_PCIE_GENC_DBI2_MODE;
	WRITE_PMU_REG(BK_PMU_PCIE_GENC, reg);

	/* 3.2 Set writing to RO Registers Using DBI */
	WRITE_PCIE_REG(PCIE_MISC_CONTROL_1_OFF, DBI_RO_WR_EN);

	/* 4.1 Allow access to the PHY registers,
	 * phy0_mgmt_pcs_reg_sel = 0x1.
	 */
	reg = READ_PMU_REG(BK_PMU_PCIE_GENC);
	reg |= PMU_PCIE_GENC_MGMT_ENABLE;
	WRITE_PMU_REG(BK_PMU_PCIE_GENC, reg);

	/* 4.2 All lanes can read/write PHY registers using
	 * the management interface.
	 */
	WRITE_PCIE_REG(PCIE_BK_MGMT_SEL_LANE, 0xF);

	/*
	 * 7. Wait for stable clocks: SDS_PCS_CLOCK_READY bit in
	 * DWC_GLBL_PLL_MONITOR register of PCIe PHY.
	 */
	for (i = 0; i < PCIE_PHY_RETRIES; i++) {
		reg = dw_pcie_phy_read(PCIE_PHY_DWC_GLBL_PLL_MONITOR);

		if (reg == PCIE_ERROR_VALUE) {
			return ERROR;
		}
		if ((reg & SDS_PCS_CLOCK_READY) == SDS_PCS_CLOCK_READY) {
			break;
		}
	}

	if (i == PCIE_PHY_RETRIES) {
		/* Return an error if the timeout expired. */
		return ERROR;
	}

	/*
	 * 8. Read value of BK_PMU_PCIE_RSTC, reset bits: PIPE_RESET, CORE_RST,
	 * PWR_RST, STICKY_RST, NONSTICKY_RST, HOT_RESET.
	 */
	reg = READ_PMU_REG(BK_PMU_PCIE_RSTC);
	reg &= ~(PMU_PCIE_RSTC_PIPE_RESET | PMU_PCIE_RSTC_CORE_RST |
		 PMU_PCIE_RSTC_PWR_RST | PMU_PCIE_RSTC_STICKY_RST |
		 PMU_PCIE_RSTC_NONSTICKY_RST | PMU_PCIE_RSTC_HOT_RESET);
	WRITE_PMU_REG(BK_PMU_PCIE_RSTC, reg);

	pr_info("%s: DEV_ID_VEND_ID=0x%x CLASS_CODE_REV_ID=0x%x\n", __func__,
		READ_PCIE_REG(PCIE_TYPE1_DEV_ID_VEND_ID_REG),
		READ_PCIE_REG(PCIE_TYPE1_CLASS_CODE_REV_ID_REG));

	/* 5. Set the fast mode. */
#if 0
	reg = READ_PCIE_REG(PCIE_PORT_LINK_CTRL_OFF);
	reg |= FAST_LINK_MODE;
	WRITE_PCIE_REG(PCIE_PORT_LINK_CTRL_OFF, reg);

	reg = dw_pcie_phy_read(PCIE_PHY_DWC_GLBL_PLL_CFG_0);
	reg &= ~PCS_SDS_PLL_FTHRESH_MASK;
	dw_pcie_phy_write(PCIE_PHY_DWC_GLBL_PLL_CFG_0, reg);

	reg = dw_pcie_phy_read(PCIE_PHY_DWC_GLBL_TERM_CFG);
	reg |= FAST_TERM_CAL;
	dw_pcie_phy_write(PCIE_PHY_DWC_GLBL_TERM_CFG, reg);

	reg = dw_pcie_phy_read(PCIE_PHY_DWC_RX_LOOP_CTRL);
	reg |= (FAST_OFST_CNCL | FAST_DLL_LOCK);
	dw_pcie_phy_write(PCIE_PHY_DWC_RX_LOOP_CTRL, reg);

	reg = dw_pcie_phy_read(PCIE_PHY_DWC_TX_CFG_0);
	reg |= (FAST_TRISTATE_MODE | FAST_RDET_MODE | FAST_CM_MODE);
	dw_pcie_phy_write(PCIE_PHY_DWC_TX_CFG_0, reg);
#endif

	/* 6. Set number of lanes. */
	reg = READ_PCIE_REG(PCIE_GEN2_CTRL_OFF);
	reg &= ~NUM_OF_LANES_MASK;
	reg |= (0x4 << NUM_OF_LANES_SHIFT);
	//reg |= DIRECT_SPEED_CHANGE; // also force Directed Speed Change
	WRITE_PCIE_REG(PCIE_GEN2_CTRL_OFF, reg);

	reg = READ_PCIE_REG(PCIE_PORT_LINK_CTRL_OFF);
	reg &= ~LINK_CAPABLE_MASK;
	reg |= (0x7 << LINK_CAPABLE_SHIFT);
	WRITE_PCIE_REG(PCIE_PORT_LINK_CTRL_OFF, reg);

	/* 7. Enable GEN3 */
#if 0
	reg = READ_PCIE_REG(PCIE_GEN3_EQ_CONTROL_OFF);
	reg &= ~(GEN3_EQ_FB_MODE_MASK | GEN3_EQ_PSET_REQ_VEC_MASK);
	reg |= ((GEN3_EQ_EVAL_2MS_DISABLE) | (0x1 << GEN3_EQ_FB_MODE_SHIFT) |
		(0x1 << GEN3_EQ_PSET_REQ_VEC_SHIFT));
	WRITE_PCIE_REG(PCIE_GEN3_EQ_CONTROL_OFF, reg);

	WRITE_PCIE_REG(PCIE_LANE_EQUALIZATION_CONTROL01_REG, 0);
	WRITE_PCIE_REG(PCIE_LANE_EQUALIZATION_CONTROL23_REG, 0);

	dw_pcie_phy_write(PCIE_PHY_DWC_RX_PRECORR_CTRL, 0);
	dw_pcie_phy_write(PCIE_PHY_DWC_RX_CTLE_CTRL, 0x200);
	dw_pcie_phy_write(PCIE_PHY_DWC_RX_VMA_CTRL, 0xc000);
	dw_pcie_phy_write(PCIE_PHY_DWC_PCS_LANE_VMA_FINE_CTRL_0, 0);
	dw_pcie_phy_write(PCIE_PHY_DWC_PCS_LANE_VMA_FINE_CTRL_1, 0);
	dw_pcie_phy_write(PCIE_PHY_DWC_PCS_LANE_VMA_FINE_CTRL_2, 0);
	dw_pcie_phy_write(PCIE_PHY_DWC_EQ_WAIT_TIME, 0xa);
#endif

	/* 7.1 Disable entire DFE */
	reg = dw_pcie_phy_read(PCIE_PHY_DWC_RX_LOOP_CTRL);
	reg |= 0x2;
	dw_pcie_phy_write(PCIE_PHY_DWC_RX_LOOP_CTRL, reg);

	//reg = dw_pcie_phy_read(PCIE_PHY_DWC_RX_AEQ_VALBBD_2);
	reg = 0x3F;
	dw_pcie_phy_write(PCIE_PHY_DWC_RX_AEQ_VALBBD_2, reg);

	//reg = dw_pcie_phy_read(PCIE_PHY_DWC_RX_AEQ_VALBBD_1);
	reg = 0;
	dw_pcie_phy_write(PCIE_PHY_DWC_RX_AEQ_VALBBD_1, reg);

	/* Configure bus. */
	reg = READ_PCIE_REG(PCIE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG);
	reg &= 0xff000000;
	/* IDT PCI Bridge don't like the primary bus equals 0. */
	reg |= (0x00ff0000 | (PCIE_ROOT_BUS_NUM << 8));
	WRITE_PCIE_REG(PCIE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG, reg);

	/* Setup memory base. */
	reg = ((PHYS_PCIMEM_LIMIT_ADDR & 0xfff00000) |
		((PHYS_PCIMEM_BASE_ADDR & 0xfff00000) >> 16));
	WRITE_PCIE_REG(PCIE_MEM_LIMIT_MEM_BASE_REG, reg);

	/* Setup IO base. */
	reg = ((PHYS_PCIIO_LIMIT_ADDR & 0x0000f000) |
		((PHYS_PCIIO_BASE_ADDR & 0x0000f000) >> 8));
	WRITE_PCIE_REG(PCIE_SEC_STAT_IO_LIMIT_IO_BASE_REG, reg);
	reg = ((PHYS_PCIIO_LIMIT_ADDR & 0xffff0000) |
		((PHYS_PCIIO_BASE_ADDR & 0xffff0000) >> 16));
	WRITE_PCIE_REG(PCIE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG, reg);

	/* 8. Set master for PCIe EP. */
	reg = READ_PCIE_REG(PCIE_TYPE1_STATUS_COMMAND_REG);
	reg |= (TYPE1_STATUS_COMMAND_REG_BME | TYPE1_STATUS_COMMAND_REG_MSE |
		TYPE1_STATUS_COMMAND_REG_IOSE);
	reg |= (PCI_COMMAND_PARITY | PCI_COMMAND_SERR); // Add check error.
	WRITE_PCIE_REG(PCIE_TYPE1_STATUS_COMMAND_REG, reg);

	/* AER */
	reg =  READ_PCIE_REG(PCIE_DEVICE_CONTROL_DEVICE_STATUS);
	reg |= PCI_EXP_DEVCTL_CERE; /* Correctable Error Reporting */
	reg |= PCI_EXP_DEVCTL_NFERE; /* Non-Fatal Error Reporting */
	reg |= PCI_EXP_DEVCTL_FERE;	/* Fatal Error Reporting */
	reg |= PCI_EXP_DEVCTL_URRE;	/* Unsupported Request */
	WRITE_PCIE_REG(PCIE_DEVICE_CONTROL_DEVICE_STATUS, reg);

	/* Unmask Uncorrectable Errors. */
	reg = READ_PCIE_REG(PCIE_UNCORR_ERR_STATUS_OFF);
	WRITE_PCIE_REG(PCIE_UNCORR_ERR_STATUS_OFF, reg);
	WRITE_PCIE_REG(PCIE_UNCORR_ERR_MASK_OFF, 0);

	/* Unmask Correctable Errors. */
	reg = READ_PCIE_REG(PCIE_CORR_ERR_STATUS_OFF);
	WRITE_PCIE_REG(PCIE_CORR_ERR_STATUS_OFF, reg);
	WRITE_PCIE_REG(PCIE_CORR_ERR_MASK_OFF, 0);

#ifdef DW_CHECK_ECRC
	reg = READ_PCIE_REG(PCIE_ADV_ERR_CAP_CTRL_OFF);
	/* ECRC Generation Enable */
	if (reg & PCI_ERR_CAP_ECRC_GENC)
		reg |= PCI_ERR_CAP_ECRC_GENE;
	/* ECRC Check Enable */
	if (reg & PCI_ERR_CAP_ECRC_CHKC)
		reg |= PCI_ERR_CAP_ECRC_CHKE;
	WRITE_PCIE_REG(PCIE_ADV_ERR_CAP_CTRL_OFF, reg);
#endif /* DW_CHECK_ECRC */

	/* 9. Set Inbound/Outbound iATU regions. */

	/* dw_set_iatu_region(dir,  index, base_addr, limit_addr,
	 * target_addr, tlp_type)
	 */

	dw_set_iatu_region(REGION_DIR_OUTBOUND, IATU_RD0_INDEX,
				PHYS_PCI_RD0_BASE_ADDR >> 16,
				PHYS_PCI_RD0_LIMIT_ADDR >> 16, 0x0000,
				TLP_TYPE_CFGRD0);

	dw_set_iatu_region(REGION_DIR_OUTBOUND, IATU_RD1_INDEX,
				PHYS_PCI_RD1_BASE_ADDR >> 16,
				PHYS_PCI_RD1_LIMIT_ADDR >> 16, 0x0000,
				TLP_TYPE_CFGRD1);

	dw_set_iatu_region(REGION_DIR_OUTBOUND, IATU_MEM_INDEX,
				PHYS_PCIMEM_BASE_ADDR >> 16,
				PHYS_PCIMEM_LIMIT_ADDR >> 16,
				PHYS_PCIMEM_BASE_ADDR >> 16, TLP_TYPE_MEM);

	dw_set_iatu_region(REGION_DIR_OUTBOUND, IATU_IO_INDEX,
				PHYS_PCIIO_BASE_ADDR >> 16,
				PHYS_PCIIO_LIMIT_ADDR >> 16,
				PHYS_PCIIO_BASE_ADDR >> 16,  TLP_TYPE_IO);

	/*
	 * Set GEN1 speed. In case EP supports GEN2
	 * a link will be retrained later (see fixup-baikal.c).
	 * At that moment it's impossible to
	 * configure a link on GEN3 speed.
	 */
	reg = READ_PCIE_REG(PCIE_LINK_CONTROL2_LINK_STATUS2_REG);
	reg &= ~PCIE_LINK_CONTROL2_GEN_MASK;
	reg |= PCIE_LINK_CONTROL2_GEN1;
	WRITE_PCIE_REG(PCIE_LINK_CONTROL2_LINK_STATUS2_REG, reg);

	wmb();

	/* 10. Set LTSSM enable, app_ltssm_enable=0x1 */
	reg = READ_PMU_REG(BK_PMU_PCIE_GENC);
	reg |= PMU_PCIE_GENC_LTSSM_ENABLE;
	WRITE_PMU_REG(BK_PMU_PCIE_GENC, reg);

	/* 11-12 Analyze BK_PMU_PCIE_PMSC */
	for (i = 0; i < PCIE_PHY_RETRIES; i++) {
		reg = READ_PMU_REG(BK_PMU_PCIE_PMSC);
		st = 0;
		if ((reg & PMU_PCIE_PMSC_SMLH_LINKUP) == 0) {
			st |= ERROR_MISMATCH3;
		}

		if ((reg & PMU_PCIE_PMSC_RDLH_LINKUP) == 0) {
			st |= ERROR_MISMATCH4;
		}

		if ((reg & PMU_PCIE_PMSC_LTSSM_STATE_MASK) != LTSSM_L0) {
			st |= ERROR_MISMATCH5;
		}

		if (!st) {
			break;
		}
	}

	pr_err("%s: PCIe error code = 0x%x\n", __func__, st);

	/* Check the speed is set in PCIE_LINK_CONTROL_LINK_STATUS_REG. */
	reg = READ_PCIE_REG(PCIE_LINK_CONTROL_LINK_STATUS_REG);
	reg = ((reg & PCIE_CAP_LINK_SPEED_MASK) >> PCIE_CAP_LINK_SPEED_SHIFT);
	pr_info("%s: PCIe link speed GEN%d\n", __func__, reg);

	wmb();

	return st;
}

void __init mips_pcibios_init(void)
{
	struct pci_controller *controller;

	if (dw_pcie_init()) {
		pr_err("%s: Init DW PCI controller failed\n", __func__);
		return;
	}

#ifdef CONFIG_PCI_MSI
	if (dw_msi_init()) {
		pr_err("%s: Init DW PCI MSI failed\n", __func__);
		return;
	}
#endif /* CONFIG_PCI_MSI */

	pci_set_flags(PCI_REASSIGN_ALL_RSRC);

	/* Register PCI controller */
	controller = &dw_controller;

	iomem_resource.end &= 0xfffffffffULL;
	ioport_resource.end = controller->io_resource->end;
	controller->io_map_base = IO_BASE;
	controller->io_offset = 0;
	register_pci_controller(controller);

#ifdef CONFIG_MIPS_BAIKAL_PCI_DMA
	pci_dw_dma_init();
#endif

#ifdef CONFIG_CPU_SUPPORTS_UNCACHED_ACCELERATED
	baikal_find_vga_mem_init();
#endif /* CONFIG_CPU_SUPPORTS_UNCACHED_ACCELERATED */
}

#ifdef CONFIG_PCIEAER
irqreturn_t aer_irq(int irq, void *context);
#endif /* CONFIG_PCIEAER */

irqreturn_t dw_aer_interrupt(int id, void *dev_id)
{
#ifdef CONFIG_PCIEAER
	aer_irq(id, dev_id);
#endif /* CONFIG_PCIEAER */

	return IRQ_HANDLED;
}

static int dw_pci_drv_probe(struct platform_device *pdev)
{

#ifdef CONFIG_PCI_MSI
	if ((dw_msi_irq = platform_get_irq(pdev, 0)) < 0) {
		dev_err(&pdev->dev, "There is no MSI IRQ resource specified.\n");
		return -EINVAL;
	}

	if (request_irq(dw_msi_irq, dw_msi_interrupt, IRQF_SHARED,
			"MSI PCI", pdev)) {
		dev_err(&pdev->dev, "Cannot request MSI irq %d.\n",
			dw_msi_irq);
		return -ENXIO;
	}
#endif /* CONFIG_PCI_MSI */

	if ((dw_aer_irq = platform_get_irq(pdev, 1)) < 0) {
		dev_err(&pdev->dev, "There is no AER IRQ resource specified.\n");
		return -EINVAL;
	}

	if (request_irq(dw_aer_irq, dw_aer_interrupt, IRQF_SHARED,
			"AER PCI", pdev)) {
		dev_err(&pdev->dev, "Cannot request AER irq %d.\n",
			dw_aer_irq);
		return -ENXIO;
	}

	dev_info(&pdev->dev, "DW PCIe driver successfully loaded.\n");
	dev_info(&pdev->dev, "MSI IRQ:%d   AER IRQ:%d\n",
		 dw_msi_irq, dw_aer_irq);

	/* Return success */
	return 0;
}

static int dw_pci_drv_remove(struct platform_device *pdev)
{
#ifdef CONFIG_PCI_MSI
	/* Free IRQ resource */
	free_irq(dw_msi_irq, pdev);
#endif /* CONFIG_PCI_MSI */

	free_irq(dw_aer_irq, pdev);

	/* Return success */
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dw_pci_of_match[] = {
	{ .compatible = "be,baikal-pci", },
	{ .compatible = "snps,dw-pci", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dw_pci_of_match);
#endif

static struct platform_driver dw_pci_driver = {
	.probe          = dw_pci_drv_probe,
	.remove         = dw_pci_drv_remove,
	.driver         = {
		.name   = "dw_pci",
		.owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(dw_pci_of_match),
#endif /* CONFIG_OF */
		},
};

module_platform_driver(dw_pci_driver);
MODULE_VERSION("1.2");
MODULE_DESCRIPTION("Baikal Electronics PCIe Driver.");
MODULE_LICENSE("Proprietary");
MODULE_AUTHOR("Alexey Malakhov");
MODULE_ALIAS("platform:dw_pci");

