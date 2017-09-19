/*
 *  Baikal-T SOC platform support code.
 *
 *  Copyright (C) 2015 Baikal Electronics.
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

#ifndef __PCI_BAIKAL_H__
#define __PCI_BAIKAL_H__

#include <linux/interrupt.h>

/* Define DW_CHECK_ECRC to add checking CRC. */
//#define DW_CHECK_ECRC

#define PCIE_CFG_BASE					0xBF052000
#define PMU_BASE						0xBF04D000
/* Start enumerating the busses from 1 since IDT-switch oddly acts, when it's
 * directly connected to the RC and has bus number 0 */
#define PCIE_ROOT_BUS_NUM      1

#define	PHYS_PCIMEM_BASE_ADDR		(0x08000000)
#define	PHYS_PCIMEM_LIMIT_ADDR		(0x18000000 - 1)
#define IATU_MEM_INDEX			2

#define	PHYS_PCI_RD0_BASE_ADDR		(0x18000000)
#define	PHYS_PCI_RD0_LIMIT_ADDR		(0x18010000 - 1)
#define PCI_RD0_BASE_ADDR		KSEG1ADDR(PHYS_PCI_RD0_BASE_ADDR)
#define IATU_RD0_INDEX			0

#define	PHYS_PCI_RD1_BASE_ADDR		(0x18010000)
#define	PHYS_PCI_RD1_LIMIT_ADDR		(0x18020000 - 1)
#define PCI_RD1_BASE_ADDR		KSEG1ADDR(PHYS_PCI_RD1_BASE_ADDR)
#define IATU_RD1_INDEX			1

#define	PHYS_PCIIO_BASE_ADDR		0x18020000
#define	PHYS_PCIIO_LIMIT_ADDR		(0x1BDB0000 - 1)
#define IATU_IO_INDEX			3

#define	PHYS_PCI_MSI_BASE_ADDR		(0x1BDB0000)
#define	PHYS_PCI_START_ADDR		(0x08000000)
#define	PHYS_PCI_END_ADDR		(0x1BDC0000)

#define PCIE_TYPE1_DEV_ID_VEND_ID_REG		(0x0)	/* Device ID and Vendor ID Register. */
#define PCIE_TYPE1_STATUS_COMMAND_REG		(0x4)	/* Command and Status Register. */
#define PCIE_TYPE1_CLASS_CODE_REV_ID_REG	(0x8)	/* Class Code and Revision ID Register. */
#define PCIE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG	(0xc)	/* BIST, Header Type, Cache Line Size, and Master Latency Timer Register. */
#define PCIE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG		(0x18)	/* Primary, Secondary, Subordinate Bus Numbers and Latency Timer Regisers . */
#define PCIE_SEC_STAT_IO_LIMIT_IO_BASE_REG	(0x1c)	/* Secondary Status and I/O Base and Limit Registers. */
#define PCIE_MEM_LIMIT_MEM_BASE_REG		(0x20)	/* Memory Base and Memory Limit Register. */
#define PCIE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG	(0x24)	/* Prefetchable Memory Base and Limit Register. */
#define PCIE_PREF_BASE_UPPER_REG		(0x28)	/* Prefetchable Base Upper 32 Bits Register. */
#define PCIE_PREF_LIMIT_UPPER_REG		(0x2c)	/* I/O Base and Limit Upper 16 Bits Register. */
#define PCIE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG	(0x30)	/* Expansion ROM BAR and Mask Register. */
#define PCIE_TYPE1_CAP_PTR_REG			(0x34)	/* Capability Pointer Register. */
#define PCIE_TYPE1_EXP_ROM_BASE_REG		(0x38)	/* Expansion ROM BAR and Mask Register. */
#define PCIE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG	(0x3c)	/* Interrupt Line and Pin and Bridge Control Registers. */
#define PCIE_CAP_ID_NXT_PTR_REG			(0x40)	/* Power Management Capabilities Register. */
#define PCIE_CON_STATUS_REG			(0x44)	/* Power Management Control and Status Register. */
#define PCIE_PCI_MSI_CAP_ID_NEXT_CTRL_REG	(0x50)	/* MSI Capability ID, Next Pointer, Control Registers. */
#define PCIE_MSI_CAP_OFF_04H_REG		(0x54)	/* MSI Capability ID, Next Pointer, Control Registers. */
#define PCIE_MSI_CAP_OFF_08H_REG		(0x58)	/* MSI Capability ID, Next Pointer, Control Registers. */
#define PCIE_MSI_CAP_OFF_0CH_REG		(0x5c)	/* MSI Capability ID, Next Pointer, Control Registers. */
#define PCIE_MSI_CAP_OFF_10H_REG		(0x60)	/* MSI Capability ID, Next Pointer, Control Registers. */
#define PCIE_MSI_CAP_OFF_14H_REG		(0x64)	/* MSI Capability ID, Next Pointer, Control Registers. */
#define PCIE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG	(0x70)	/* PCI Express Capabilities, ID, Next Pointer Register. */
#define PCIE_DEVICE_CAPABILITIES_REG		(0x74)	/* Device Capabilities Register. */
#define PCIE_DEVICE_CONTROL_DEVICE_STATUS	(0x78)	/* Device Control and Status Register. */
#define PCIE_LINK_CAPABILITIES_REG		(0x7c)	/* Link Capabilities Register. */
#define PCIE_LINK_CONTROL_LINK_STATUS_REG	(0x80)	/* Link Control and Status Register. */
#define PCIE_ROOT_CONTROL_ROOT_CAPABILITIES_REG	(0x8c)	/* Root Control and Capabilities Register. */
#define PCIE_ROOT_STATUS_REG			(0x90)	/* Root Status Register. */
#define PCIE_DEVICE_CAPABILITIES2_REG		(0x94)	/* Device Capabilities 2 Register. */
#define PCIE_DEVICE_CONTROL2_DEVICE_STATUS2_REG	(0x98)	/* Device Control 2 and Status 2 Register. */
#define PCIE_LINK_CAPABILITIES2_REG		(0x9c)	/* Link Capabilities 2 Register. */
#define PCIE_LINK_CONTROL2_LINK_STATUS2_REG	(0xa0)	/* Link Control 2 and Status 2 Register. */
#define PCIE_PCI_MSIX_CAP_ID_NEXT_CTRL_REG	(0xb0)	/* MSI-X Capability ID, Next Pointer, Control Registers. */
#define PCIE_MSIX_TABLE_OFFSET_REG		(0xb4)	/* MSI-X Table Offset and BIR Register. */
#define PCIE_MSIX_PBA_OFFSET_REG		(0xb8)	/* MSI-X PBA Offset and BIR Register. */
#define PCIE_SLOTNUM_BASE			(0xc0)	/* Slot Numbering Capabilities Register. */
#define PCIE_VPD_BASE				(0xd0)	/* VPD Control and Capabilities Register. */
#define PCIE_DATA_REG				(0xd4)	/* VPD Data Register. */
#define PCIE_AER_EXT_CAP_HDR_OFF		(0x100)	/* Advanced Error Reporting Extended Capability Header. */
#define PCIE_UNCORR_ERR_STATUS_OFF		(0x104)	/* Uncorrectable Error Status Register. */
#define PCIE_UNCORR_ERR_MASK_OFF		(0x108)	/* Uncorrectable Error Mask Register. */
#define PCIE_UNCORR_ERR_SEV_OFF			(0x10c)	/* Uncorrectable Error Severity Register. */
#define PCIE_CORR_ERR_STATUS_OFF		(0x110)	/* Correctable Error Status Register. */
#define PCIE_CORR_ERR_MASK_OFF			(0x114)	/* Correctable Error Mask Register. */
#define PCIE_ADV_ERR_CAP_CTRL_OFF		(0x118)	/* Advanced Error Capabilities and Control Register. */
#define PCIE_HDR_LOG_0_OFF			(0x11c)	/* Header Log Register 0. */
#define PCIE_HDR_LOG_1_OFF			(0x120)	/* Header Log Register 1. */
#define PCIE_HDR_LOG_2_OFF			(0x124)	/* Header Log Register 2. */
#define PCIE_HDR_LOG_3_OFF			(0x128)	/* Header Log Register 3. */
#define PCIE_ROOT_ERR_CMD_OFF			(0x12c)	/* Root Error Command Register. */
#define PCIE_ROOT_ERR_STATUS_OFF		(0x130)	/* Root Error Status Register. */
#define PCIE_ERR_SRC_ID_OFF			(0x134)	/* Error Source Identification Register. */
#define PCIE_TLP_PREFIX_LOG_OFF			(0x138)	/* TLP Prefix Log Register. */
#define PCIE_VC_BASE				(0x148)	/* VC Extended Capability Header. */
#define PCIE_VC_CAPABILITIES_REG_1		(0x14c)	/* Port VC Capability Register 1. */
#define PCIE_VC_CAPABILITIES_REG_2		(0x150)	/* Port VC Capability Register 2. */
#define PCIE_VC_STATUS_CONTROL_REG		(0x154)	/* Port VC Control and Status Register. */
#define PCIE_RESOURCE_CAP_REG_VC0		(0x158)	/* VC Resource Capability Register (0). */
#define PCIE_RESOURCE_CON_REG_VC0		(0x15c)	/* VC Resource Control Register (0). */
#define PCIE_RESOURCE_STATUS_REG_VC0		(0x160)	/* VC Resource Status Register (0). */
#define PCIE_RESOURCE_CAP_REG_VC1		(0x164)	/* VC Resource Capability Register (1). */
#define PCIE_RESOURCE_CON_REG_VC1		(0x168)	/* VC Resource Control Register (1). */
#define PCIE_RESOURCE_STATUS_REG_VC1		(0x16c)	/* For a description of this standard PCIe register field, see the PCI Express. */
#define PCIE_RESOURCE_CAP_REG_VC2		(0x170)	/* VC Resource Capability Register (2). */
#define PCIE_RESOURCE_CON_REG_VC2		(0x174)	/* VC Resource Control Register (2). */
#define PCIE_RESOURCE_STATUS_REG_VC2		(0x178)	/* For a description of this standard PCIe register field, see the PCI Express. */
#define PCIE_RESOURCE_CAP_REG_VC3		(0x17c)	/* VC Resource Capability Register (3). */
#define PCIE_RESOURCE_CON_REG_VC3		(0x180)	/* VC Resource Control Register (3). */
#define PCIE_RESOURCE_STATUS_REG_VC3		(0x184)	/* For a description of this standard PCIe register field, see the PCI Express. */
#define PCIE_RESOURCE_CAP_REG_VC4		(0x188)	/* VC Resource Capability Register (4). */
#define PCIE_RESOURCE_CON_REG_VC4		(0x18c)	/* VC Resource Control Register (4). */
#define PCIE_RESOURCE_STATUS_REG_VC4		(0x190)	/* For a description of this standard PCIe register field, see the PCI Express. */
#define PCIE_RESOURCE_CAP_REG_VC5		(0x194)	/* VC Resource Capability Register (5). */
#define PCIE_RESOURCE_CON_REG_VC5		(0x198)	/* VC Resource Control Register (5). */
#define PCIE_RESOURCE_STATUS_REG_VC5		(0x19c)	/* For a description of this standard PCIe register field, see the PCI Express. */
#define PCIE_RESOURCE_CAP_REG_VC6		(0x1a0)	/* VC Resource Capability Register (6). */
#define PCIE_RESOURCE_CON_REG_VC6		(0x1a4)	/* VC Resource Control Register (6). */
#define PCIE_RESOURCE_STATUS_REG_VC6		(0x1a8)	/* For a description of this standard PCIe register field, see the PCI Express. */
#define PCIE_RESOURCE_CAP_REG_VC7		(0x1ac)	/* VC Resource Capability Register (7). */
#define PCIE_RESOURCE_CON_REG_VC7		(0x1b0)	/* VC Resource Control Register (7). */
#define PCIE_RESOURCE_STATUS_REG_VC7		(0x1b4)	/* For a description of this standard PCIe register field, see the PCI Express. */
#define PCIE_SN_BASE				(0x168)	/* Device Serial Number Extended Capability Header. */
#define PCIE_SER_NUM_REG_DW_1			(0x16c)	/* Serial Number 1 Register. */
#define PCIE_SER_NUM_REG_DW_2			(0x170)	/* Serial Number 2 Register. */
#define PCIE_PB_BASE				(0x178)	/* Power Budgeting Extended Capability Header. */
#define PCIE_DATA_REG_PB			(0x180)	/* Data Register. */
#define PCIE_CAP_REG_PB				(0x184)	/* Power Budget Capability Register. */
#define PCIE_SPCIE_CAP_HEADER_REG		(0x198)	/* SPCIE Capability Header. */
#define PCIE_LINK_CONTROL3_REG			(0x19c)	/* Link Control 3 Register. */
#define PCIE_LANE_ERR_STATUS_REG		(0x1a0)	/* Lane Error Status Register. */
#define PCIE_LANE_EQUALIZATION_CONTROL01_REG	(0x1a4)	/* Equalization Control Register for Lanes 1-0. */
#define PCIE_LANE_EQUALIZATION_CONTROL23_REG	(0x1a8)	/* Equalization Control Register for Lanes 3-2. */
#define PCIE_LANE_EQUALIZATION_CONTROL45_REG	(0x1ac)	/* Equalization Control Register for Lanes 5-4. */
#define PCIE_LANE_EQUALIZATION_CONTROL67_REG	(0x1b0)	/* Equalization Control Register for Lanes 7-6. */
#define PCIE_LANE_EQUALIZATION_CONTROL89_REG	(0x1b4)	/* Equalization Control Register for Lanes 9-8. */
#define PCIE_LANE_EQUALIZATION_CONTROL1011_REG	(0x1b8)	/* Equalization Control Register for Lanes 11-10. */
#define PCIE_LANE_EQUALIZATION_CONTROL1213_REG	(0x1bc)	/* Equalization Control Register for Lanes 13-12. */
#define PCIE_LANE_EQUALIZATION_CONTROL1415_REG	(0x1c0)	/* Equalization Control Register for Lanes 15-14. */
#define PCIE_TPH_EXT_CAP_HDR_REG		(0x1f8)	/* TPH Extended Capability Header. */
#define PCIE_TPH_REQ_CAP_REG_REG		(0x1fc)	/* TPH Requestor Capability Register. */
#define PCIE_TPH_REQ_CONTROL_REG_REG		(0x200)	/* TPH Requestor Control Register. */
#define PCIE_TPH_ST_TABLE_REG_0			(0x204)	/* TPH ST Table Register 0. */
#define PCIE_TPH_ST_TABLE_REG_1			(0x208)	/* TPH ST Table Register 1. */
#define PCIE_TPH_ST_TABLE_REG_2			(0x20c)	/* TPH ST Table Register 2. */
#define PCIE_TPH_ST_TABLE_REG_3			(0x210)	/* TPH ST Table Register 3. */
#define PCIE_TPH_ST_TABLE_REG_4			(0x214)	/* TPH ST Table Register 4. */
#define PCIE_TPH_ST_TABLE_REG_5			(0x218)	/* TPH ST Table Register 5. */
#define PCIE_TPH_ST_TABLE_REG_6			(0x21c)	/* TPH ST Table Register 6. */
#define PCIE_TPH_ST_TABLE_REG_7			(0x220)	/* TPH ST Table Register 7. */
#define PCIE_L1SUB_CAP_HEADER_REG		(0x2e0)	/* L1 Substates Extended Capability Header. */
#define PCIE_L1SUB_CAPABILITY_REG		(0x2e4)	/* L1 Substates Capability Register. */
#define PCIE_L1SUB_CONTROL1_REG			(0x2e8)	/* L1 Substates Control 1 Register. */
#define PCIE_L1SUB_CONTROL2_REG			(0x2ec)	/* L1 Substates Control 2 Register. */
#define PCIE_ACK_LATENCY_TIMER_OFF		(0x700)	/* Ack Latency Timer and Replay Timer Register. */
#define PCIE_VENDOR_SPEC_DLLP_OFF		(0x704)	/* Vendor Specific DLLP Register. */
#define PCIE_PORT_FORCE_OFF			(0x708)	/* Port Force Link Register. */
#define PCIE_ACK_F_ASPM_CTRL_OFF		(0x70c)	/* Ack Frequency and L0-L1 ASPM Control Register. */
#define PCIE_PORT_LINK_CTRL_OFF			(0x710)	/* Port Link Control Register. */
#define PCIE_LANE_SKEW_OFF			(0x714)	/* Lane Skew Register. */
#define PCIE_TIMER_CTRL_MAX_FUNC_NUM_OFF	(0x718)	/* Timer Control and Max Function Number Register. */
#define PCIE_SYMBOL_TIMER_FILTER_1_OFF		(0x71c)	/* Symbol Timer Register and Filter Mask 1 . */
#define PCIE_FILTER_MASK_2_OFF			(0x720)	/* Filter Mask 2 . */
#define PCIE_AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF	(0x724)	/* AMBA Multiple Outbound Decomposed NP SubRequests Control Register. */
#define PCIE_PL_DEBUG0_OFF			(0x728)	/* Debug Register 0. */
#define PCIE_PL_DEBUG1_OFF			(0x72c)	/* Debug Register 1. */
#define PCIE_TX_P_FC_CREDIT_STATUS_OFF		(0x730)	/* Transmit Posted FC Credit Status. */
#define PCIE_TX_NP_FC_CREDIT_STATUS_OFF		(0x734)	/* Transmit Non-Posted FC Credit Status. */
#define PCIE_TX_CPL_FC_CREDIT_STATUS_OFF	(0x738)	/* Transmit Completion FC Credit Status. */
#define PCIE_QUEUE_STATUS_OFF			(0x73c)	/* Queue Status. */
#define PCIE_VC_TX_ARBI_1_OFF			(0x740)	/* VC Transmit Arbitration Register 1. */
#define PCIE_VC_TX_ARBI_2_OFF			(0x744)	/* VC Transmit Arbitration Register 2. */
#define PCIE_VC0_P_RX_Q_CTRL_OFF		(0x748)	/* Segmented-Buffer VC0 Posted Receive Queue Control . */
#define PCIE_VC0_NP_RX_Q_CTRL_OFF		(0x74c)	/* Segmented-Buffer VC0 Non-Posted Receive Queue Control . */
#define PCIE_VC0_CPL_RX_Q_CTRL_OFF		(0x750)	/* Segmented-Buffer VC0 Completion Receive Queue Control . */
#define PCIE_VC1_P_RX_Q_CTRL_OFF		(0x754)	/* Segmented-Buffer VC1 Posted Receive Queue Control . */
#define PCIE_VC1_NP_RX_Q_CTRL_OFF		(0x758)	/* Segmented-Buffer VC1 Non-Posted Receive Queue Control . */
#define PCIE_VC1_CPL_RX_Q_CTRL_OFF		(0x75c)	/* Segmented-Buffer VC1 Completion Receive Queue Control . */
#define PCIE_VC2_P_RX_Q_CTRL_OFF		(0x760)	/* Segmented-Buffer VC2 Posted Receive Queue Control. */
#define PCIE_VC2_NP_RX_Q_CTRL_OFF		(0x764)	/* Segmented-Buffer VC2 Non-Posted Receive Queue Control. */
#define PCIE_VC2_CPL_RX_Q_CTRL_OFF		(0x768)	/* Segmented-Buffer VC2 Completion Receive Queue Control. */
#define PCIE_VC3_P_RX_Q_CTRL_OFF		(0x76c)	/* Segmented-Buffer VC3 Posted Receive Queue Control. */
#define PCIE_VC3_NP_RX_Q_CTRL_OFF		(0x770)	/* Segmented-Buffer VC3 Non-Posted Receive Queue Control. */
#define PCIE_VC3_CPL_RX_Q_CTRL_OFF		(0x774)	/* Segmented-Buffer VC3 Completion Receive Queue Control. */
#define PCIE_VC4_P_RX_Q_CTRL_OFF		(0x778)	/* Segmented-Buffer VC4 Posted Receive Queue Control. */
#define PCIE_VC4_NP_RX_Q_CTRL_OFF		(0x77c)	/* Segmented-Buffer VC4 Non-Posted Receive Queue Control. */
#define PCIE_VC4_CPL_RX_Q_CTRL_OFF		(0x780)	/* Segmented-Buffer VC4 Completion Receive Queue Control. */
#define PCIE_VC5_P_RX_Q_CTRL_OFF		(0x784)	/* Segmented-Buffer VC5 Posted Receive Queue Control. */
#define PCIE_VC5_NP_RX_Q_CTRL_OFF		(0x788)	/* Segmented-Buffer VC5 Non-Posted Receive Queue Control. */
#define PCIE_VC5_CPL_RX_Q_CTRL_OFF		(0x78c)	/* Segmented-Buffer VC5 Completion Receive Queue Control. */
#define PCIE_VC6_P_RX_Q_CTRL_OFF		(0x790)	/* Segmented-Buffer VC6 Posted Receive Queue Control. */
#define PCIE_VC6_NP_RX_Q_CTRL_OFF		(0x794)	/* Segmented-Buffer VC6 Non-Posted Receive Queue Control. */
#define PCIE_VC6_CPL_RX_Q_CTRL_OFF		(0x798)	/* Segmented-Buffer VC6 Completion Receive Queue Control. */
#define PCIE_VC7_P_RX_Q_CTRL_OFF		(0x79c)	/* Segmented-Buffer VC7 Posted Receive Queue Control. */
#define PCIE_VC7_NP_RX_Q_CTRL_OFF		(0x7a0)	/* Segmented-Buffer VC7 Non-Posted Receive Queue Control. */
#define PCIE_VC7_CPL_RX_Q_CTRL_OFF		(0x7a4)	/* Segmented-Buffer VC7 Completion Receive Queue Control. */
#define PCIE_GEN2_CTRL_OFF			(0x80c)	/* Link Width and Speed Change Control Register. */
#define PCIE_PHY_STATUS_OFF			(0x810)	/* PHY Status Register. */
#define PCIE_PHY_CONTROL_OFF			(0x814)	/* PHY Control Register. */
#define PCIE_MSI_CTRL_ADDR_OFF			(0x820)	/* MSI Controller Address Register. */
#define PCIE_MSI_CTRL_UPPER_ADDR_OFF		(0x824)	/* MSI Controller Upper Address Register. */
#define MSI_INTERRUPT_OFF			(12)
#define PCIE_MSI_CTRL_INT_0_EN_OFF		(0x828)	/* MSI Controller Interrupt#0 Enable Register. */
#define PCIE_MSI_CTRL_INT_0_MASK_OFF		(0x82c)	/* MSI Controller Interrupt#0 Mask Register. */
#define PCIE_MSI_CTRL_INT_0_STATUS_OFF		(0x830)	/* MSI Controller Interrupt#0 Status Register. */
#define PCIE_MSI_CTRL_INT_1_EN_OFF		(0x834)	/* MSI Controller Interrupt#1 Enable Register. */
#define PCIE_MSI_CTRL_INT_1_MASK_OFF		(0x838)	/* MSI Controller Interrupt#1 Mask Register. */
#define PCIE_MSI_CTRL_INT_1_STATUS_OFF		(0x83c)	/* MSI Controller Interrupt#1 Status Register. */
#define PCIE_MSI_CTRL_INT_2_EN_OFF		(0x840)	/* MSI Controller Interrupt#2 Enable Register. */
#define PCIE_MSI_CTRL_INT_2_MASK_OFF		(0x844)	/* MSI Controller Interrupt#2 Mask Register. */
#define PCIE_MSI_CTRL_INT_2_STATUS_OFF		(0x848)	/* MSI Controller Interrupt#2 Status Register. */
#define PCIE_MSI_CTRL_INT_3_EN_OFF		(0x84c)	/* MSI Controller Interrupt#3 Enable Register. */
#define PCIE_MSI_CTRL_INT_3_MASK_OFF		(0x850)	/* MSI Controller Interrupt#3 Mask Register. */
#define PCIE_MSI_CTRL_INT_3_STATUS_OFF		(0x854)	/* MSI Controller Interrupt#3 Status Register. */
#define PCIE_MSI_CTRL_INT_4_EN_OFF		(0x858)	/* MSI Controller Interrupt#4 Enable Register. */
#define PCIE_MSI_CTRL_INT_4_MASK_OFF		(0x85c)	/* MSI Controller Interrupt#4 Mask Register. */
#define PCIE_MSI_CTRL_INT_4_STATUS_OFF		(0x860)	/* MSI Controller Interrupt#4 Status Register. */
#define PCIE_MSI_CTRL_INT_5_EN_OFF		(0x864)	/* MSI Controller Interrupt#5 Enable Register. */
#define PCIE_MSI_CTRL_INT_5_MASK_OFF		(0x868)	/* MSI Controller Interrupt#5 Mask Register. */
#define PCIE_MSI_CTRL_INT_5_STATUS_OFF		(0x86c)	/* MSI Controller Interrupt#5 Status Register. */
#define PCIE_MSI_CTRL_INT_6_EN_OFF		(0x870)	/* MSI Controller Interrupt#6 Enable Register. */
#define PCIE_MSI_CTRL_INT_6_MASK_OFF		(0x874)	/* MSI Controller Interrupt#6 Mask Register. */
#define PCIE_MSI_CTRL_INT_6_STATUS_OFF		(0x878)	/* MSI Controller Interrupt#6 Status Register. */
#define PCIE_MSI_CTRL_INT_7_EN_OFF		(0x87c)	/* MSI Controller Interrupt#7 Enable Register. */
#define PCIE_MSI_CTRL_INT_7_MASK_OFF		(0x880)	/* MSI Controller Interrupt#7 Mask Register. */
#define PCIE_MSI_CTRL_INT_7_STATUS_OFF		(0x884)	/* MSI Controller Interrupt#7 Status Register. */
#define PCIE_MSI_GPIO_IO_OFF			(0x888)	/* MSI Controller General Purpose IO Register. */
#define PCIE_GEN3_RELATED_OFF			(0x890)	/* Gen3 Control Register. */
#define PCIE_GEN3_EQ_LOCAL_FS_LF_OFF		(0x894)	/* Gen3 EQ FS and LF Register. */
#define PCIE_GEN3_EQ_PSET_INDEX_OFF		(0x89c)	/* Gen3 EQ Preset Index Register. */
#define PCIE_GEN3_EQ_COEFF_LEGALITY_STATUS_OFF	(0x8a4)	/* Gen3 EQ Status Register. */
#define PCIE_GEN3_EQ_CONTROL_OFF		(0x8a8)	/* Gen3 EQ Control Register. */
#define PCIE_GEN3_EQ_FB_MODE_DIR_CHANGE_OFF	(0x8ac)	/* Gen3 EQ Direction Change Feedback Mode Control Register. */
#define PCIE_PIPE_LOOPBACK_CONTROL_OFF		(0x8b8)	/* PIPE Loopback Control Register. */
#define PCIE_MISC_CONTROL_1_OFF			(0x8bc)	/* DBI Read-Only Write Enable Register. */
#define PCIE_AMBA_ERROR_RESPONSE_DEFAULT_OFF	(0x8d0)	/* AHB/AXI Bridge Slave Error Response Register. */
#define PCIE_AMBA_LINK_TIMEOUT_OFF		(0x8d4)	/* Link Down AXI Bridge Slave Timeout Register. */
#define PCIE_AMBA_ORDERING_CTRL_OFF		(0x8d8)	/* AMBA Ordering Control. */
#define PCIE_AMBA_ORDRMGR_WDOG_OFF		(0x8dc)	/* AHB/AXI Ordering Manager Watchdog Timer. */
#define PCIE_COHERENCY_CONTROL_1_OFF		(0x8e0)	/* ACE Cache Coherency Control Register 1. */
#define PCIE_COHERENCY_CONTROL_2_OFF		(0x8e4)	/* ACE Cache Coherency Control Register 2. */
#define PCIE_COHERENCY_CONTROL_3_OFF		(0x8e8)	/* ACE Cache Coherency Control Register 3. */
#define PCIE_PL_LAST_OFF			(0x8fc)	/* This is an internally reserved register. */
#define PCIE_IATU_VIEWPORT_OFF			(0x900)	/* iATU Index Register. */
#define PCIE_IATU_REGION_CTRL_1_OFF_OUTBOUND_0	(0x904)	/* iATU Region Control 1 Register. */
#define PCIE_IATU_REGION_CTRL_2_OFF_OUTBOUND_0	(0x908)	/* iATU Region Control 2 Register. */
#define PCIE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0	(0x90c) /* The start address of the address region to be translated.*/
#define PCIE_IATU_UPR_BASE_ADDR_OFF_OUTBOUND_0	(0x910)
#define PCIE_IATU_LIMIT_ADDR_OFF_OUTBOUND_0	(0x914) /* iATU Limit Address Register */
#define PCIE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0	(0x918) /* iATU Outbound Region#N Lower Offset Address Register */
#define PCIE_IATU_UPR_TARGET_ADDR_OFF_OUTBOUND_0	(0x91C)
#define PCIE_DMA_CTRL_OFF			(0x978)	/* DMA Number of Channels Register. */
#define PCIE_DMA_WRITE_ENGINE_EN_OFF		(0x97c)	/* DMA Write Engine Enable Register. */
#define PCIE_DMA_WRITE_DOORBELL_OFF		(0x980)	/* DMA Write Doorbell Register. */
#define PCIE_DMA_WRITE_CHANNEL_ARB_WEIGHT_LOW_OFF	(0x988)	/* DMA Write Engine Channel Arbitration Weight Low Register. */
#define PCIE_DMA_WRITE_CHANNEL_ARB_WEIGHT_HIGH_OFF	(0x98c)	/* DMA Write Engine Channel Arbitration Weight High Register. */
#define PCIE_DMA_WRITE_P_REQ_TIMER_OFF		(0x998)	/* DMA Write Posted Request Deadlock Timer Register. */
#define PCIE_DMA_READ_ENGINE_EN_OFF		(0x99c)	/* DMA Read Engine Enable Register. */
#define PCIE_DMA_READ_DOORBELL_OFF		(0x9a0)	/* DMA Read Doorbell Register. */
#define PCIE_DMA_READ_CHANNEL_ARB_WEIGHT_LOW_OFF	(0x9a8)	/* DMA Read Engine Channel Arbitration Weight Low Register. */
#define PCIE_DMA_READ_CHANNEL_ARB_WEIGHT_HIGH_OFF	(0x9ac)	/* DMA Read Engine Channel Arbitration Weight High Register. */
#define PCIE_DMA_WRITE_INT_STATUS_OFF		(0x9bc)	/* DMA Write Interrupt Status Register. */
#define PCIE_DMA_WRITE_INT_MASK_OFF		(0x9c4)	/* DMA Write Interrupt Mask Register. */
#define PCIE_DMA_WRITE_INT_CLEAR_OFF		(0x9c8)	/* DMA Write Interrupt Clear Register. */
#define PCIE_DMA_WRITE_ERR_STATUS_OFF		(0x9cc)	/* DMA Write Error Status Register. */
#define PCIE_DMA_WRITE_DONE_IMWR_LOW_OFF	(0x9d0)	/* DMA Write Done IMWr Address Low Register. */
#define PCIE_DMA_WRITE_DONE_IMWR_HIGH_OFF	(0x9d4)	/* DMA Write Done IMWr Interrupt Address High Register. */
#define PCIE_DMA_WRITE_ABORT_IMWR_LOW_OFF	(0x9d8)	/* DMA Write Abort IMWr Address Low Register. */
#define PCIE_DMA_WRITE_ABORT_IMWR_HIGH_OFF	(0x9dc)	/* DMA Write Abort IMWr Address High Register. */
#define PCIE_DMA_WRITE_CH01_IMWR_DATA_OFF	(0x9e0)	/* DMA Write Channel 1 and 0 IMWr Data Register. */
#define PCIE_DMA_WRITE_CH23_IMWR_DATA_OFF	(0x9e4)	/* DMA Write Channel 3 and 2 IMWr Data Register. */
#define PCIE_DMA_WRITE_CH45_IMWR_DATA_OFF	(0x9e8)	/* DMA Write Channel 5 and 4 IMWr Data Register. */
#define PCIE_DMA_WRITE_CH67_IMWR_DATA_OFF	(0x9ec)	/* DMA Write Channel 7 and 6 IMWr Data Register. */
#define PCIE_DMA_WRITE_LINKED_LIST_ERR_EN_OFF	(0xa00)	/* DMA Write Linked List Error Enable Register. */
#define PCIE_DMA_READ_INT_STATUS_OFF		(0xa10)	/* DMA Read Interrupt Status Register. */
#define PCIE_DMA_READ_INT_MASK_OFF		(0xa18)	/* DMA Read Interrupt Mask Register. */
#define PCIE_DMA_READ_INT_CLEAR_OFF		(0xa1c)	/* DMA Read Interrupt Clear Register. */
#define PCIE_DMA_READ_ERR_STATUS_LOW_OFF	(0xa24)	/* DMA Read Error Status Low Register. */
#define PCIE_DMA_READ_ERR_STATUS_HIGH_OFF	(0xa28)	/* DMA Read Error Status High Register. */
#define PCIE_DMA_READ_LINKED_LIST_ERR_EN_OFF	(0xa34)	/* DMA Read Linked List Error Enable Register. */
#define PCIE_DMA_READ_DONE_IMWR_LOW_OFF		(0xa3c)	/* DMA Read Done IMWr Address Low Register. */
#define PCIE_DMA_READ_DONE_IMWR_HIGH_OFF	(0xa40)	/* DMA Read Done IMWr Address High Register. */
#define PCIE_DMA_READ_ABORT_IMWR_LOW_OFF	(0xa44)	/* DMA Read Abort IMWr Address Low Register. */
#define PCIE_DMA_READ_ABORT_IMWR_HIGH_OFF	(0xa48)	/* DMA Read Abort IMWr Address High Register. */
#define PCIE_DMA_READ_CH01_IMWR_DATA_OFF	(0xa4c)	/* DMA Read Channel 1 and 0 IMWr Data Register. */
#define PCIE_DMA_READ_CH23_IMWR_DATA_OFF	(0xa50)	/* DMA Read Channel 3 and 2 IMWr Data Register. */
#define PCIE_DMA_READ_CH45_IMWR_DATA_OFF	(0xa54)	/* DMA Read Channel 5 and 4 IMWr Data Register. */
#define PCIE_DMA_READ_CH67_IMWR_DATA_OFF	(0xa58)	/* DMA Read Channel 7 and 6 IMWr Data Register. */
#define PCIE_DMA_VIEWPORT_SEL_OFF		(0xa6c)	/* DMA Channel Context Index Register. */
#define PCIE_PL_LTR_LATENCY_OFF			(0xb30)	/* LTR Latency Register. */
#define PCIE_AUX_CLK_FREQ_OFF			(0xb40)	/* Auxiliary Clock Frequency Control Register. */
#define PCIE_L1_SUBSTATES_OFF			(0xb44)	/* L1 Substates Timing Register. */
/* Baikal-specific registers. */
#define PCIE_BK_MGMT_SEL_LANE			(0xd04) /* Select lane. */
#define PCIE_BK_MGMT_CTRL			(0xd08) /* Control management register. */
#define PCIE_BK_MGMT_WRITE_DATA			(0xd0c) /* Data write register. */
#define PCIE_BK_MGMT_READ_DATA			(0xd10) /* Data read register. */

/* PCIE_BK_MGMT_CTRL */
#define BK_MGMT_CTRL_ADDR_MASK			(0x1FFFFF) /* [21:0] bits */
#define BK_MGMT_CTRL_READ			(0 << 29)
#define BK_MGMT_CTRL_WRITE			(1 << 29)
#define BK_MGMT_CTRL_DONE			(1 << 30)
#define BK_MGMT_CTRL_BUSY			(1 << 31)

/* PCIE_MISC_CONTROL_1_OFF */
#define DBI_RO_WR_EN				(1 << 0)	/* Write to RO Registers Using DBI. */

/* PCIE_PORT_LINK_CTRL_OFF */
#define FAST_LINK_MODE				(1 << 7)	/* Fast Link Mode. */
#define LINK_CAPABLE_SHIFT			(16)		/* Link Mode Enable. */
#define LINK_CAPABLE_MASK			0x3F0000

/* GEN2_CTRL_OFF */
#define NUM_OF_LANES_SHIFT			(8)		/* Predetermined Number of Lanes. */
#define NUM_OF_LANES_MASK			0x1F00
#define DIRECT_SPEED_CHANGE			(1 << 17)

/* GEN3_EQ_CONTROL_OFF */
#define GEN3_EQ_EVAL_2MS_DISABLE		(1 << 5)	/* Phase2_3 2 ms Timeout Disable. */
#define GEN3_EQ_FB_MODE_SHIFT			(0)		/* Feedback Mode */
#define GEN3_EQ_FB_MODE_MASK			0xF
#define GEN3_EQ_PSET_REQ_VEC_SHIFT		(8)		/* Preset Request Vector. */
#define GEN3_EQ_PSET_REQ_VEC_MASK		0xFFFF00

/* LINK_CONTROL_LINK_STATUS_REG */
#define PCIE_CAP_LINK_SPEED_SHIFT		16
#define PCIE_CAP_LINK_SPEED_MASK		0xF0000
#define PCIE_CAP_LINK_SPEED_GEN1		0x1
#define PCIE_CAP_LINK_SPEED_GEN2		0x2
#define PCIE_CAP_LINK_SPEED_GEN3		0x3
#define PCIE_STA_LINK_TRAINING			0x8000000
#define PCIE_STA_LINK_WIDTH_MASK		0x3f00000
#define PCIE_STA_LINK_WIDTH_SHIFT		(20)

/* IATU_VIEWPORT_OFF */
#define REGION_DIR_SHIFT			(31)		/* Region Direction. */
#define REGION_INDEX_SHIFT			(0)		/* Region Index. */
#define REGION_DIR_OUTBOUND			(0)
#define REGION_DIR_INBOUND			(1)

/* TYPE1_STATUS_COMMAND_REG */
#define TYPE1_STATUS_COMMAND_REG_BME		(1 << 2)
#define TYPE1_STATUS_COMMAND_REG_MSE		(1 << 1)
#define TYPE1_STATUS_COMMAND_REG_IOSE		(1 << 0)

/* IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0 */
#define LWR_BASE_RW_SHIFT			(16)

/* IATU_LIMIT_ADDR_OFF_OUTBOUND_0 */
#define LIMIT_ADDR_RW_SHIFT			(16)

/* IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0 */
#define LWR_TARGET_RW_SHIFT			(16)

/* IATU_REGION_CTRL_1_OFF_OUTBOUND_0 */
#define IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_SHIFT	(0)
#define TLP_TYPE_MEM				(0)
#define TLP_TYPE_IO				(2)
#define TLP_TYPE_CFGRD0				(4)
#define TLP_TYPE_CFGRD1				(5)

/* IATU_REGION_CTRL_2_OFF_OUTBOUND_0 */
#define IATU_REGION_CTRL_2_OFF_OUTBOUND_0_REGION_EN	(1 << 31)

/* PCIE_LINK_CONTROL2_LINK_STATUS2 */
#define PCIE_LINK_CONTROL2_GEN_MASK		(0xF)
#define PCIE_LINK_CONTROL2_GEN1			(1)
#define PCIE_LINK_CONTROL2_GEN2			(2)
#define PCIE_LINK_CONTROL2_GEN3			(3)

/* PHY control registers. */
#define PCIE_PHY_DWC_GLBL_PLL_CFG_0		(0x1c000)	/* PLL Global Configuration Register #0 */
#define PCIE_PHY_DWC_GLBL_PLL_CFG_1		(0x1c001)	/* PLL Global Configuration Register #1 */
#define PCIE_PHY_DWC_GLBL_PLL_CFG_2		(0x1c002)	/* PLL Global Configuration Register #2 */
#define PCIE_PHY_DWC_GLBL_PLL_CFG_3		(0x1c003)	/* PLL Global Configuration Register #3 */
#define PCIE_PHY_DWC_GLBL_PLL_CFG_4		(0x1c004)	/* PLL Global Configuration Register #4 */
#define PCIE_PHY_DWC_GLBL_MISC_CONFIG_0		(0x1c005)	/* Global Miscellaneous Configuration #0 */
#define PCIE_PHY_DWC_GLBL_MISC_CONFIG_1		(0x1c006)	/* Global Miscellaneous Configuration #1 */
#define PCIE_PHY_DWC_SLICE_CFG			(0x1c00c)	/* Slice Configuration */
#define PCIE_PHY_DWC_GLBL_REGU_CFG		(0x1c00d)	/* Global Regulator Configuration */
#define PCIE_PHY_DWC_GLBL_TERM_CFG		(0x1c00e)	/* Global Termination Calibration Configuration */
#define PCIE_PHY_DWC_GLBL_CAL_CFG		(0x1c00f)	/* Global PLL Calibration Configuration */
#define PCIE_PHY_DWC_GLBL_RD_SYNC_STATUS	(0x1c010)	/* Global Read Synchronization Status */
#define PCIE_PHY_DWC_RX_PWR_CTRL_P0		(0x1c014)	/* RX Power Controls in Power State P0 */
#define PCIE_PHY_DWC_RX_PWR_CTRL_P0S		(0x1c015)	/* RX Power Controls in Power State P0S */
#define PCIE_PHY_DWC_RX_PWR_CTRL_P1		(0x1c016)	/* RX Power Controls in Power State P1 */
#define PCIE_PHY_DWC_RX_PWR_CTRL_P2		(0x1c017)	/* RX Power Controls in Power State P2 */
#define PCIE_PHY_DWC_TX_PWR_CTRL_P0_P0S		(0x1c018)	/* TX Power Controls in Power States P0 and POS */
#define PCIE_PHY_DWC_TX_PWR_CTRL_P1_P2		(0x1c019)	/* TX Power Controls in Power States P1 and P2 */
#define PCIE_PHY_DWC_GLBL_PWR_CTRL		(0x1c01a)	/* Global Power State Machine Control Override */
#define PCIE_PHY_DWC_RX_TXDIR_CTRL_0		(0x1c01d)	/* Far-end TX Direction Control Register #0 */
#define PCIE_PHY_DWC_RX_TXDIR_CTRL_1		(0x1c01e)	/* Far-end TX Direction Control Register #1 */
#define PCIE_PHY_DWC_RX_TXDIR_CTRL_2		(0x1c01f)	/* Far-end TX Direction Control Register #2 */
#define PCIE_PHY_DWC_GLBL_PLL_MONITOR		(0x1c020)	/* Monitor for SerDes Global to Raw PCS Global Interface */
#define PCIE_PHY_DWC_GLBL_TERM_MON_1		(0x1c022)	/* Monitor for SerDes Global to Raw PCS Global Interface */
#define PCIE_PHY_DWC_GLBL_SDS_PIN_MON_0		(0x1c023)	/* Monitor for Raw PCS Global to SerDes Global to Raw PCS Interface */
#define PCIE_PHY_DWC_GLBL_SDS_PIN_MON_1		(0x1c024)	/* Monitor for Raw PCS Global to SerDes Global to Raw PCS Interface */
#define PCIE_PHY_DWC_GLBL_PWR_MON_0		(0x1c025)	/* Monitor of Global Power State Machine Values */
#define PCIE_PHY_DWC_GLBL_PWR_MON_1		(0x1c026)	/* Monitor of Global Power State Machine Values */
#define PCIE_PHY_DWC_GLBL_PWR_MON_2		(0x1c027)	/* Monitor of Global Power State Machine Values */
#define PCIE_PHY_DWC_GLBL_PLL_SSC_FRAC_BASE	(0x1c060)	/* Global PLL SSC Fractional Base */
#define PCIE_PHY_DWC_GLBL_PLL_SSC_CYCLES	(0x1c061)	/* Global PLL SSC Cycles Configuration */
#define PCIE_PHY_DWC_GLBL_PLL_SSC_FMFREQ	(0x1c062)	/* Global PLL SSC Modulation Frequency */
#define PCIE_PHY_DWC_GLBL_PLL_SSC_FREF		(0x1c063)	/* Global PLL SSC Reference Frequency */
#define PCIE_PHY_DWC_GLBL_PLL_SSC_PPM		(0x1c064)	/* Global PLL SSC PPM */
#define PCIE_PHY_DWC_GLBL_PLL_SSC_CFG		(0x1c065)	/* Global PLL SSC Configuration */
#define PCIE_PHY_DWC_GLBL_PLL_SSC_ALU_CMD	(0x1c067)	/* Global PLL SSC ALU Command */
#define PCIE_PHY_DWC_GLBL_PLL_SSC_MON		(0x1c069)	/* Global PLL SSC Monitor */
#define PCIE_PHY_DWC_GLBL_PLL_SSC_ALU_OUT_0	(0x1c06b)	/* Global PLL SSC ALU Output Register #0 */
#define PCIE_PHY_DWC_GLBL_PLL_SSC_ALU_OUT_1	(0x1c06c)	/* Global PLL SSC ALU Output Register #1 */
#define PCIE_PHY_DWC_GLBL_PLL_SSC_DIV		(0x1c06d)	/* Global PLL SSC Divider */
#define PCIE_PHY_DWC_GLBL_PLL_SSC_FRAC		(0x1c06e)	/* Global PLL SSC Fraction */
#define PCIE_PHY_DWC_GLBL_TAD			(0x1c080)	/* Global Test Analog and Digital Monitor */
#define PCIE_PHY_DWC_GLBL_TM_ADMON		(0x1c081)	/* Global Test Mode Analog/Digital Monitor Enable */
#define PCIE_PHY_DWC_EQ_WAIT_TIME		(0x3c000)	/* TX and RX Equalization Wait Times */
#define PCIE_PHY_DWC_RDET_TIME			(0x3c001)	/* Receiver Detect Wait Times */
#define PCIE_PHY_DWC_PCS_LANE_LINK_CFG		(0x3c002)	/* Link Configuration Override */
#define PCIE_PHY_DWC_PCS_PLL_CTLIFC_0		(0x3c003)	/* PLL Control Interface Override Register #0 */
#define PCIE_PHY_DWC_PCS_PLL_CTLIFC_1		(0x3c004)	/* PLL Control Interface Override Register #1 */
#define PCIE_PHY_DWC_PCS_REG_RD_TIMEOUT		(0x3c005)	/* Register Read Timeout */
#define PCIE_PHY_DWC_PCS_PLL_PCIE1_MODE_0	(0x3c006)	/* PLL Configuration Register #0 for PCIe1 */
#define PCIE_PHY_DWC_PCS_PLL_PCIE1_MODE_1	(0x3c007)	/* PLL Configuration Register #1 for PCIe1 */
#define PCIE_PHY_DWC_PCS_LANE_PCIE1_MODE_0	(0x3c008)	/* Lane Configuration Register #0 for PCIe1 */
#define PCIE_PHY_DWC_PCS_LANE_PCIE1_MODE_1	(0x3c009)	/* Lane Configuration Register #1 for PCIe1 */
#define PCIE_PHY_DWC_PCS_PLL_PCIE2_MODE_0	(0x3c00a)	/* PLL Configuration Register #0 for PCIe2 */
#define PCIE_PHY_DWC_PCS_PLL_PCIE2_MODE_1	(0x3c00b)	/* PLL Configuration Register #1 for PCIe2 */
#define PCIE_PHY_DWC_PCS_LANE_PCIE2_MODE_0	(0x3c00c)	/* Lane Configuration Register #0 for PCIe2 */
#define PCIE_PHY_DWC_PCS_LANE_PCIE2_MODE_1	(0x3c00d)	/* Lane Configuration Register #1 for PCIe2 */
#define PCIE_PHY_DWC_PCS_PLL_PCIE3_MODE_0	(0x3c00e)	/* PLL Configuration Register #0 for PCIe3 */
#define PCIE_PHY_DWC_PCS_PLL_PCIE3_MODE_1	(0x3c00f)	/* PLL Configuration Register #1 for PCIe3 */
#define PCIE_PHY_DWC_PCS_LANE_PCIE3_MODE_0	(0x3c010)	/* Lane Configuration Register #0 for PCIe3 */
#define PCIE_PHY_DWC_PCS_LANE_PCIE3_MODE_1	(0x3c011)	/* Lane Configuration Register #1 for PCIe3 */
#define PCIE_PHY_DWC_PCS_PLL_KX_MODE_1		(0x3c013)	/* PLL Configuration Register #1 for KX */
#define PCIE_PHY_DWC_PCS_LANE_KX_MODE_0		(0x3c014)	/* Lane Configuration Register #0 for KX */
#define PCIE_PHY_DWC_PCS_LANE_KX_MODE_1		(0x3c015)	/* Lane Configuration Register #1 for KX */
#define PCIE_PHY_DWC_PCS_PLL_KX4_MODE_0		(0x3c016)	/* PLL Configuration Register #0 for KX4 */
#define PCIE_PHY_DWC_PCS_PLL_KX4_MODE_1		(0x3c017)	/* PLL Configuration Register #1 for KX4 */
#define PCIE_PHY_DWC_PCS_LANE_KX4_MODE_0	(0x3c018)	/* Lane Configuration Register #0 for KX4 */
#define PCIE_PHY_DWC_PCS_LANE_KX4_MODE_1	(0x3c019)	/* Lane Configuration Register #1 for KX4 */
#define PCIE_PHY_DWC_PCS_PLL_KR_MODE_0		(0x3c01a)	/* PLL Configuration Register #0 for KR */
#define PCIE_PHY_DWC_PCS_PLL_KR_MODE_1		(0x3c01b)	/* PLL Configuration Register #1 for KR */
#define PCIE_PHY_DWC_PCS_LANE_KR_MODE_0		(0x3c01c)	/* Lane Configuration Register #0 for KR */
#define PCIE_PHY_DWC_PCS_LANE_KR_MODE_1		(0x3c01d)	/* Lane Configuration Register #1 for KR */
#define PCIE_PHY_DWC_PCS_PLL_SGMII_MODE_0	(0x3c01e)	/* PLL Configuration Register #0 for SGMII */
#define PCIE_PHY_DWC_PCS_PLL_SGMII_MODE_1	(0x3c01f)	/* PLL Configuration Register #1 for SGMII */
#define PCIE_PHY_DWC_PCS_LANE_SGMII_MODE_0	(0x3c020)	/* Lane Configuration Register #0 for SGMII */
#define PCIE_PHY_DWC_PCS_LANE_SGMII_MODE_1	(0x3c021)	/* Lane Configuration Register #1 for SGMII */
#define PCIE_PHY_DWC_PCS_PLL_QSGMII_MODE_0	(0x3c022)	/* PLL Configuration Register #0 for QSGMII */
#define PCIE_PHY_DWC_PCS_PLL_QSGMII_MODE_1	(0x3c023)	/* PLL Configuration Register #1 for QSGMII */
#define PCIE_PHY_DWC_PCS_LANE_QSGMII_MODE_0	(0x3c024)	/* Lane Configuration Register #0 for QSGMII */
#define PCIE_PHY_DWC_PCS_LANE_QSGMII_MODE_1	(0x3c025)	/* Lane Configuration Register #1 for QSGMII */
#define PCIE_PHY_DWC_PCS_PLL_CEI_MODE_0		(0x3c026)	/* PLL Configuration Register #0 for CEI */
#define PCIE_PHY_DWC_PCS_PLL_CEI_MODE_1		(0x3c027)	/* PLL Configuration Register #1 for CEI */
#define PCIE_PHY_DWC_PCS_LANE_CEI_MODE_0		(0x3c028)	/* Lane Configuration Register #0 for CEI */
#define PCIE_PHY_DWC_PCS_LANE_CEI_MODE_1		(0x3c029)	/* Lane Configuration Register #1 for CEI */
#define PCIE_PHY_DWC_PCS_PLL_PCIE1_125M_MODE_0		(0x3c02a)	/* PLL Configuration Register #0 for PCIe1 with 125MHz refclk */
#define PCIE_PHY_DWC_PCS_PLL_PCIE1_125M_MODE_1		(0x3c02b)	/* PLL Configuration Register #1 for PCIe1 with 125MHz refclk */
#define PCIE_PHY_DWC_PCS_LANE_PCIE1_125M_MODE_0		(0x3c02c)	/* Lane Configuration Register #0 for PCIe1 with 125MHz refclk */
#define PCIE_PHY_DWC_PCS_LANE_PCIE1_125M_MODE_1		(0x3c02d)	/* Lane Configuration Register #1 for PCIe1 with 125MHz refclk */
#define PCIE_PHY_DWC_PCS_PLL_PCIE2_125M_MODE_0		(0x3c02e)	/* PLL Configuration Register #0 for PCIe2 with 125MHz refclk */
#define PCIE_PHY_DWC_PCS_PLL_PCIE2_125M_MODE_1		(0x3c02f)	/* PLL Configuration Register #1 for PCIe2 with 125MHz refclk */
#define PCIE_PHY_DWC_PCS_LANE_PCIE2_125M_MODE_0		(0x3c030)	/* Lane Configuration Register #0 for PCIe2 with 125MHz refclk */
#define PCIE_PHY_DWC_PCS_LANE_PCIE2_125M_MODE_1		(0x3c031)	/* Lane Configuration Register #1 for PCIe2 with 125MHz refclk */
#define PCIE_PHY_DWC_PCS_PLL_PCIE3_125M_MODE_0		(0x3c032)	/* PLL Configuration Register #0 for PCIe3 with 125MHz refclk */
#define PCIE_PHY_DWC_PCS_PLL_PCIE3_125M_MODE_1		(0x3c033)	/* PLL Configuration Register #1 for PCIe3 with 125MHz refclk */
#define PCIE_PHY_DWC_PCS_LANE_PCIE3_125M_MODE_0		(0x3c034)	/* Lane Configuration Register #0 for PCIe3 with 125MHz refclk */
#define PCIE_PHY_DWC_PCS_LANE_PCIE3_125M_MODE_1		(0x3c035)	/* Lane Configuration Register #1 for PCIe3 with 125MHz refclk */
#define PCIE_PHY_DWC_PCS_LANE_VMA_COARSE_CTRL_0		(0x3c036)	/* Lane VMA Coarse Control Register #0 */
#define PCIE_PHY_DWC_PCS_LANE_VMA_COARSE_CTRL_1		(0x3c037)	/* Lane VMA Coarse Control Register #1 */
#define PCIE_PHY_DWC_PCS_LANE_VMA_COARSE_CTRL_2		(0x3c038)	/* Lane VMA Coarse Control Register #2 */
#define PCIE_PHY_DWC_PCS_LANE_VMA_FINE_CTRL_0		(0x3c039)	/* Lane VMA Fine Control Register #0 */
#define PCIE_PHY_DWC_PCS_LANE_VMA_FINE_CTRL_1		(0x3c03a)	/* Lane VMA Fine Control Register #1 */
#define PCIE_PHY_DWC_PCS_LANE_VMA_FINE_CTRL_2		(0x3c03b)	/* Lane VMA Fine Control Register #2 */
#define PCIE_PHY_DWC_PCS_LANE_MODE_OVRD			(0x3c03c)	/* Lane Mode Override in Raw PCS Global and Slice */
#define PCIE_PHY_DWC_PCS_LANE_LINK_MON			(0x3c040)	/* Monitor of MAC to Raw PCS Link Configuration Interface */
#define PCIE_PHY_DWC_PCS_MAC_PLLIFC_MON_2		(0x3c043)	/* Monitor of MAC to Raw PCS PLL_PCS Divider Value */
#define PCIE_PHY_DWC_PCS_MAC_PLLIFC_MON_3		(0x3c044)	/* Monitor of MAC to Raw PCS PLL OP_Range and Divider Values */
#define PCIE_PHY_DWC_SLICE_TRIM			(0x1c040)	/* Slice TX and RX Bias Trim Settings */
#define PCIE_PHY_DWC_RX_LDLL_CTRL		(0x1c043)	/* RX Lane DLL Test Controls */
#define PCIE_PHY_DWC_RX_SDLL_CTRL		(0x1c044)	/* RX Slice DLL test controls */
#define PCIE_PHY_DWC_SLICE_PCIE1_MODE		(0x1c045)	/* Slice Configuration Settings for PCIE1 @ 100MHz */
#define PCIE_PHY_DWC_SLICE_PCIE2_MODE		(0x1c046)	/* Slice Configuration Settings for PCIE2 @ 100Mhz */
#define PCIE_PHY_DWC_SLICE_PCIE3_MODE		(0x1c047)	/* Slice Configuration Settings for PCIE3 @ 100Mhz */
#define PCIE_PHY_DWC_SLICE_KX_MODE		(0x1c048)	/* Slice Configuration Settings for KX */
#define PCIE_PHY_DWC_SLICE_KX4_MODE		(0x1c049)	/* Slice Configuration Settings for KX4 */
#define PCIE_PHY_DWC_SLICE_KR_MODE		(0x1c04a)	/* Slice Configuration Settings for KR */
#define PCIE_PHY_DWC_SLICE_SGMII_MODE		(0x1c04b)	/* Slice Configuration Settings for SGMII */
#define PCIE_PHY_DWC_SLICE_QSGMII_MODE		(0x1c04c)	/* Slice Configuration Settings for QSGMII */
#define PCIE_PHY_DWC_SLICE_CEI_MODE		(0x1c04d)	/* Slice Configuration Settings for CEI */
#define PCIE_PHY_DWC_SLICE_PCIE1_125M_MODE	(0x1c04e)	/* Slice Configuration Settings for PCIE1 @ 125MHz */
#define PCIE_PHY_DWC_SLICE_PCIE2_125M_MODE	(0x1c04f)	/* Slice Configuration Settings for PCIE2 @ 125MHz */
#define PCIE_PHY_DWC_SLICE_PCIE3_125M_MODE	(0x1c050)	/* Slice Configuration Settings for PCIE3 @ 125MHz */
#define PCIE_PHY_DWC_SLICE_OVRD_MODE		(0x1c051)	/* Slice Configuration Settings Override */
#define PCIE_PHY_DWC_RX_CFG_0			(0x18000)	/* Lane RX Configuration Register #0 */
#define PCIE_PHY_DWC_RX_CFG_1			(0x18001)	/* Lane RX Configuration Register #1 */
#define PCIE_PHY_DWC_RX_CFG_2			(0x18002)	/* Lane RX Configuration Register #2 */
#define PCIE_PHY_DWC_RX_CFG_3			(0x18003)	/* Lane RX Configuration Register #3 */
#define PCIE_PHY_DWC_RX_CFG_4			(0x18004)	/* Lane RX Configuration Register #4 */
#define PCIE_PHY_DWC_RX_CFG_5			(0x18005)	/* Lane RX Configuration Register #5 */
#define PCIE_PHY_DWC_RX_CDR_CTRL_0		(0x18006)	/* Lane RX CDR Control Register #0 */
#define PCIE_PHY_DWC_RX_CDR_CTRL_1		(0x18007)	/* Lane RX CDR Control Register #1 */
#define PCIE_PHY_DWC_RX_CDR_CTRL_2		(0x18008)	/* Lane RX CDR Control Register #2 */
#define PCIE_PHY_DWC_RX_LOOP_CTRL		(0x18009)	/* Lane RX Loop Control */
#define PCIE_PHY_DWC_RX_MISC_CTRL		(0x1800a)	/* Lane RX Miscellaneous Control */
#define PCIE_PHY_DWC_RX_CTLE_CTRL		(0x1800b)	/* Lane RX CTLE Control */
#define PCIE_PHY_DWC_RX_PRECORR_CTRL		(0x1800c)	/* Lane RX Pre-Correlation Control */
#define PCIE_PHY_DWC_RX_PHS_ACCM_CTRL		(0x1800d)	/* Lane RX Phase Accumulator Control */
#define PCIE_PHY_DWC_RX_PHS_ACCM_FR_VAL		(0x1800e)	/* Lane RX Phase Accumulator Frequency Portion Control */
#define PCIE_PHY_DWC_RX_PRECORR_VAL		(0x1800f)	/* Lane RX Pre-Correlation Count */
#define PCIE_PHY_DWC_RX_DELTA_PM_0		(0x18010)	/* Lane RX VMA Performance Metric Register #0 */
#define PCIE_PHY_DWC_RX_DELTA_PM_1		(0x18011)	/* Lane RX VMA Performance Metric Register #1 */
#define PCIE_PHY_DWC_TX_CAPT_CTRL		(0x18012)	/* Lane TX Latch Control */
#define PCIE_PHY_DWC_TX_CFG_0			(0x18015)	/* Lane TX Configuration Register #0 */
#define PCIE_PHY_DWC_TX_CFG_1			(0x18016)	/* Lane TX Configuration Register #1 */
#define PCIE_PHY_DWC_TX_CFG_2			(0x18017)	/* Lane TX Configuration Register #2 */
#define PCIE_PHY_DWC_TX_CFG_3			(0x18018)	/* Lane TX Configuration Register #3 */
#define PCIE_PHY_DWC_TX_PREEMPH_0		(0x18019)	/* Lane TX Pre-Emphasis */
#define PCIE_PHY_DWC_PMA_LOOPBACK_CTRL		(0x1801a)	/* Lane PMA Loopback Control */
#define PCIE_PHY_DWC_LANE_PWR_CTRL		(0x1801b)	/* Lane Power Control */
#define PCIE_PHY_DWC_TERM_CTRL			(0x1801c)	/* Lane Termination Control */
#define PCIE_PHY_DWC_RX_MISC_STATUS		(0x18025)	/* RX Miscellaneous Status */
#define PCIE_PHY_DWC_SDS_PIN_MON_0		(0x18026)	/* SerDes Pin Monitor 0 */
#define PCIE_PHY_DWC_SDS_PIN_MON_1		(0x18027)	/* SerDes Pin Monitor 1 */
#define PCIE_PHY_DWC_SDS_PIN_MON_2		(0x18028)	/* SerDes Pin Monitor 2 */
#define PCIE_PHY_DWC_RX_PWR_MON_0		(0x18029)	/* RX Power State Machine Monitor 0 */
#define PCIE_PHY_DWC_RX_PWR_MON_1		(0x1802a)	/* RX Power State Machine Monitor 1 */
#define PCIE_PHY_DWC_RX_PWR_MON_2		(0x1802b)	/* RX Power State Machine Monitor 2 */
#define PCIE_PHY_DWC_TX_PWR_MON_0		(0x1802c)	/* TX Power State Machine Monitor 0 */
#define PCIE_PHY_DWC_TX_PWR_MON_1		(0x1802d)	/* TX Power State Machine Monitor 1 */
#define PCIE_PHY_DWC_TX_PWR_MON_2		(0x1802e)	/* TX Power State Machine Monitor 2 */
#define PCIE_PHY_DWC_RX_VMA_CTRL		(0x18040)	/* Lane RX VMA Control */
#define PCIE_PHY_DWC_RX_CDR_MISC_CTRL_0		(0x18041)	/* Lane RX CDR Miscellaneous Control Register #0 */
#define PCIE_PHY_DWC_RX_CDR_MISC_CTRL_1		(0x18042)	/* Lane RX CDR Miscellaneous Control Register #1 */
#define PCIE_PHY_DWC_RX_PWR_CTRL		(0x18043)	/* Lane RX Power Control */
#define PCIE_PHY_DWC_RX_OS_MVALBBD_0		(0x18045)	/* Lane RX Offset Calibration Manual Control Register #0 */
#define PCIE_PHY_DWC_RX_OS_MVALBBD_1		(0x18046)	/* Lane RX Offset Calibration Manual Control Register #1 */
#define PCIE_PHY_DWC_RX_OS_MVALBBD_2		(0x18047)	/* Lane RX Offset Calibration Manual Control Register #2 */
#define PCIE_PHY_DWC_RX_AEQ_VALBBD_0		(0x18048)	/* Lane RX Adaptive Equalizer Control Register #0 */
#define PCIE_PHY_DWC_RX_AEQ_VALBBD_1		(0x18049)	/* Lane RX Adaptive Equalizer Control Register #1 */
#define PCIE_PHY_DWC_RX_AEQ_VALBBD_2		(0x1804a)	/* Lane RX Adaptive Equalizer Control Register #2 */
#define PCIE_PHY_DWC_RX_MISC_OVRRD		(0x1804b)	/* Lane RX Miscellaneous Override Controls */
#define PCIE_PHY_DWC_RX_OVRRD_PHASE_ACCUM_ADJ	(0x1804c)	/* Lane RX Phase Accumulator Adjust Override */
#define PCIE_PHY_DWC_RX_AEQ_OUT_0		(0x18050)	/* Lane RX Adaptive Equalizer Status Register #0 */
#define PCIE_PHY_DWC_RX_AEQ_OUT_1		(0x18051)	/* Lane RX Adaptive Equalizer Status Register #1 */
#define PCIE_PHY_DWC_RX_AEQ_OUT_2		(0x18052)	/* Lane RX Adaptive Equalizer Status Register #2 */
#define PCIE_PHY_DWC_RX_OS_OUT_0		(0x18053)	/* Lane RX Offset Calibration Status Register #0 */
#define PCIE_PHY_DWC_RX_OS_OUT_1		(0x18054)	/* Lane RX Offset Calibration Status Register #1 */
#define PCIE_PHY_DWC_RX_OS_OUT_2		(0x18055)	/* Lane RX Offset Calibration Status Register #2 */
#define PCIE_PHY_DWC_RX_OS_OUT_3		(0x18056)	/* Lane RX Offset Calibration Status Register #3 */
#define PCIE_PHY_DWC_RX_VMA_STATUS_0		(0x18057)	/* Lane RX CDR Status Register #0 */
#define PCIE_PHY_DWC_RX_VMA_STATUS_1		(0x18058)	/* Lane RX CDR Status Register #1 */
#define PCIE_PHY_DWC_RX_CDR_STATUS_0		(0x18059)	/* Lane RX CDR Status Register #0 */
#define PCIE_PHY_DWC_RX_CDR_STATUS_1		(0x1805a)	/* Lane RX CDR Status Register #1 */
#define PCIE_PHY_DWC_RX_CDR_STATUS_2		(0x1805b)	/* Lane RX CDR Status Register #2 */
#define PCIE_PHY_DWC_PCS_MISC_CFG_0		(0x38000)	/* Lane Miscellaneous Configuration Register #0 */
#define PCIE_PHY_DWC_PCS_MISC_CFG_1		(0x38001)	/* Lane Raw PCS Miscellaneous Configuration Register #1 */
#define PCIE_PHY_DWC_PCS_LBERT_PAT_CFG		(0x38003)	/* LBERT Pattern Configuration */
#define PCIE_PHY_DWC_PCS_LBERT_CFG		(0x38004)	/* LBERT Configuration */
#define PCIE_PHY_DWC_PCS_LBERT_ECNT		(0x38005)	/* LBERT Error Counter */
#define PCIE_PHY_DWC_PCS_RESET_0		(0x38006)	/* Lane Raw PCS Reset Register #0 */
#define PCIE_PHY_DWC_PCS_RESET_1		(0x38007)	/* Lane Raw PCS Reset Register #1 */
#define PCIE_PHY_DWC_PCS_RESET_2		(0x38008)	/* Lane Raw PCS Reset Register #2 */
#define PCIE_PHY_DWC_PCS_RESET_3		(0x38009)	/* Lane Raw PCS Reset Register #3 */
#define PCIE_PHY_DWC_PCS_CTLIFC_CTRL_0		(0x3800c)	/* Lane Raw PCS Control Interface Configuration Register #0 */
#define PCIE_PHY_DWC_PCS_CTLIFC_CTRL_1		(0x3800d)	/* Lane Raw PCS Control Interface Configuration Register #1 */
#define PCIE_PHY_DWC_PCS_CTLIFC_CTRL_2		(0x3800e)	/* Lane Raw PCS Control Interface Configuration Register #2 */
#define PCIE_PHY_DWC_PCS_MACIFC_MON_0		(0x38021)	/* MAC to Raw PCS Interface Monitor Register #0 */
#define PCIE_PHY_DWC_PCS_MACIFC_MON_2		(0x38023)	/* MAC to Raw PCS Interface Monitor Register #1 */

/* DWC_GLBL_PLL_MONITOR */
#define SDS_PCS_CLOCK_READY			(1 << 6)	/* Clock status signal. */

/* DWC_GLBL_PLL_CFG_0 */
#define PCS_SDS_PLL_FTHRESH_SHIFT		6
#define PCS_SDS_PLL_FTHRESH_MASK		0xC0		/* PLL frequency comparison threshold */

/* DWC_GLBL_TERM_CFG */
#define FAST_TERM_CAL				(1 << 8)	/* Enable fast termination calibration. */

/* DWC_RX_LOOP_CTRL */
#define FAST_OFST_CNCL				(1 << 10)	/* Enable fast offset cancellation. */
#define FAST_DLL_LOCK				(1 << 11)	/* Enable fast DLL lock. */

/* Enable PCIe 3.0 PHY */
#define EN_PCIE3                                (1 << 10)

/* DWC_TX_CFG_0 */
#define FAST_TRISTATE_MODE			(1 << 1)	/* Enable fast Tristate power up. */
#define FAST_RDET_MODE				(1 << 2)	/* Enable fast RX Detection */
#define FAST_CM_MODE				(1 << 8)	/* Enable fast common-mode charge up. */

/* Macros to read/write PCIe registers. */
#define READ_PCIE_REG(r)	readl((const volatile void *)(PCIE_CFG_BASE + (r)))
#define WRITE_PCIE_REG(r, v)	writel((v), (volatile void *)(PCIE_CFG_BASE + (r)))


/* PMU registers */

#define BK_PMU_LOCK_BIT			(1 << 31)
#define BK_PMU_EN_BIT			(1 << 0)
#define BK_PMU_RST_BIT			(1 << 1)
#define BK_PMU_INIT_BIT			(1 << 2)

/* BK_PMU_AXI_PCIE_M_CTL */
#define PMU_AXI_PCIE_M_CTL_EN		(1 << 0)	/* Enable AXI PCIe Master clock. */
#define PMU_AXI_PCIE_M_CTL_RST		(1 << 1)	/* Software AXI PCIe Master clock domain reset. */

/* BK_PMU_AXI_PCIE_S_CTL */
#define PMU_AXI_PCIE_S_CTL_EN		(1 << 0)	/* Enable AXI PCIe Slave clock. */
#define PMU_AXI_PCIE_S_CTL_RST		(1 << 1)	/* Software AXI PCIe Slave clock domain reset. */

/* BK_PMU_PCIE_RSTC */
#define PMU_PCIE_RSTC_PHY_RESET		(1 << 0)	/* PCIe PHY phy_rts_n reset control bit. */
#define PMU_PCIE_RSTC_PIPE_RESET	(1 << 4)	/* PCIe PHY PCS pipe_reset_n reset control bit. */
#define PMU_PCIE_RSTC_CORE_RST		(1 << 8)	/* PCIe core core_rst_n reset control bit. */
#define PMU_PCIE_RSTC_PWR_RST		(1 << 9)	/* PCIe core pwr_rst_n reset control bit. */
#define PMU_PCIE_RSTC_STICKY_RST	(1 << 10)	/* PCIe core sticky_rst_n reset control bit. */
#define PMU_PCIE_RSTC_NONSTICKY_RST	(1 << 11)	/* PCIe core nonsticky_rst_n reset control bit. */
#define PMU_PCIE_RSTC_HOT_RESET		(1 << 12)	/* Hot Reset control bit. */
#define PMU_PCIE_RSTC_REQ_RESET		(1 << 13)	/* PCIe core link_req_rst_not ready for reset signal status bit */
#define PMU_PCIE_RSTC_SMLH_REQ_RST	(1 << 14)
#define PMU_PCIE_RSTC_REQ_PHY_RST	(1 << 16)
#define PMU_PCIE_RSTC_REQ_CORE_RST	(1 << 24)
#define PMU_PCIE_RSTC_REQ_STICKY_RST	(1 << 26)
#define PMU_PCIE_RSTC_REQ_NON_STICKY_RST (1 << 27)
#define PMU_PCIE_RSTC_BRIDGE_FLUSH	(1 << 19)	/* PCIe AXI bridge bridge_flush_not signal status bit. */

/* BK_PMU_PCIE_GENC */
#define PMU_PCIE_GENC_LTSSM_ENABLE	(1 << 1)	/* LTSSM enable bit. */
#define PMU_PCIE_GENC_DBI2_MODE		(1 << 2)	/* PCIe core registers access mode bit: DBI(=0) / DBI2(=1) */
#define PMU_PCIE_GENC_MGMT_ENABLE	(1 << 3)	/* PCIe PHY management interface enable bit. */

/* BK_PMU_PCIE_PMSC */
#define PMU_PCIE_PMSC_LTSSM_STATE_SHIFT	(0)		/* LTSSM state (smlh_ltssm_state[5:0] signal) */
#define PMU_PCIE_PMSC_LTSSM_STATE_MASK	(0x3F)
#define LTSSM_L0			0x11
#define PMU_PCIE_PMSC_SMLH_LINKUP	(1 << 6)	/* Physical level (PL) state bit (smlh_link_up signal) */
#define PMU_PCIE_PMSC_RDLH_LINKUP	(1 << 7)	/* Channel level (DLL) state bit (rdlh_link_up signal) */

/* Register map */

#define BK_COREPLL_CTL_OFFSET		0x000
#define BK_PCIEPLL_CTL_OFFSET		0x018
#define BK_PCIEPLL_CTL1_OFFSET		0x01C
#define BK_AXI_PCIE_M_CTL_OFFSET	0x048
#define BK_AXI_PCIE_S_CTL_OFFSET	0x04C
#define BK_PCIE_REF_CTL_OFFSET		0x05C
#define BK_PCIE_CLKC_OFFSET		0x140
#define BK_PCIE_RSTC_OFFSET		0x144
#define BK_PCIE_PMSC_OFFSET		0x148
#define BK_PCIE_GENC_OFFSET		0x14C

#define BK_PMU_COREPLL_CTL	(PMU_BASE + BK_COREPLL_CTL_OFFSET)
#define BK_PMU_PCIEPLL_CTL	(PMU_BASE + BK_PCIEPLL_CTL_OFFSET)
#define BK_PMU_PCIEPLL_CTL1	(PMU_BASE + BK_PCIEPLL_CTL1_OFFSET)
#define BK_PMU_AXI_PCIE_M_CTL	(PMU_BASE + BK_AXI_PCIE_M_CTL_OFFSET)
#define BK_PMU_AXI_PCIE_S_CTL	(PMU_BASE + BK_AXI_PCIE_S_CTL_OFFSET)
#define BK_PMU_PCIE_REF_CTL	(PMU_BASE + BK_PCIE_REF_CTL_OFFSET)
#define BK_PMU_PCIE_CLKC	(PMU_BASE + BK_PCIE_CLKC_OFFSET)
#define BK_PMU_PCIE_RSTC	(PMU_BASE + BK_PCIE_RSTC_OFFSET)
#define BK_PMU_PCIE_PMSC	(PMU_BASE + BK_PCIE_PMSC_OFFSET)
#define BK_PMU_PCIE_GENC	(PMU_BASE + BK_PCIE_GENC_OFFSET)

/* Macros to read/write PMU registers. */
#define READ_PMU_REG(r)		readl((const volatile void *)(r))
#define WRITE_PMU_REG(r, v)	writel(v, (volatile void *)(r))

void dw_set_iatu_region(int dir, int index, int base_addr, int limit_addr, int target_addr, int tlp_type);
irqreturn_t dw_msi_interrupt(int id, void *dev_id);
int dw_msi_init(void);

#endif /* __PCI_BAIKAL_H__ */
