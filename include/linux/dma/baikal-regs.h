/*
 * Driver for the Baikal SoC PCIe DMA Controller
 *
 * Copyright (C) 2005-2007 Atmel Corporation
 * Copyright (C) 2010-2011 ST Microelectronics
 * Copyright (C) 2017 Baikal Electronics JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_DMA_BAIKAL_REGS_H
#define _LINUX_DMA_BAIKAL_REGS_H

#include <linux/interrupt.h>
#include <linux/dmaengine.h>

#define BAIKAL_EDMA_RD_CHANNELS	4
#define BAIKAL_EDMA_WR_CHANNELS	4
#define BAIKAL_EDMA_TOTAL_CHANNELS BAIKAL_EDMA_RD_CHANNELS + BAIKAL_EDMA_WR_CHANNELS

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

/*******************************************************************************
 *
 *  MACRO DEFINITIONS
 *
 ******************************************************************************/
/**
 * Maximum block size for a DMA transfer, in unit of byte.
 */
#define MAX_BLOCK_SIZE      (4 * 1024 * 1024)   // 4G bytes
/**
 * Minimum block size for a DMA transfer, in unit of byte.
 */
#define MIN_BLOCK_SIZE      1   // 1 byte
/**
 * Maximum element number of DMA linked list descriptor.
 */
#define MAX_LLELEMENT_NUM   20
/**
 * Minimum element number of DMA linked list descriptor.
 */
#define MIN_LLELEMENT_NUM   5
/**
 * Maximum length of linked list in unit of dword
 */
//#define MAX_LL_LEN          (MAX_LLELEMENT_NUM * 6)     // dwords
/**
 * Maximum weight value a channel can be specified.
 */
#define MAX_WEIGHT          31
/**
 * Minimum weight value a channel can be specified.
 */
#define MIN_WEIGHT          0

#define DMA_REG_RETRIES	    1000000

//#define MAX_MSG_SIZE	    1024
//#define DPRINTF
//#define INT_SYNC_RUN(f, a)	(f)((a))

/******************************************************************************
 *
 *  DATA STRUCTURE DEFINITIONS
 *
 ******************************************************************************/

/**
 * Physcial address.
 */
typedef union {
	struct {
		uint32_t low_part;
		uint32_t high_part;
	};
	uint64_t quad_part;
} phyaddr;

/**
 * The direction of DMA transfer.
 */
typedef enum {
	DMA_WRITE = 0,  /**< Write */
	DMA_READ        /**< Read */
} baikal_dma_dirc;

/**
 * DMA return code.
 */
typedef enum {
	DMA_SUCCESS = 0,
	DMA_CHAN_BUSY,         /**< Channel is busy to respond */
	DMA_INVALID_CHAN,      /**< Invalid channel number */
	DMA_INVALID_BLOCK_SIZE,/**< Invalid block size */
	DMA_INVALID_LL_LEN,    /**< Invalid linked list length */
	DMA_INVALID_WM,        /**< Invalid watermark element index */
	DMA_INVALID_WEIGHT,    /**< Invalid weight value */
	DMA_NULL_ARG,          /**< Null arguments are received */
	DMA_UNKNOWN_FAILURE,   /**< Unknown failure */
	DMA_NO_QUEUING_JOBS,   /**< No queuing jobs found */
	DMA_NO_FEED,           /**< No new data are feed to linked list */
	DMA_NO_LL,             /**< No linked list are created */
	DMA_FATAL,             /**< Driver fatal error */

	DMA_ERR_CODE_NUM       /**< HAL return code numbers */

} baikal_dmareturn_t;

/**
 * DMA transfer mode.
 */
typedef enum {
	DMA_SINGLE_BLOCK = 0,/**< Single block mode */
	DMA_MULTI_BLOCK      /**< Multi block mode */
} baikal_dma_xfer_mode;

/**
 * DMA interrupt type.
 */
typedef enum {
	DMA_INT_DONE = 0,   /**< DONE interrupt */
	DMA_INT_ABORT       /**< ABORT interrupt */
} baikal_dma_irq_type;

#define DMA_INT_DONE_MASK	(1 << 0)
#define DMA_INT_ABORT_MASK	(1 << 16)

/**
 * DMA Channel status.
 */
typedef enum {
	CHAN_UNKNOWN = 0,   /**< Unknown */
	CHAN_RUNNING,       /**< Channel is running */
	CHAN_HALTED,        /**< Channel is halted */
	CHAN_STOPPED,       /**< Channel is stopped */
	CHAN_QUEUING	    /**< Queuing. Not a real HW status */
} baikal_dma_chan_status;

/**
 * DMA hardware error types.
 */
typedef enum {
	DMA_ERR_NONE,       /**< No DMA error found */
	DMA_ERR_WR,         /**< The DMA Write Channel has received an error
						 *   response from the AHB/AXI bus (or RTRGT1 interface
						 *   when the AHB/AXI Bridge is not used) while reading
						 *   data from it. It's fatal error. */
	DMA_ERR_RD,         /**< The DMA Read Channel has received an error response
						 *   from the AHB/AXI bus (or RTRGT1 interface when the
						 *   AHB/AXI Bridge is not used) while writing data to
						 *   it. It's fatal error.*/
	DMA_ERR_FETCH_LL,   /**< The DMA Write/Read Channel has received an error
						 *   response from the AHB/AXI bus (or RTRGT1 interface
						 *   when the AHB/AXI Bridge is not used) while reading
						 *   a Linked List Element from local memory. It's fatal
						 *   error. */
	DMA_ERR_UNSUPPORTED_RQST,
	/**< The DMA Read Channel has received a PCIe
	 *   Unsupported Request CPL status from the remote
	 *   device in response to the MRd Request.*/
	DMA_ERR_COMPLETER_ABORT,
	/**< The DMA Read Channel has received a PCIe Completer
	 *  Abort CPL status from the remote device in response
	 *  to the MRd Request. Non-fatal error.
	 */
	DMA_ERR_CPL_TIME_OUT,
	/**< The DMA Read Channel has timed-out while waiting
	 * for the remote device to respond to the MRd Request,
	 * or a malformed CplD has been received. Non-fatal
	 * error. */
	DMA_ERR_DATA_POISONING,
	/**< The DMA Read Channel has detected data poisoning
	 * in the CPL from the remote device in response to the
	 * MRd Request. Non-fatal error. */
} baikal_dma_err;

/*******************************************************************************
 *
 * DEVICE REGISTERS
 * 
 ******************************************************************************/
/**
 * The Enable Register for read and write.
 */
typedef union {
	struct {
		//LSB
		uint32_t      enb         :1;  // 0
		uint32_t      reserved0   :31; // 1:31
		//MSB
	}; 
	uint32_t dword;
} baikal_dma_enb_reg;

/**
 * The DMA Control Register.
 */
typedef union {
	struct {
		//LSB
		uint32_t      write_chans     :3;  // 0:2
		uint32_t      reserved0       :13; // 3:15
		uint32_t      read_chans      :3;  // 16:18
		uint32_t      reserved1       :13; // 19:31
		//MSB
	};
	uint32_t dword;
} baikal_dma_ctrl_reg;

/**
 * The Doorbell Register for read and write.
 */
typedef union {
	struct {
		//LSB
		uint32_t      channel     :3;  // 0
		uint32_t      reserved0   :28; // 3:30
		uint32_t      stop        :1;  // 31
		//MSB
	}; 
	uint32_t dword;
} baikal_dma_doorbell_reg;

/**
 * The Interrupt Status Register for read and write.
 */
typedef union {
	struct {
		//LSB
		uint32_t      done_status     :8;
		uint32_t      reserved0       :8;
		uint32_t      abort_status    :8;
		uint32_t      reserved1       :8;
		//MSB
	};
	uint32_t dword;
} baikal_dma_irq_reg;

/**
 * The Interrupt Clear Register for read and write.
 */
typedef union {
	struct {
		//LSB
		uint32_t      done_clear     :8;
		uint32_t      reserved0      :8;
		uint32_t      abort_clear    :8;
		uint32_t      reserved1      :8;
		//MSB
	};
	uint32_t dword;
} baikal_dma_irq_clear_reg;

/**
 * The Error Status Register for read and write.
 */
typedef union {
	struct {
		//LSB
		uint32_t      brdg_err        :8;
		uint32_t      reserved0       :8;
		uint32_t      ll_err          :8;
		uint32_t      reserved1       :8;
		//MSB
	};
	uint32_t dword;
} baikal_dma_write_err, baikal_dma_read_err_lo;

/**
 * Extra Error Status Register for read.
 */
typedef union {
	struct {
		//LSB
		uint32_t      ur_err       :8;
		uint32_t      ca_err       :8;
		uint32_t      to_err       :8;
		uint32_t      ep_err       :8;
		//MSB
	};
	uint32_t dword;
} baikal_dma_read_err_hi;

/**
 * The Channel Control Register for read and write.
 */
typedef union { 
	struct {
		//LSB
		uint32_t      CB          :1;    // 0
		uint32_t      TCB         :1;    // 1
		uint32_t      LLP         :1;    // 2
		uint32_t      LIE         :1;    // 3
		uint32_t      RIE         :1;    // 4
		uint32_t      CS          :2;    // 5:6
		uint32_t      reserved1   :1;    // 7
		uint32_t      CCS         :1;    // 8
		uint32_t      LLEN        :1;    // 9
		uint32_t      b_64S       :1;    // 10
		uint32_t      b_64D       :1;    // 11
		uint32_t      PF          :5;    // 12:16
		uint32_t      reserved2   :7;    // 17:23
		uint32_t      SN          :1;    // 24
		uint32_t      RO          :1;    // 25
		uint32_t      TD          :1;    // 26
		uint32_t      TC          :3;    // 27:29
		uint32_t      AT          :2;    // 30:31
		//MSB
	};
	uint32_t dword;
} baikal_dma_chan_ctrl_lo;

/**
 * The Channel Control Register high part for read and write.
 */
typedef union {
	struct {
		//LSB
		uint32_t      vf_enb          :1;     // 0
		uint32_t      vf_unc          :8;     // 1-8
		uint32_t      reserved0       :23;    // 9-31
		//MSB
	};
	uint32_t dword;
} baikal_dma_chan_ctrl_hi;

/**
 * The Channel Context Registers for read and write.
 */
typedef struct {
	baikal_dma_chan_ctrl_lo    ctrl_lo;     // 0x00
	baikal_dma_chan_ctrl_hi    ctrl_hi;     // 0x04
	uint32_t               xfer_size;   // 0x08
	uint32_t               sar_ptr_lo;   // 0x0C
	uint32_t               sar_ptr_hi;   // 0x10
	uint32_t               dar_ptr_lo;   // 0x14
	uint32_t               dar_ptr_hi;   // 0x18
	uint32_t               el_ptr_lo;    // 0x1C
	uint32_t               el_ptr_hi;    // 0x20
} baikal_dma_ctx_regs;

/**
 * The Context Register selector for read and write.
 */
typedef union {
	struct {
		//LSB
		uint32_t      viw_pnt         :3;     // 0:2
		uint32_t      reserved0       :28;    // 3:30
		uint32_t      sel             :1;     // 31
		//MSB
	};
	uint32_t  dword;
} baikal_dma_ctx_sel_reg;

/**
 * The Channel Weight Register.
 */
typedef union {
	struct {
		//LSB
		uint32_t      weight0     :5;     // 0:4
		uint32_t      weight1     :5;     // 5:9
		uint32_t      weight2     :5;     // 10:14
		uint32_t      weight3     :5;     // 15:19
		uint32_t      reserved    :12;    // 20:31
		//MSB
	};
	uint32_t dword;
} baikal_dma_weight_reg;

/**
 * The Linked List Interrupt Error Enable Register for write and read
 */
typedef union {
	struct {
		//LSB
		uint32_t      reie        :8;     // 0:7
		uint32_t      rsvd0       :8;     // 8:15
		uint32_t      leie        :8;     // 16:23
		uint32_t      reserved1   :8;     // 24:31
		//MSB
	};
	uint32_t dword;
} baikal_dma_ll_err_enb_reg;

/**
 * DMA Registers starts from 0x970 to 0xB2C.
 */
typedef struct baikal_dma_regs {
	uint32_t               dummy[604];             // 0x000 - 0x96F
	uint32_t               reserved0[2];           // 0x970 - 0X974
	baikal_dma_ctrl_reg        dma_ctrl;               // 0x978
	baikal_dma_enb_reg         write_enb;              // 0x97C
	baikal_dma_doorbell_reg    write_db;               // 0x980
	uint32_t               write_ctrl;             // 0x984
	baikal_dma_weight_reg      write_weight_lo;        // 0x988
	baikal_dma_weight_reg      write_weight_hi;        // 0x98C
	uint32_t               reserved1[3];           // 0x990 - 0x998
	baikal_dma_enb_reg         read_enb;               // 0x99C
	baikal_dma_doorbell_reg    read_db;                // 0x9A0
	uint32_t               read_ctrl;              // 0x9A4
	baikal_dma_weight_reg      read_weight_lo;         // 0x9A8
	baikal_dma_weight_reg      read_weight_hi;         // 0x9AC
	uint32_t               reserved2[3];           // 0x9B0 - 0x9B8
	baikal_dma_irq_reg         write_irq_status;       // 0x9BC
	uint32_t               reserved3;              // 0x9C0 
	uint32_t               write_irq_mask;         // 0x9C4
	baikal_dma_irq_clear_reg   write_irq_clear;        // 0x9C8
	baikal_dma_write_err       write_err_status;       // 0x9CC
	uint32_t               write_msg_done_lo;      // 0x9D0
	uint32_t               write_msg_done_hi;      // 0x9D4
	uint32_t               write_msg_abt_lo;       // 0x9D8
	uint32_t               write_msg_abt_hi;       // 0x9DC
	int16_t                write_msg_data[8];      // 0x9E0 - 0x9EC
	uint32_t               reserved4[4];           // 0x9F0 - 0x9FC
	baikal_dma_ll_err_enb_reg  write_irq_ll_err_enb;   // 0xA00
	uint32_t               reserved5[3];           // 0xA04 - 0xA0C
	baikal_dma_irq_reg         read_irq_status;        // 0xA10
	uint32_t               reserved6;              // 0xA14
	uint32_t               read_irq_mask;          // 0xA18
	baikal_dma_irq_clear_reg   read_irq_clear;         // 0xA1C
	uint32_t               reserved7;              // 0xA20
	baikal_dma_read_err_lo     read_err_status_lo;     // 0xA24
	baikal_dma_read_err_hi     read_err_status_hi;     // 0xA28
	uint32_t               reserved8[2];          // 0xA2C - 0xA30
	baikal_dma_ll_err_enb_reg  read_irq_ll_err_enb;    // 0xA34
	uint32_t               reserved9;              // 0xA38
	uint32_t               read_msg_done_lo;       // 0xA3C
	uint32_t               read_msg_done_hi;       // 0xA40
	uint32_t               read_msg_abt_lo;        // 0xA44
	uint32_t               read_msg_abt_hi;        // 0xA48
	int16_t                read_msg_data[8];       // 0xA4C - 0xA58
	uint32_t               reserved10[4];         // 0xA5C - 0xA68
	baikal_dma_ctx_sel_reg     ctx_sel;                // 0xA6C 
	baikal_dma_ctx_regs        ctx_regs;               // 0xA70 - 0xA90 
} baikal_dma_regs;

/*
 * Big endian I/O access when reading and writing to the DMA controller
 * registers.  This is needed on some platforms, like the Atmel AVR32
 * architecture.
 */

#define dma_readl_native readl
#define dma_writel_native writel

/* To access the registers in early stage of probe */
#define dma_read_byaddr(addr, name) \
	dma_readl_native((addr) + offsetof(struct baikal_dma_regs, name))

/* bursts size */
enum baikal_dma_msize {
	BE_DMA_MSIZE_1,
	BE_DMA_MSIZE_4,
	BE_DMA_MSIZE_8,
	BE_DMA_MSIZE_16,
	BE_DMA_MSIZE_32,
	BE_DMA_MSIZE_64,
	BE_DMA_MSIZE_128,
	BE_DMA_MSIZE_256,
};

struct baikal_dma_chan {
	struct dma_chan	        chan;
	void __iomem	        *ch_regs;
	uint32_t		        mask;
	u8				        priority;
	baikal_dma_dirc			dirc;
	bool			        paused;
	bool			        initialized;

	uint32_t                id;

	int						irq;		/* IRQ number from dtb  */
	baikal_dma_irq_reg		irq_status;	/* snapshot of last IRQ status (a copy of register) */
	bool					has_err;	/* error status */
	struct tasklet_struct   tasklet;	/* tasklet to serve IRQ */
	void					(*callback) (void *);

	spinlock_t		        lock;

	uint32_t				time;
	uint32_t				irq_count;

	/* these other elements are all protected by lock */
	unsigned long		    flags;
	struct list_head	    active_list;
	struct list_head	    queue;
	struct list_head		free_list;
	u32			            residue;

	uint32_t                 *llp_addr;
	uint32_t                 *llv_addr;

	//unsigned int		    descs_allocated;

	/* hardware configuration */
	unsigned int		    block_size;
	bool			        nollp;

	// Initialized by user request
	uint64_t                feed_size;          /**< Total size of DMA transfer. */
	uint8_t                 ll_elements;        /**< Number of linked list elements */
	uint8_t                 water_mark;         /**< Index of watermark element */
	uint8_t                 weight;             /**< The weight of this channel */
	uint32_t                recycle;            /**< How many times to recycle to linked list */
	//PRODUCER_FUNC           producer_func;      /**< Linked list producer function for this channel */
	void*                   producer_context;   /**< The context used by ProducerFunc */
	// Updated by HW
	uint32_t                xfer_size;          /**< The data size have not been xferred in one block.*/ 
	phyaddr                 element_pos;        /**< The element the DMA is currently transferring */
	baikal_dma_chan_status      status;             /**< Channel status, like running, stopped etc. */
	baikal_dma_err              error;              /**< DMA hardware error if any */
	// Updated by driver
	uint8_t                 pcs;                /**< PCS flag. Used in multi block transfer mode. */
	uint8_t                 element_idx;        /**< Index of data element the DMA is currently working
												 *   on during linked list update. It's for multi block
												 *   transfer use. */
	uint32_t                src_pos;            /**< The source address the DMA is working on. */
	uint32_t                dst_pos;            /**< The destination address the DMA is working on. */
	uint32_t                done_intr_cnt;      /**< DONE interrupt count */
	uint32_t                abort_intr_cnt;     /**< ABORT interrupt count */

	/* configuration passed via .device_config */
	struct dma_slave_config dma_sconfig;

};

/**
 * The Context Registers for read and write.
 */
typedef struct baikal_dma_chan_regs {
	baikal_dma_chan_ctrl_lo    ctrl_lo;     // 0x00
	baikal_dma_chan_ctrl_hi    ctrl_hi;     // 0x04
	uint32_t           xfer_size;   // 0x08
	uint32_t           sar_ptr_lo;   // 0x0C
	uint32_t           sar_ptr_hi;   // 0x10
	uint32_t           dar_ptr_lo;   // 0x14
	uint32_t           dar_ptr_hi;   // 0x18
	uint32_t           el_ptr_lo;    // 0x1C
	uint32_t           el_ptr_hi;    // 0x20
} baikal_dma_chan_regs;

static inline struct baikal_dma_chan *to_baikal_dma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct baikal_dma_chan, chan);
}

struct baikal_dma {
	struct dma_device		dma;
	void __iomem			*regs;
	struct dma_pool			*desc_pool;
	//struct tasklet_struct	tasklet;

	spinlock_t				lock;

	/* channels */
	struct baikal_dma_chan	*chan;
	uint32_t	all_chan_mask; /* actually 8 bit width */
	uint32_t	in_use;			/* actually bool */ 

	/* hardware configuration */

	phyaddr             ll_phy_addr[16];    /**< */
	uint32_t*           ll_vir_addr[16];    /**< */
	//CHAN_INFO           wr_chan_info[8];    /**< The information for all write channels */
	//CHAN_INFO           rd_chan_info[8];    /**< The information for all read channels */
	uint32_t            msi_addr;           /**< MSI address */
	uint16_t			*msi_data;          /**< MSI data */
	void*               intr_obj;           /**< The interrupt object, OS-specific */
	//xfer_done_func_clbk xfer_done_func;
	/**< Callback will be called when xfer is done. */
	void*               xfer_done_context;  /**< Context for XferDoneFunc callback */
	// Following members are read from DMA register
	uint32_t             write_chans;          /**< Enabled write channels, actually 3 bit width */
	uint32_t             read_chans;          /**< Enabled read channels, actually 3 bit width */
};

	static inline baikal_dma_ctx_regs __iomem *
__baikal_dma_chan_regs(struct baikal_dma* dmac, uint32_t chan)
{
	return &((baikal_dma_regs *) dmac->regs)->ctx_regs;
}

#define channel_writel(dmac, chan, name, val)                                          \
	dma_writel_native((val), &(((baikal_dma_ctx_regs *) __baikal_dma_chan_regs(dmac, chan))->name));       \
for (i = 0; i < DMA_REG_RETRIES; i++) {                                            \
	new_val = dma_readl_native(&(((baikal_dma_ctx_regs *) __baikal_dma_chan_regs(dmac, chan))->name)); \
	if (new_val == (val)) {                                                        \
		break;                                                                     \
	}                                                                              \
}                                                                                  \

#define channel_readl(dmac, chan, name)                                    \
	dma_readl_native(&(((baikal_dma_ctx_regs *) __baikal_dma_chan_regs(dmac, chan))->name))

	static inline struct baikal_dma_regs __iomem *
__baikal_dma_regs(struct baikal_dma *dmac)
{
	return dmac->regs;
}

#define dma_readl(dmac, name) \
	dma_readl_native(&(__baikal_dma_regs(dmac)->name))

#define dma_writel(dmac, name, val)                                                    \
	dma_writel_native((val), &(__baikal_dma_regs(dmac)->name));                        \
for (i = 0; i < DMA_REG_RETRIES; i++) {                                            \
	new_val = dma_readl_native(&(__baikal_dma_regs(dmac)->name));                  \
	if (new_val == (val)) {                                                        \
		break;                                                                     \
	}                                                                              \
}                                                                                  \

static inline struct baikal_dma *to_baikal_dma(struct dma_device *ddev)
{
	return container_of(ddev, struct baikal_dma, dma);
}

/* LLI == Linked List Item; a.k.a. DMA block descriptor */
struct baikal_dma_lli {
	/* values that are not changed by hardware */
	uint32_t    ctrl;
	uint32_t    xfer_size;
	phyaddr		sar;
	phyaddr		dar;
};

struct baikal_dma_desc {
	/* FIRST values the hardware uses */
	struct baikal_dma_lli			lli;

	/* THEN values for driver housekeeping */
	struct list_head				desc_node;
	struct list_head				tx_list;
	struct dma_async_tx_descriptor	txd;
	size_t							len;
	size_t							total_len; 
};

#define to_baikal_dma_desc(h)	list_entry(h, struct baikal_dma_desc, desc_node)

#endif /* _DMA_BAIKAL_H */
