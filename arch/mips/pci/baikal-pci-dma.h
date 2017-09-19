#ifndef __BAIKAL_PCI_DMA__
#define __BAIKAL_PCI_DMA__

/*******************************************************************************
 *
 *  HEADER FILE INCLUDES
 *
 ******************************************************************************/
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/platform_device.h>

#define TRUE	1
#define FALSE	0

#define PEDD_READ_REG(r)	readl((volatile void *)(r))
#define	PEDD_WRITE_REG(r, v)	writel(v, (volatile void *)(r))

#define _ehb_()                                 \
  do {                                          \
    __asm__ __volatile__(                       \
      "sll    $0, $0, 3 \n"                     \
      );                                        \
  } while (0)


//#define DTRACE		printk
#define DTRACE(...)		;

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
    uint32_t LowPart;
    uint32_t HighPart;
  };
  uint64_t QuadPart;
} PHYADDR;

/**
 * The direction of DMA transfer.
 */
typedef enum {
  DMA_WRITE = 0,  /**< Write */
  DMA_READ        /**< Read */
} DMA_DIRC;

/**
 * DMA transfer mode.
 */
typedef enum {
  DMA_SINGLE_BLOCK = 0,/**< Single block mode */
  DMA_MULTI_BLOCK      /**< Multi block mode */
} DMA_XFER_MODE;

/**
 * DMA interrupt type.
 */
typedef enum {
  DMA_INT_DONE = 0,   /**< DONE interrupt */
  DMA_INT_ABORT       /**< ABORT interrupt */
} DMA_INT_TYPE;

/**
 * DMA Channel status.
 */
typedef enum {
	CHAN_UNKNOWN = 0,   /**< Unknown */
  CHAN_RUNNING,       /**< Channel is running */
  CHAN_HALTED,        /**< Channel is halted */
  CHAN_STOPPED,       /**< Channel is stopped */
  CHAN_QUEUING	    /**< Queuing. Not a real HW status */
} DMA_CHAN_STATUS;

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
} DMA_ERR;

typedef struct DMA_REGS *PDMA_REGS;




/*******************************************************************************
 *
 *  MACRO DEFINITIONS
 *
 ******************************************************************************/
/**
 * Maximum block size for a DMA transfer, in unit of byte.
 */
#define MAX_BLOCK_SIZE      (1 * 1024 * 1024 * 1024)   // 1G bytes
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
#define MAX_LL_LEN          (MAX_LLELEMENT_NUM * 6)     // dwords
/**
 * Maximum weight value a channel can be specified.
 */
#define MAX_WEIGHT          31
/**
 * Minimum weight value a channel can be specified.
 */
#define MIN_WEIGHT          0


#define MAX_MSG_SIZE	1024
#define DPRINTF
#define INT_SYNC_RUN(f, a)	(f)((a))

/*******************************************************************************
 *
 *  CALLBACK DECLARATIONS
 *
 ******************************************************************************/
/**
 * Linked list producer callback function that the DMA HAL user need to
 * provide when multi block transfer is requested. Whenever the HAL tries
 * to update the linked list element, it will call this function to get next
 * SAR/DAR/XferSize. If the linked list has N data elements and it will be
 * recycled M times, then user must guarantee that this callback function can
 * return meaningful data (N * M) times.
 *
 * @note Because this callback is called during ISR, user must guarantee that
 * it can return as soon as possible.
 *
 * @param[in,out] src   The source physical address.
 * @param[in,out] dst   The destination physical address.
 * @param[in,out] size  The transfer size.
 * @param[in] dirc      The direction of current channel.
 * @param[in] chan      The number of current channel.
 * @param[in] context   The context the user need to know.
 * @return True if new data is provided; False if no more data need to be
 * transferred.
 */
typedef int (*PRODUCER_FUNC)(PHYADDR *src, PHYADDR *dst, uint32_t *size,
                             DMA_DIRC dirc, uint8_t chan, void* context);
/**
 * This is a callback that will be called every time the transfer on specified
 * DMA channel get finished. User can set this callback in DMA_SRC struct to
 * do something they need to do as a post-transfer process.
 *
 * @note Because this callback is called during ISR, user must guarantee that
 * it can return as soon as possible. For heavy tasks, user should schedule DPC
 * to do the job outside ISR.
 *
 * @param[in] dirc      The direction of the channel that transfer is done.
 * @param[in] chan      The number of the channel that transfer is done.
 * @param[in] context   The context the callback need to know.
 */
typedef void (*XFER_DONE_FUNC)(DMA_DIRC dirc, uint8_t chan, void* context);

/*******************************************************************************
 *
 *  DATA STRUCTURE DEFINITIONS
 *
 ******************************************************************************/
/**
 * DMA Channel related information. This data struct is designed for HAL
 * internal use. User should NOT modify the value of any data memeber.
 */
typedef struct {
  // Initialized by user request
  uint64_t FeedSize;    /**< Total size of DMA transfer. */
  uint8_t LLElements;   /**< Number of linked list elements */
  uint8_t WaterMark;    /**< Index of watermark element */
  uint8_t Weight;       /**< The weight of this channel */
  uint32_t Recycle;     /**< How many times to recycle to linked list */
  PRODUCER_FUNC ProducerFunc;
  /**< Linked list producer function for this channel */
  void* ProducerContext;
  /**< The context used by ProducerFunc */
  // Updated by HW
  uint32_t XferSize;    /**< The data size have not been xferred in one block.*/
  PHYADDR ElementPos; /**< The element the DMA is currently transferring */
  DMA_CHAN_STATUS Status;
  /**< Channel status, like running, stopped etc. */
  DMA_ERR Error;      /**< DMA hardware error if any */
  // Updated by driver
  uint8_t PCS;          /**< PCS flag. Used in multi block transfer mode. */
  uint8_t ElementIdx;   /**< Index of data element the DMA is currently working
                         *   on during linked list update. It's for multi block
                         *   transfer use. */
  PHYADDR SrcPos;     /**< The source address the DMA is working on. */
  PHYADDR DstPos;     /**< The destination address the DMA is working on. */
  uint32_t DoneIntrCnt;
  /**< DONE interrupt count */
  uint32_t AbortIntrCnt;
  /**< ABORT interrupt count */
} CHAN_INFO, *PCHAN_INFO;

/**
 * The DMA resource data structure. DMA resourse is the primary data structure
 * that HAL works upon. It includes the information of DMA register map, the
 * linked list address, the number enabled channels, the MSI configuration, and
 * the context data for each channel etc.
 */
typedef struct {
  PDMA_REGS DmaRegs;      /**< The pointer to DMA registers */

  PHYADDR LLPhyAddr[16];  /**< */
  uint32_t* LLVirAddr[16];  /**< */
  CHAN_INFO WrChanInfo[8];/**< The information for all write channels */
  CHAN_INFO RdChanInfo[8];/**< The information for all read channels */
  PHYADDR MsiAddr;        /**< MSI address */
  uint16_t* MsiData;         /**< MSI data */
  void* IntrObj;          /**< The interrupt object, OS-specific */
  XFER_DONE_FUNC XferDoneFunc;
  /**< Callback will be called when xfer is done. */
  void* XferDoneContext;  /**< Context for XferDoneFunc callback */
  // Following members are read from DMA register
  uint8_t WrChans;          /**< Enabled write channels */
  uint8_t RdChans;          /**< Enabled read channels */
} DMA_RSC, *PDMA_RSC;

/**
 * HAL DMA return code.
 * @see HalDmaGetErrInfo
 */
typedef enum {
  DMA_SUCCESS = 0,       /**< HAL operation succeed */
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

} HAL_DMA_RET;

typedef struct {
  PDMA_RSC DmaRsc;
  DMA_DIRC Dirc;
  uint8_t Chan;
  void* Data;
} DMA_SYNCRUN_CONTEXT, *PDMA_SYNCRUN_CONTEXT;


static HAL_DMA_RET SingleBlockRW(PDMA_RSC pdrsc, DMA_DIRC dirc, uint8_t chan,
                                 PHYADDR src, PHYADDR dst, uint32_t size, uint8_t wei, int queue);

static void ProtectedInitSingleBlockXfer(PDMA_SYNCRUN_CONTEXT context);
static void ProtectedGetChanState(PDMA_SYNCRUN_CONTEXT context);
static void ProtectedSetDoorBell(PDMA_SYNCRUN_CONTEXT context);

/**
 * The Interrupt Status Register for read and write.
 */
typedef union {
  struct {
    //LSB
    uint32_t      DoneSta     :8;
    uint32_t      Rsvd0       :8;
    uint32_t      AbortSta    :8;
    uint32_t      Rsvd1       :8;
    //MSB
  };
  uint32_t AsDword;
} INT_STATUS;

/**
 * The Interrupt Clear Register for read and write.
 */
typedef union {
  struct {
    //LSB
    uint32_t      DoneClr     :8;
    uint32_t      Rsvd0       :8;
    uint32_t      AbortClr    :8;
    uint32_t      Rsvd1       :8;
    //MSB
  };
  uint32_t AsDword;
} INT_CLEAR;

/**
 * The Enable Register for read and write.
 */
typedef union {
  struct {
    //LSB
    uint32_t      Enb         :1;  // 0
    uint32_t      Reserved0   :31; // 1:31
    //MSB
  }; 
  uint32_t AsDword;
} ENB;

/**
 * The DMA Control Register.
 */
typedef union {
  struct {
    //LSB
    uint32_t      WrChans     :3;  // 0:2
    uint32_t      Reserved0   :13; // 3:15
    uint32_t      RdChans     :3;  // 16:18
    uint32_t      Reserved1   :13; // 19:31
    //MSB
  };
  uint32_t AsDword;
} DMA_CTRL;

/**
 * The Doorbell Register for read and write.
 */
typedef union {
  struct {
    //LSB
    uint32_t      Chnl        :3;  // 0
    uint32_t      Reserved0   :28; // 3:30
    uint32_t      Stop        :1;  // 31
    //MSB
  }; 
  uint32_t AsDword;
} DOORBELL;



/**
 * The Error Status Register for read and write.
 */
typedef union {
  struct {
    //LSB
    uint32_t      BrdgErr     :8;
    uint32_t      Rsvd0       :8;
    uint32_t      LLErr       :8;
    uint32_t      Rsvd1       :8;
    //MSB
  };
  uint32_t AsDword;
} WR_ERR, RD_ERR_LO;

/**
 * Extra Error Status Register for read.
 */
typedef union {
  struct {
    //LSB
    uint32_t      UrErr       :8;
    uint32_t      CaErr       :8;
    uint32_t      ToErr       :8;
    uint32_t      EpErr       :8;
    //MSB
  };
  uint32_t AsDword;
} RD_ERR_HI;

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
    uint32_t      Rsvd1       :1;    // 7
    uint32_t      CCS         :1;    // 8
    uint32_t      LLEN        :1;    // 9
    uint32_t      b_64S       :1;    // 10
    uint32_t      b_64D       :1;    // 11
    uint32_t      PF          :5;    // 12:16
    uint32_t      Rsvd2       :7;    // 17:23
    uint32_t      SN          :1;    // 24
    uint32_t      RO          :1;    // 25
    uint32_t      TD          :1;    // 26
    uint32_t      TC          :3;    // 27:29
    uint32_t      AT          :2;    // 30:31
    //MSB
  };
  uint32_t AsDword;
} CHAN_CTRL_LO;

/**
 * The Channel Control Register high part for read and write.
 */
typedef union {
  struct {
    //LSB
    uint32_t      VFEnb       :1;     // 0
    uint32_t      VFunc       :8;     // 1-8
    uint32_t      Rsvd0       :23;    // 9-31
    //MSB
  };
  uint32_t AsDword;
} CHAN_CTRL_HI;

/**
 * The Context Registers for read and write.
 */
typedef struct {
  CHAN_CTRL_LO    CtrlLo;     // 0x00
  CHAN_CTRL_HI    CtrlHi;     // 0x04
  uint32_t          XferSize;   // 0x08
  uint32_t          SarPtrLo;   // 0x0C
  uint32_t          SarPtrHi;   // 0x10
  uint32_t          DarPtrLo;   // 0x14
  uint32_t          DarPtrHi;   // 0x18
  uint32_t          ElPtrLo;    // 0x1C
  uint32_t          ElPtrHi;    // 0x20
} CTX_REGS;

/**
 * The Context Register selector for read and write.
 */
typedef union {
  struct {
    //LSB
    uint32_t      ViwPnt      :3;     // 0:2
    uint32_t      Rsvd0       :28;    // 3:30
    uint32_t      Sel         :1;     // 31
    //MSB
  };
  uint32_t  AsDword;
} CTX_SEL;

/**
 * The Channel Weight Register.
 */
typedef union {
  struct {
    //LSB
    uint32_t      Weight0     :5;     // 0:4
    uint32_t      Weight1     :5;     // 5:9
    uint32_t      Weight2     :5;     // 10:14
    uint32_t      Weight3     :5;     // 15:19
    uint32_t      Rsvd        :12;    // 20:31
    //MSB
  };
  uint32_t AsDword;
} WEIGHT;

/**
 * The Linked List Interrupt Error Enable Register for write and read
 */
typedef union {
  struct {
    //LSB
    uint32_t      Reie        :8;     // 0:7
    uint32_t      Rsvd0       :8;     // 8:15
    uint32_t      Leie        :8;     // 16:23
    uint32_t      Rsvd1       :8;     // 24:31
    //MSB
  };
  uint32_t AsDword;
} LL_ERR_ENB;


/**
 * DMA Registers starts from 0x970 to 0xB2C.
 */
struct DMA_REGS {        
  uint32_t          Dummy[604];     // 0x000 - 0x96F
  uint32_t          Rsvd0[2];       // 0x970 - 0X974
  DMA_CTRL        DmaCtrl;        // 0x978
  ENB             WrEnb;          // 0x97C
  DOORBELL        WrDb;           // 0x980
  uint32_t          WrCtrl;         // 0x984
  WEIGHT          WrWeiLo;        // 0x988
  WEIGHT          WrWeiHi;        // 0x98C
  uint32_t          Rsvd1[3];       // 0x990 - 0x998
  ENB             RdEnb;          // 0x99C
  DOORBELL        RdDb;           // 0x9A0
  uint32_t          RdCtrl;         // 0x9A4
  WEIGHT          RdWeiLo;        // 0x9A8
  WEIGHT          RdWeiHi;        // 0x9AC
  uint32_t          Rsvd2[3];       // 0x9B0 - 0x9B8
  INT_STATUS      WrIntSta;       // 0x9BC
  uint32_t          Rsvd3;          // 0x9C0 
  uint32_t          WrIntMsk;       // 0x9C4
  INT_CLEAR       WrIntClr;       // 0x9C8
  WR_ERR          WrErrSta;       // 0x9CC
  uint32_t          WrMsgDonAddrLo; // 0x9D0
  uint32_t          WrMsgDonAddrHi; // 0x9D4
  uint32_t          WrMsgAbtAddrLo; // 0x9D8
  uint32_t          WrMsgAbtAddrHi; // 0x9DC
  int16_t          WrMsgData[8];   // 0x9E0 - 0x9EC
  uint32_t          Rsvd4[4];       // 0x9F0 - 0x9FC
  LL_ERR_ENB      WrIntLLErrEnb;  // 0xA00
  uint32_t          Rsvd5[3];       // 0xA04 - 0xA0C
  INT_STATUS      RdIntSta;       // 0xA10
  uint32_t          Rsvd6;          // 0xA14
  uint32_t          RdIntMsk;       // 0xA18
  INT_CLEAR       RdIntClr;       // 0xA1C
  uint32_t          Rsvd7;          // 0xA20
  RD_ERR_LO       RdErrStaLo;     // 0xA24
  RD_ERR_HI       RdErrStaHi;     // 0xA28
  uint32_t          Rsvd8[2];       // 0xA2C - 0xA30
  LL_ERR_ENB      RdIntLLErrEnb;  // 0xA34
  uint32_t          Rsvd9;          // 0xA38
  uint32_t          RdMsgDonAddrLo; // 0xA3C
  uint32_t          RdMsgDonAddrHi; // 0xA40
  uint32_t          RdMsgAbtAddrLo; // 0xA44
  uint32_t          RdMsgAbtAddrHi; // 0xA48
  int16_t          RdMsgData[8];   // 0xA4C - 0xA58
  uint32_t          Rsvd10[4];      // 0xA5C - 0xA68
  CTX_SEL         CtxSel;         // 0xA6C 
  CTX_REGS        CtxRegs;        // 0xA70 - 0xA90 
};


/******************************************************************************
 *
 *  EXTERN FUNCTION DECLARATIONS
 *
 ******************************************************************************/

void DmaSetOp(PDMA_REGS pdreg, DMA_DIRC dirc, int enb);
void DmaSetDoorBell(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chnl, int stop);
void DmaSetXferSize(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chnl, uint32_t size);
void DmaSetSrc(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chnl, PHYADDR addr);
void DmaSetDst(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chnl, PHYADDR addr);
void DmaSetWei(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chnl, uint8_t wei);
void DmaSetCtrl(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chnl, DMA_XFER_MODE mode);
void DmaGetChanNum(PDMA_REGS pdreg, uint8_t *wrch, uint8_t *rdch);
void DmaGetChanStatus(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chnl,
                      DMA_CHAN_STATUS *status);
void DmaGetXferSize(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chnl, uint32_t *size);
void DmaGetElementPtr(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chan, PHYADDR *addr);
HAL_DMA_RET HalDmaInit(PDMA_RSC pdrsc);

#endif //
