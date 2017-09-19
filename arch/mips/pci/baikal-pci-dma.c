#include "baikal-pci-dma.h"
#include "asm/addrspace.h"


PDMA_RSC dma_rsc_val;

void pci_dw_dma_init(void)
{
	
	if ((dma_rsc_val = kmalloc(sizeof(DMA_RSC), GFP_KERNEL)) == NULL) {
		printk("%s: cannot allocate memory\n", __FUNCTION__);
		return;
	}

	/* Set PCIe host controller base address. */
	dma_rsc_val->DmaRegs = (PDMA_REGS)0xBF052000;

	/* Init PCI DMA.*/
	if (HalDmaInit(dma_rsc_val) != DMA_SUCCESS) {
		printk("%s: init failed\n", __FUNCTION__);
	}
}

void DmaGetChanNum(PDMA_REGS pdreg, uint8_t *wrch, uint8_t *rdch) {

  DMA_CTRL ctrl;

  ctrl.AsDword = PEDD_READ_REG(&pdreg->DmaCtrl);
  *wrch = ctrl.WrChans;
  *rdch = ctrl.RdChans;
}


HAL_DMA_RET HalDmaInit(PDMA_RSC pdrsc) {
  PDMA_REGS pdregs;
  /* Check arguments */
  if (pdrsc == NULL) {
    return DMA_NULL_ARG;
  }

  pdregs = pdrsc->DmaRegs;

  DTRACE("%s: DmaRegs = 0x%x\n", __FUNCTION__, (uint32_t)pdrsc->DmaRegs);

  /* Firstly reset DMA read/write logic */
  DmaSetOp(pdrsc->DmaRegs, DMA_WRITE, 0/*FALSE*/);
  DmaSetOp(pdrsc->DmaRegs, DMA_READ, 0/*FALSE*/);

  /* Read enabled write/read channels */
  DmaGetChanNum(pdregs, &pdrsc->WrChans, &pdrsc->RdChans);
  if (pdrsc->WrChans == 0 && pdrsc->RdChans == 0) {
    DTRACE("No active DMA channels found!");
    return DMA_FATAL;
  }
  printk("PCIe DMA: read chans=%u, write chans=%u\n", (uint32_t)pdrsc->RdChans, (uint32_t)pdrsc->WrChans);
  {
    int i;
    DMA_SYNCRUN_CONTEXT _context = {dma_rsc_val, DMA_WRITE, 0, NULL};
    for (i = 0; i < pdrsc->WrChans; i++) {
      CTX_SEL sel = { 0 };

      sel.Sel = (uint32_t) DMA_WRITE;
      sel.ViwPnt = (uint32_t) i;

      PEDD_WRITE_REG( &pdregs->CtxSel, sel.AsDword );
      pdregs->CtxRegs.CtrlLo.AsDword = 0;     // 0x00
      pdregs->CtxRegs.CtrlLo.CS = CHAN_STOPPED;
      pdregs->CtxRegs.CtrlHi.AsDword = 0;     // 0x04
	    pdregs->CtxRegs.XferSize = 0;   // 0x08
	    pdregs->CtxRegs.SarPtrLo = 0;   // 0x0C
	    pdregs->CtxRegs.SarPtrHi = 0;   // 0x10
	    pdregs->CtxRegs.DarPtrLo = 0;   // 0x14
	    pdregs->CtxRegs.DarPtrHi = 0;   // 0x18
	    pdregs->CtxRegs.ElPtrLo = 0;    // 0x1C
	    pdregs->CtxRegs.ElPtrHi = 0;    // 0x20

      _context.Chan = i;
      ProtectedGetChanState (&_context);
    }
    for (i = 0; i < pdrsc->RdChans; i++) {
      CTX_SEL sel = { 0 };

      sel.Sel = (uint32_t) DMA_READ;
      sel.ViwPnt = (uint32_t) i;

      PEDD_WRITE_REG( &pdregs->CtxSel, sel.AsDword );
      pdregs->CtxRegs.CtrlLo.AsDword = 0;     // 0x00
      pdregs->CtxRegs.CtrlLo.CS = CHAN_STOPPED;
      pdregs->CtxRegs.CtrlHi.AsDword = 0;     // 0x04
	    pdregs->CtxRegs.XferSize = 0;   // 0x08
	    pdregs->CtxRegs.SarPtrLo = 0;   // 0x0C
	    pdregs->CtxRegs.SarPtrHi = 0;   // 0x10
	    pdregs->CtxRegs.DarPtrLo = 0;   // 0x14
	    pdregs->CtxRegs.DarPtrHi = 0;   // 0x18
	    pdregs->CtxRegs.ElPtrLo = 0;    // 0x1C
	    pdregs->CtxRegs.ElPtrHi = 0;    // 0x20

      _context.Chan = i;
      ProtectedGetChanState (&_context);
    }
  }

#if 0
  PHYADDR addr = pdrsc->MsiAddr;
  uint16_t data = pdrsc->MsiData;
  /* We use exactly the same addr/data for all interrupts and channels */
  DmaSetMSIAddr(pdrsc->DmaRegs, DMA_WRITE, DMA_INT_DONE, addr);
  DmaSetMSIAddr(pdrsc->DmaRegs, DMA_WRITE, DMA_INT_ABORT, addr);
  DmaSetMSIAddr(pdrsc->DmaRegs, DMA_READ, DMA_INT_DONE, addr);
  DmaSetMSIAddr(pdrsc->DmaRegs, DMA_READ, DMA_INT_ABORT, addr);

  uint8_t chan;
  for (chan = 0; chan < pdrsc->WrChans; chan++) {
    DmaSetMSIData(pdrsc->DmaRegs, DMA_WRITE, chan, data);
  }
  for (chan = 0; chan < pdrsc->RdChans; chan++) {
    DmaSetMSIData(pdrsc->DmaRegs, DMA_READ, chan, data);
  }
#endif

  /* Zero out internal variables */
  memset(&pdrsc->WrChanInfo, 0, 8 * sizeof(CHAN_INFO));
  memset(&pdrsc->RdChanInfo, 0, 8 * sizeof(CHAN_INFO));

  return DMA_SUCCESS;
}

HAL_DMA_RET DWDmaSingleBlockWrite(uint8_t chan, void* src, void* dst, uint32_t size, uint8_t wei, int queue)
{
	PHYADDR src_dma, dst_dma;

	src_dma.LowPart = (uint32_t)(src);
	src_dma.HighPart = 0;

	dst_dma.LowPart = (uint32_t)(dst);
	dst_dma.HighPart = 0;
	DTRACE("%s: regs=0x%x src=0x%x dst=0x%x\n", __FUNCTION__, (uint32_t)dma_rsc_val->DmaRegs, (uint32_t)src, (uint32_t)dst);
	return SingleBlockRW(dma_rsc_val, DMA_WRITE, chan, src_dma, dst_dma, size, wei, queue);
}

void DWGetChanWrStatus(uint8_t chan, DMA_CHAN_STATUS *status)
{
	return DmaGetChanStatus(dma_rsc_val->DmaRegs, DMA_WRITE, chan, status);
}

static HAL_DMA_RET SingleBlockRW(PDMA_RSC pdrsc, DMA_DIRC dirc, uint8_t chan,
                                 PHYADDR src, PHYADDR dst, uint32_t size, uint8_t wei, int queue)
{
  DMA_SYNCRUN_CONTEXT context = {pdrsc, dirc, chan, NULL};
  PCHAN_INFO pChanInfo = (dirc == DMA_WRITE) ? &pdrsc->WrChanInfo[chan]
    : &pdrsc->RdChanInfo[chan];

  DTRACE("%s: chan=%u size=%u wei=%u queue=%d\n", __FUNCTION__, (uint32_t)chan, (uint32_t)size, (uint32_t)wei, queue);

  /* Check arguments */
  if (pdrsc == NULL) {
    DTRACE("pdrsc == NULL\n");
    return DMA_NULL_ARG;
  }
  if ((dirc == DMA_WRITE && chan >= pdrsc->WrChans) || (dirc == DMA_READ
                                                        && chan >= pdrsc->RdChans)) {
    DTRACE("DMA_INVALID_CHAN\n");
    return DMA_INVALID_CHAN;
  }
  if (size < MIN_BLOCK_SIZE || size > MAX_BLOCK_SIZE) {
    DTRACE("DMA_INVALID_BLOCK_SIZE");
    return DMA_INVALID_BLOCK_SIZE;
  }
  if (wei < MIN_WEIGHT || wei > MAX_WEIGHT) {
    DTRACE("DMA_INVALID_WEIGHT");
    return DMA_INVALID_WEIGHT;
  }

  /* Check the status of the channel to make sure it's not running. Otherwise
   * we can't start new transfer on this channel.
   */

  if (pChanInfo->Status == CHAN_RUNNING) {
    return DMA_CHAN_BUSY;
  }
  DTRACE("DMA R/W. dirc=%d channel=%d size=%d\n", dirc, chan, size);

  /* Reset/init channel info */
  pChanInfo->FeedSize = size;
  pChanInfo->SrcPos = src;
  pChanInfo->DstPos = dst;
  pChanInfo->Weight = wei;
  pChanInfo->LLElements = 0;
  pChanInfo->DoneIntrCnt = 0;
  pChanInfo->AbortIntrCnt = 0;
  pChanInfo->Error = DMA_ERR_NONE;

  /* Configure the context registers for single block transfer. */
  ProtectedInitSingleBlockXfer(&context);

  /* Set Doorbell register when neccessary */
  if (queue) {
    pChanInfo->Status = CHAN_QUEUING;
  } else {
    ProtectedSetDoorBell(&context);
  }
  return DMA_SUCCESS;
}


static void ProtectedInitSingleBlockXfer(PDMA_SYNCRUN_CONTEXT context) {
  PDMA_RSC pdrsc = context->DmaRsc;
  DMA_DIRC dirc = context->Dirc;
  uint8_t chan = context->Chan;
  PCHAN_INFO pChanInfo = (dirc == DMA_WRITE) ? &pdrsc->WrChanInfo[chan]
    : &pdrsc->RdChanInfo[chan];
  uint32_t blkSize = (uint32_t)pChanInfo->FeedSize;
  PHYADDR src = pChanInfo->SrcPos;
  PHYADDR dst = pChanInfo->DstPos;
  uint8_t wei = pChanInfo->Weight;
  DTRACE("SingleBlock: src.lo=0x%08x, dst.lo=0x%08x chan=%d\n", src.LowPart, dst.LowPart, chan);

  DmaSetOp(pdrsc->DmaRegs, dirc, TRUE);
  DmaSetCtrl(pdrsc->DmaRegs, dirc, chan, DMA_SINGLE_BLOCK);
  DmaSetXferSize(pdrsc->DmaRegs, dirc, chan, blkSize);
  DmaSetSrc(pdrsc->DmaRegs, dirc, chan, src);
  DmaSetDst(pdrsc->DmaRegs, dirc, chan, dst);
  DmaSetWei(pdrsc->DmaRegs, dirc, chan, wei);
}

static void ProtectedGetChanState(PDMA_SYNCRUN_CONTEXT context) {
  PDMA_RSC pdrsc = context->DmaRsc;
  DMA_DIRC dirc = context->Dirc;
  uint8_t chan = context->Chan;
  PCHAN_INFO pChanInfo = (dirc == DMA_WRITE) ? &pdrsc->WrChanInfo[chan]
    : &pdrsc->RdChanInfo[chan];

  DmaGetChanStatus(pdrsc->DmaRegs, dirc, chan, &pChanInfo->Status);
  DTRACE("%s: dirc=%d channel=%d status=%d\n", __FUNCTION__, dirc, (uint32_t)chan, pChanInfo->Status);
  DmaGetXferSize(pdrsc->DmaRegs, dirc, chan, &pChanInfo->XferSize);
  DTRACE("%s: dirc=%d channel=%d size=%u\n", __FUNCTION__, dirc, (uint32_t)chan, pChanInfo->XferSize);
  DmaGetElementPtr(pdrsc->DmaRegs, dirc, chan, &pChanInfo->ElementPos);
  DTRACE("%s: dirc=%d channel=%d pos=%llu\n", __FUNCTION__, dirc, (uint32_t)chan, pChanInfo->ElementPos.QuadPart);
}

void DmaSetOp(PDMA_REGS pdreg, DMA_DIRC dirc, int enb) {
  uint32_t val = enb ? 0x1 : 0x0;
  uint32_t* reg = dirc == DMA_WRITE ? &pdreg->WrEnb.AsDword
    : &pdreg->RdEnb.AsDword;

  PEDD_WRITE_REG(reg, val);

  /* We need to make sure the reset has been propagated to all logic. */
  if (!enb) {
    int i = 200;
    while(PEDD_READ_REG(reg)) {
      DTRACE("DmaSetOp: Wait 5 ms...");
      msleep(5);
      if (i-- == 0) {
        DTRACE("DmaSetOp: Reset propagation time out!");
        break;
      }
    };
  }
}

void DmaSetCtrl(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chan, DMA_XFER_MODE mode) {
  CHAN_CTRL_LO cctrl;
  CTX_SEL sel = {0};
  cctrl.AsDword = 0x0; 

  switch ( mode ) {
  case DMA_SINGLE_BLOCK:
    cctrl.LIE = 1;
    cctrl.RIE = 1;
    cctrl.TD = 1;
    break;
  case DMA_MULTI_BLOCK:
    cctrl.LLEN = 1;
    cctrl.CCS = 1;
    cctrl.TD = 1;
  default:
    // TODO: Not supported
    break;
  } 

  sel.Sel = (uint32_t)dirc;
  sel.ViwPnt = (uint32_t)chan;

  PEDD_WRITE_REG( &pdreg->CtxSel, sel.AsDword );
  // TODO: need to wait for hardware operation?
  PEDD_WRITE_REG( &pdreg->CtxRegs.CtrlLo, cctrl.AsDword );
  // TODO: Virtual function
  PEDD_WRITE_REG( &pdreg->CtxRegs.CtrlHi, 0x0);
}

void DmaSetXferSize(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chan, uint32_t size) {
  CTX_SEL sel = {0};    

  sel.Sel = (uint32_t)dirc;
  sel.ViwPnt = (uint32_t)chan;

  PEDD_WRITE_REG( &pdreg->CtxSel, sel.AsDword );
  // TODO: need to wait for hardware operation?
  PEDD_WRITE_REG( &pdreg->CtxRegs.XferSize, size );
}
void DmaSetSrc( PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chan, PHYADDR addr ) {
  CTX_SEL sel = {0};    

  sel.Sel = (uint32_t)dirc;
  sel.ViwPnt = (uint32_t)chan;

  PEDD_WRITE_REG( &pdreg->CtxSel, sel.AsDword );
  // TODO: need to wait for hardware operation?
  PEDD_WRITE_REG( &pdreg->CtxRegs.SarPtrLo, addr.LowPart );
  PEDD_WRITE_REG( &pdreg->CtxRegs.SarPtrHi, addr.HighPart );
}

void DmaSetDst( PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chan, PHYADDR addr ) {

  CTX_SEL sel = {0};    

  sel.Sel = (uint32_t)dirc;
  sel.ViwPnt = (uint32_t)chan;

  PEDD_WRITE_REG( &pdreg->CtxSel, sel.AsDword );
  // TODO: need to wait for hardware operation?
  PEDD_WRITE_REG( &pdreg->CtxRegs.DarPtrLo, addr.LowPart );
  PEDD_WRITE_REG( &pdreg->CtxRegs.DarPtrHi, addr.HighPart );
}

void DmaSetWei(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chan, uint8_t wei) {
  uint32_t* reg = dirc == DMA_WRITE
    ? chan < 4 ? &pdreg->WrWeiLo.AsDword : &pdreg->WrWeiHi.AsDword
    : chan < 4 ? &pdreg->RdWeiLo.AsDword : &pdreg->RdWeiHi.AsDword;

  WEIGHT val;
  val.AsDword = PEDD_READ_REG(reg);
  switch (chan % 4) {
  case 0:
    val.Weight0 = wei;
    break;
  case 1:
    val.Weight1 = wei;
    break;
  case 2:
    val.Weight2 = wei;
    break;
  default:
    val.Weight3 = wei;
    break;
  }
  PEDD_WRITE_REG(reg, val.AsDword);
}

void DmaGetChanStatus(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chan,
                      DMA_CHAN_STATUS *status) {

  CTX_SEL sel = { 0 };
  CHAN_CTRL_LO ctrllo;

  sel.Sel = (uint32_t) dirc;
  sel.ViwPnt = (uint32_t) chan;

//DTRACE("%s: pdreg=%p CTX sel addr=%08X\n", __FUNCTION__, pdreg, (uint32_t)(&pdreg->CtxSel) - (uint32_t)pdreg);

  PEDD_WRITE_REG( &pdreg->CtxSel, sel.AsDword );
  // TODO: need to wait for hardware operation?

//DTRACE("%s: pdreg=%p CTX CtrlLo=%08X\n", __FUNCTION__, pdreg, (uint32_t)(&pdreg->CtxRegs.CtrlLo) - (uint32_t)pdreg);
  ctrllo.AsDword = PEDD_READ_REG( &pdreg->CtxRegs.CtrlLo );

  *status = (DMA_CHAN_STATUS) ctrllo.CS;
}
void DmaGetXferSize( PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chan, uint32_t *size) {
  CTX_SEL sel = {0};
  sel.Sel = (uint32_t)dirc;
  sel.ViwPnt = (uint32_t)chan;

  PEDD_WRITE_REG( &pdreg->CtxSel, sel.AsDword );
  // TODO: need to wait for hardware operation?
  *size = PEDD_READ_REG( &pdreg->CtxRegs.XferSize );
}
void DmaGetElementPtr(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chan,
                      PHYADDR *addr) {
  CTX_SEL sel = {0};
  sel.Sel = (uint32_t)dirc;
  sel.ViwPnt = (uint32_t)chan;

  PEDD_WRITE_REG( &pdreg->CtxSel, sel.AsDword );
  // TODO: need to wait for hardware operation?
  addr->LowPart = PEDD_READ_REG( &pdreg->CtxRegs.ElPtrLo);
  addr->HighPart = PEDD_READ_REG( &pdreg->CtxRegs.ElPtrHi);
}

static void ProtectedSetDoorBell(PDMA_SYNCRUN_CONTEXT context) {
  PDMA_RSC pdrsc = context->DmaRsc;
  DMA_DIRC dirc = context->Dirc;
  uint8_t chan = context->Chan;
  DmaSetDoorBell(pdrsc->DmaRegs, dirc, chan, FALSE);
}

void DmaSetDoorBell(PDMA_REGS pdreg, DMA_DIRC dirc, uint8_t chan, int stop) {
  DOORBELL db = {0};
  db.Stop = (uint32_t)stop;
  db.Chnl = (uint32_t)chan;
    
  if ( dirc == DMA_READ ) {  
    PEDD_WRITE_REG( &pdreg->RdDb, db.AsDword );
  } else {
    PEDD_WRITE_REG( &pdreg->WrDb, db.AsDword );
  } 
}
