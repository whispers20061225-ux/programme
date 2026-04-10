 
// WARNING: Instantiation, initialization steps are missing from the examples

// EXAMPLE FOR FDCAN input CLK, Brg::GetClk() and Brg::IsFdcanSupport

    Brg *m_pBrg;
	**********[ Missing init steps ]**********
	Brg_StatusT brgStat = BRG_NO_ERR;
	uint32_t currFreqKHz=0;
	uint8_t com = COM_FDCAN;
	uint32_t StlHClkKHz, comInputClkKHz;
	// Get the current bridge input Clk
	brgStat = m_pBrg->GetClk(com, &comInputClkKHz, &StlHClkKHz);
	if ((brgStat == BRG_CMD_NOT_SUPPORTED) && (m_pBrg->IsFdcanSupport()==false)) {
		// expected: FDCAN not supported by all STLINK
		brgStat = BRG_NO_ERR;
	} else {
		printf( "FDCAN input CLK: %d KHz, ST-Link HCLK: %d KHz \n", (int)comInputClkKHz, (int)StlHClkKHz);
	}

// EXAMPLE FOR FDCAN Initialization, Brg::InitFDCAN(), Brg::GetFDCANbaudratePrescal()

    Brg *m_pBrg;
	**********[ Missing init steps ]**********
	Brg_StatusT cBrgExample::FdcanTestInit(void)
	{
		Brg_StatusT brgStat = BRG_NO_ERR;
		Brg_FdcanInitT fdcanParam;
		uint32_t nomPrescal, dataPrescal;
		uint32_t fdcanNomBaudrate = 250000; //250kbps
		uint32_t fdcanDataBaudrate = 1000000; // 1Mbps
		uint32_t finalBaudrate = 0;
	
		// FDCAN nominal baudrate = 250kbps data baudrate = 1Mbps
		// STLINK-V3PWR FDCAN clock = 80MHz
		fdcanParam.FrameMode = FDCAN_FRAME_FD_BRS; // default FDCAN mode (CAN and FDCAN support)
		// FDCAN Nominal bit Time: N=1+23+40+16= 80, nominal baudrate 250000bps (-> prescal = 4 = (FdcanClk = 80MHz)/(80*250000))
		fdcanParam.NomBitTimeConf.PropSegInTq = 23;
		fdcanParam.NomBitTimeConf.PhaseSeg1InTq = 40;
		fdcanParam.NomBitTimeConf.PhaseSeg2InTq = 16;
		fdcanParam.NomBitTimeConf.SjwInTq = 16;
		// get Nominal prescaler (bIsNomBitTime = true)
		brgStat = m_pBrg->GetFDCANbaudratePrescal(&fdcanParam.NomBitTimeConf, fdcanNomBaudrate, (uint32_t*)&nomPrescal, (uint32_t*)&finalBaudrate, fdcanParam.FrameMode, true);
		if (brgStat == BRG_COM_FREQ_MODIFIED) {
			printf("WARNING Bridge FDCAN init Nominal baudrate asked %d bps but applied %d bps \n", (int)fdcanNomBaudrate, (int)finalBaudrate);
		} else if( brgStat == BRG_COM_FREQ_NOT_SUPPORTED ) {
			printf("ERROR Bridge FDCAN init baudrate %d bps not possible (invalid prescaler: %d) change Bit Time or baudrate settings. \n", (int)fdcanNomBaudrate, (int)nomPrescal);
		}
		
		if (brgStat == BRG_NO_ERR) {
			// FDCAN Data bit Time: N=1+1+14+4= 20, data baudrate 1000000bps (-> prescal = 4 = (FdcanClk = 80MHz)/(20*1000000))
			fdcanParam.DataBitTimeConf.PropSegInTq = 1;
			fdcanParam.DataBitTimeConf.PhaseSeg1InTq = 14;
			fdcanParam.DataBitTimeConf.PhaseSeg2InTq = 4;
			fdcanParam.DataBitTimeConf.SjwInTq = 4;
			// get Data prescaler (bIsNomBitTime = false)
			brgStat = m_pBrg->GetFDCANbaudratePrescal(&fdcanParam.DataBitTimeConf, fdcanDataBaudrate, (uint32_t*)&dataPrescal, (uint32_t*)&finalBaudrate, fdcanParam.FrameMode, false);
			if( brgStat == BRG_COM_FREQ_MODIFIED ) {
				brgStat = BRG_NO_ERR;
				printf("WARNING Bridge FDCAN init Data baudrate asked %d bps but applied %d bps \n", (int)fdcanDataBaudrate, (int)finalBaudrate);
			} else if( brgStat == BRG_COM_FREQ_NOT_SUPPORTED ) {
				printf("ERROR Bridge FDCAN init baudrate %d bps not possible (invalid prescaler: %d) change Bit Time or baudrate settings. \n", (int)fdcanDataBaudrate, (int)dataPrescal);
			}
		}
	
		// Other bit time setting example: nominal baudrate = 1Mbps data baudrate = 8Mbps 
		// FDCAN Nominal bit Time: N=1+23+8+8= 40, nominal baudrate 1000000bps (-> prescal = 4 = (FdcanClk = 80MHz)/(40*1000000))
		// fdcanParam.NomBitTimeConf.PropSegInTq = 23; fdcanParam.NomBitTimeConf.PhaseSeg1InTq = 8; fdcanParam.NomBitTimeConf.PhaseSeg2InTq = 8; fdcanParam.NomBitTimeConf.SjwInTq = 8;
		// FDCAN Data bit Time: N=1+1+5+3= 10, data baudrate 8000000bps (-> prescal = 1 = (FdcanClk = 80MHz)/(10*8000000))
		// fdcanParam.DataBitTimeConf.PropSegInTq = 1; fdcanParam.DataBitTimeConf.PhaseSeg1InTq = 5; fdcanParam.DataBitTimeConf.PhaseSeg2InTq = 3; fdcanParam.DataBitTimeConf.SjwInTq = 3;
	
		if (brgStat == BRG_NO_ERR) {
			fdcanParam.NomPrescaler = nomPrescal;
			fdcanParam.DataPrescaler = dataPrescal;
			// fdcanParam.DataBitTimeConf already filled above
			// fdcanParam.NomBitTimeConf already filled above
			fdcanParam.Mode = FDCAN_MODE_INT_LOOPBACK; // FDCAN_MODE_INT_LOOPBACK (for internal FDCAN STLINK test) FDCAN_MODE_NORMAL for loopback with real target
			// fdcanParam.FrameMode already filled above
			fdcanParam.Fifo0Mode = FDCAN_FIFO_BLOCKING;
			fdcanParam.Fifo1Mode = FDCAN_FIFO_BLOCKING;
			fdcanParam.bIsArEn = true;
			fdcanParam.bIsTxpEn = false;
			fdcanParam.bIsPexhEn = true;
			// transceiver delay compensation not required for low baudrate
			fdcanParam.bIsTdcEn = false;
			fdcanParam.tdcOffset = 0;
			fdcanParam.tdcFilter = 0;
			// Without transceiver delay compensation, the bitrate in the data phase of a CAN FD frame is limited by the transceivers loop delay.
			// for 8Mbps it may be needed to use fdcanParam.bIsTdcEn = true; with as example fdcanParam.tdcOffset = 7; (to be set according to FDCAN bus delay)
	
			// bStartBus = true -> StartFDCAN() called in InitFDCAN()
			brgStat = m_pBrg->InitFDCAN(&fdcanParam, BRG_INIT_FULL, true);
		}
	
		return brgStat;
	}

// EXAMPLE FOR FDCAN loopback test, Brg::StartMsgReceptionFDCAN() Brg::InitFilterFDCAN() Brg::WriteMsgFDCAN() Brg::GetRxMsgNbFDCAN() Brg::GetRxMsgFDCAN()</B>\n

    Brg *m_pBrg;
	**********[ Missing init steps ]**********
    // use init with fdcanParam.Mode = FDCAN_MODE_INT_LOOPBACK or connect a target that send back the received message

	// Test FDCAN commands Brg::StartMsgReceptionFDCAN() Brg::InitFilterFDCAN() Brg::WriteMsgFDCAN() Brg::GetRxMsgNbFDCAN() Brg::GetRxMsgFDCAN()
	Brg_StatusT cBrgExample::FdcanTestLoopback(bool bIsFdcanClassicCan)
	{
		Brg_StatusT brgStat = BRG_NO_ERR;
		uint8_t dataRx[64], dataTx[64]; // Classic CAN: DLC limited to 8, FDCAN DLC limited to 64
		int i, nb;
		Brg_FdcanFilterConfT filterConf;
		Brg_FdcanRxMsgT fdcanRxMsg;
		Brg_FdcanMsgT fdcanTxMsg;
		uint8_t size=0;
		uint8_t maxMsgSize;
		if (bIsFdcanClassicCan == true) {
			maxMsgSize = 8;
			printf("FDCAN-CAN Loopback test using CAN messages \n");
		} else { // full FDCAN tests
			maxMsgSize = 64;
			printf("FDCAN-FD Loopback test using FDCAN messages \n");
		}
	
		brgStat =  m_pBrg->StartMsgReceptionFDCAN();
		if (brgStat != BRG_NO_ERR) {
			printf("FDCAN StartMsgReceptionFDCAN failed \n");
		}
	
		// robustess test
		if (brgStat == BRG_NO_ERR) {
			int maxLoop;
			maxLoop = 500;
	
			// Receive messages with specific ID with all Classic CAN DLC possible size (0->8) and, if not classical CAN only, all FDCAN size (12, 16, 20, 24, 32, 48 or 64)
			// std filter0 FDCAN prepare receive in FIFO0
			filterConf.AssignedFifo = CAN_MSG_RX_FIFO0;
			filterConf.bIsFilterEn = true;
			filterConf.FilterNb = 0; //0 to 27 standard or 0-7 extended
			filterConf.FilterMode = FDCAN_FILTER_ID_LIST; //FDCAN_FILTER_ID_RANGE  FDCAN_FILTER_ID_MASK FDCAN_FILTER_ID_LIST
			filterConf.bIsFilterReject = false; //Messages passing the filter are NOT rejected
			filterConf.ID1 = 0x789;
			filterConf.ID2 = 0x789;
			filterConf.IDE = CAN_ID_STANDARD;//CAN_ID_STANDARD CAN_ID_EXTENDED
	
			brgStat = m_pBrg->InitFilterFDCAN(&filterConf);
			if (brgStat != BRG_NO_ERR) {
				printf("FDCAN filter0 init failed \n");
			}
			//Init Rx / Tx msg	
			FdcanMsgRxInitStd(&fdcanRxMsg, FDCAN_F_CLASSIC_CAN);
			fdcanRxMsg.Header.IDE = CAN_ID_EXTENDED; // must be = canTxMsg.IDE for the test
			if (bIsFdcanClassicCan == true) {
				FdcanMsgTxInitStd(&fdcanTxMsg, FDCAN_F_CLASSIC_CAN);
			} else { // full FDCAN tests
				FdcanMsgTxInitStd(&fdcanTxMsg, FDCAN_F_FD_CAN);
			}
			fdcanTxMsg.ID = 0x789; // must be <=0x7FF for CAN_ID_STANDARD, <=0x1FFFFFFF for CAN_ID_EXTENDED;
	
			nb = 0;
			while ( (brgStat == BRG_NO_ERR) && (nb<maxLoop) ) {
				for (i=0; i<maxMsgSize; i++) {
					dataRx[i] = 0;
					dataTx[i] = (BYTE)(nb+i);
				}
				fdcanRxMsg.Header.DLC = 0;
				fdcanTxMsg.DLC = 2; // unused in CAN_DATA_FRAME
				size = (uint8_t)(nb%(maxMsgSize+1)); // try 0 to 8 or 0 to 64 DLC size
				// Limit size to accepted FD CAN DATA FIELD size: 0-8, 12, 16, 20, 24, 32, 48 or 64 bytes
				size = FdcanLimitedDlcSize(size);
	
				if (brgStat == BRG_NO_ERR) {
					brgStat = FdcanMsgTxRxVerif(&fdcanTxMsg, dataTx, &fdcanRxMsg, dataRx, CAN_MSG_RX_FIFO0, size);
				}
				nb++;
			}
		}
	
		// disable used filter: std filter0 
		if (brgStat == BRG_NO_ERR) {
			brgStat = FdcanFilterDisable(&filterConf, 0, CAN_ID_STANDARD);
		}
	
		if (brgStat == BRG_NO_ERR) {
			brgStat =  m_pBrg->StopMsgReceptionFDCAN();
			if( brgStat != BRG_NO_ERR) {
				printf("FDCAN StopMsgReceptionFDCAN failed \n");
			}
		} else { // stop anyway
			m_pBrg->StopMsgReceptionFDCAN();
		}
	
		if (brgStat == BRG_NO_ERR) {
			printf(" FdcanLoopBack test OK \n");
		}
	
		return brgStat;
	}
	
	// init FDCAN message to default CAN or FDCAN configuration
	void cBrgExample::FdcanMsgTxInitCanStd(Brg_FdcanMsgT *pFdcanMsg)
	{
		pFdcanMsg->ID = 0;
		pFdcanMsg->IDE = CAN_ID_STANDARD;
		pFdcanMsg->RTR = CAN_DATA_FRAME;
		pFdcanMsg->ESI = FDCAN_ESI_ACTIVE; // no error
		pFdcanMsg->BRS = FDCAN_BRS_OFF;  // Bitrate Switching Off during data phase.
		pFdcanMsg->FDF = FDCAN_F_CLASSIC_CAN;  // Classic CAN.
		pFdcanMsg->DLC = 0;
	}
	void cBrgExample::FdcanMsgRxInitStd(Brg_FdcanRxMsgT *pFdcanMsg, Brg_FdcanFdfT frame)
	{
		FdcanMsgTxInitStd(&pFdcanMsg->Header, frame); //TxMsg and RxMsg header have same format
		pFdcanMsg->FilterNb = 0xFF; // ko value
		pFdcanMsg->Overrun = CAN_RX_NO_OVERRUN;
		pFdcanMsg->TimeStamp = 0; // unused
	}
	void cBrgExample::FdcanMsgTxInitStd(Brg_FdcanMsgT* pFdcanMsg, Brg_FdcanFdfT frame)
	{
		FdcanMsgTxInitCanStd(pFdcanMsg); // default classic CAN init
		if (frame == FDCAN_F_FD_CAN) {
			// then modify to activate FD CAN with BRS ON
			pFdcanMsg->BRS = FDCAN_BRS_ON;
			pFdcanMsg->FDF = FDCAN_F_FD_CAN;
		}
	}
	
	// FDCAN Disable Filter
	Brg_StatusT cBrgExample::FdcanFilterDisable(Brg_FdcanFilterConfT *pFilterConf, uint8_t filterNb, Brg_CanMsgIdT filterIde)
	{
		Brg_StatusT brgStat;
		pFilterConf->FilterNb = filterNb;
		pFilterConf->IDE = filterIde;
		pFilterConf->bIsFilterEn = false;
		// unused but must be valid value (ID 0 valid for both std and ext)
		pFilterConf->ID1 = 0;
		pFilterConf->ID2 = 0;
		pFilterConf->FilterMode = FDCAN_FILTER_ID_LIST; //FDCAN_FILTER_ID_RANGE  FDCAN_FILTER_ID_MASK FDCAN_FILTER_ID_LIST
		pFilterConf->bIsFilterReject = false; //Messages passing the filter are NOT rejected
		brgStat = m_pBrg->InitFilterFDCAN(pFilterConf);
		if( brgStat != BRG_NO_ERR ) {
			if (filterIde == CAN_ID_EXTENDED) {
				printf("FDCAN ext filter%d Deinit failed \n", (int)filterNb);
			} else {
				printf("FDCAN std filter%d Deinit failed \n", (int)filterNb);
			}
		}
		return brgStat;
	}
	
	uint8_t cBrgExample::FdcanLimitedDlcSize(uint8_t sizeInByte)
	{
		uint8_t limitedSize = sizeInByte;
		if (limitedSize > 8) { // full FDCAN case
			// Limit size to accepted FD CAN DATA FIELD size: 0-8, 12, 16, 20, 24, 32, 48 or 64 bytes
			if (limitedSize < 12) { // 9-11 -> 8
				limitedSize = 8;
			} else if (limitedSize < 16) { // 12-15 -> 12
				limitedSize = 12;
			} else if (limitedSize < 20) { // 16-19 -> 16
				limitedSize = 16;
			} else if (limitedSize < 24) { // 20-23 -> 20
				limitedSize = 20;
			} else if (limitedSize < 32) { // 24-31 -> 24
				limitedSize = 24;
			} else if (limitedSize < 48) { // 32-47 -> 32
				limitedSize = 32;
			} else if (limitedSize < 64) { // 48-63 -> 48
				limitedSize = 48;
			} else { // 48-63 -> 48
				limitedSize = 64;
			}
		}
		return limitedSize;
	}
	
	// send a message and verify it is received and that TX = Rx WriteMsgFDCAN GetRxMsgNbFDCAN GetRxMsgFDCAN
	Brg_StatusT cBrgExample::FdcanMsgTxRxVerif(Brg_FdcanMsgT *pFdcanTxMsg, BYTE *pDataTx, Brg_FdcanRxMsgT *pFdcanRxMsg, BYTE *pDataRx, Brg_CanRxFifoT rxFifo, BYTE size, BYTE expFilter)
	{
		Brg_StatusT brgStat = BRG_NO_ERR;
		uint16_t msgNb = 0;
		// Send message
		if( brgStat == BRG_NO_ERR ) {
			brgStat = m_pBrg->WriteMsgFDCAN(pFdcanTxMsg, pDataTx, size);
			if( brgStat != BRG_NO_ERR ) {
				printf("FDCAN Write Message error (Tx ID: 0x%08X)\n", (unsigned int)pFdcanTxMsg->ID);
			}
		}
		// Receive message
		if( brgStat == BRG_NO_ERR ) {
			uint16_t dataSize;
			int retry = 100;
			while( (retry > 0)&&(msgNb==0) ) {
				brgStat = m_pBrg->GetRxMsgNbFDCAN(&msgNb, rxFifo);
				retry --;
			}
			if( msgNb == 0 ) { // check if enough messages available
				brgStat = BRG_TARGET_CMD_TIMEOUT;
				printf("FDCAN Rx error (not enough msg available: 0/1)\n");
			}
			if( brgStat == BRG_NO_ERR ) { // read only 1 msg even if more available
				brgStat = m_pBrg->GetRxMsgFDCAN(pFdcanRxMsg, 1, pDataRx, 64, &dataSize, rxFifo);
			}
			if( brgStat != BRG_NO_ERR ) {
				printf("FDCAN Read Message error (Tx ID: 0x%08X, nb of Rx msg available in fifo %d: %d)\n", (unsigned int)pFdcanTxMsg->ID, (int)rxFifo, (int)msgNb);
			}
		}
		// verif Rx = Tx
		if( brgStat == BRG_NO_ERR ) {
			if( (pFdcanRxMsg->Header.ID != pFdcanTxMsg->ID) || (pFdcanRxMsg->Header.IDE != pFdcanTxMsg->IDE) || (pFdcanRxMsg->Header.DLC != size) ||
				(pFdcanRxMsg->Header.BRS != pFdcanTxMsg->BRS) || (pFdcanRxMsg->Header.FDF != pFdcanTxMsg->FDF) ||
				(pFdcanRxMsg->Header.ESI != FDCAN_ESI_ACTIVE) || (pFdcanRxMsg->Overrun != CAN_RX_NO_OVERRUN) ||
				((expFilter!=0xFF) && (pFdcanRxMsg->FilterNb!=expFilter))) {
				brgStat = BRG_CAN_ERR;
				printf("FDCAN ERROR ID Rx: 0x%08X Tx 0x%08X, IDE Rx %d Tx %d, DLC Rx %d size Tx %d\n", (unsigned int)pFdcanRxMsg->Header.ID, (unsigned int)pFdcanTxMsg->ID, (int)pFdcanRxMsg->Header.IDE, (int) pFdcanTxMsg->IDE, (int)pFdcanRxMsg->Header.DLC, (int)size);
			} else {
				for(int i=0; i<size; i++) {
					if( pDataRx[i] != pDataTx[i] ) {
						printf("FDCAN ERROR data[%d] Rx: 0x%02hX Tx 0x%02hX \n", (int)i, (unsigned char)pDataRx[i], (unsigned char)pDataTx[i]);
						brgStat = BRG_VERIF_ERR;
					}
				}
			}
			if( brgStat != BRG_NO_ERR ) {
				printf("FDCAN ERROR Read/Write verification \n");
			}
		}
		return brgStat;
	}


// EXAMPLE FOR FDCAN close, Brg::CloseBridge() 

    Brg *m_pBrg;
	**********[ Missing init steps ]**********
	Brg_StatusT brgStat = BRG_NO_ERR;

	brgStat = m_pBrg->CloseBridge(COM_FDCAN);
