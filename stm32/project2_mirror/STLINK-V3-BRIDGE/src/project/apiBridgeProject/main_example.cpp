/**
  ******************************************************************************
  * @file    main_example.cpp
  * @author  MCD Application Team
  * @brief   This module is an example, it can be integrated to a C++ console project
  *          to do a basic USB connection and  GPIO bridge test with the
  *          STLINK-V3SET probe, using the Bridge C++ open API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
#include "platform_include.h"
#if defined(_MSC_VER) &&  (_MSC_VER >= 1000)
// no additional include needed
#else
#include <cstdlib>
#endif
#include <stdio.h>
#include "bridge.h"
#include<tchar.h>

#define TEST_BUF_SIZE 3000

class cBrgExample
{
public:
	cBrgExample();
	~cBrgExample();

	Brg_StatusT SelectSTLink(STLinkInterface *pStlinkIf, int *pFirstDevNotInUse);
	Brg_StatusT Connect(Brg* pBrg, int deviceNb);
	void Disconnect(void);
	// CLK
	Brg_StatusT ComClkTest(void);
	// GPIO
	Brg_StatusT GpioTest(void);
	//SPI
	Brg_StatusT SpiTest(void);
	Brg_StatusT SpiTestInit(void);
	Brg_StatusT SpiTestTx(void);
	void InitTestBuffer(uint8_t initType, uint8_t* pBuff, uint16_t size);
	Brg_StatusT BrgRxTxVerifData(uint8_t* pRxBuff, uint8_t* pTxBuff, uint16_t size);
	// FDCAN
	Brg_StatusT FdcanTest(void);
	Brg_StatusT FdcanTestInit(void);
	Brg_StatusT FdcanTestLoopback(bool bIsFdcanClassicCan);
	void FdcanMsgTxInitCanStd(Brg_FdcanMsgT* pFdcanMsg);
	void FdcanMsgRxInitStd(Brg_FdcanRxMsgT* pFdcanMsg, Brg_FdcanFdfT frame);
	void FdcanMsgTxInitStd(Brg_FdcanMsgT* pFdcanMsg, Brg_FdcanFdfT frame);
	Brg_StatusT FdcanFilterDisable(Brg_FdcanFilterConfT* pFilterConf, uint8_t filterNb, Brg_CanMsgIdT filterIde);
	Brg_StatusT FdcanMsgTxRxVerif(Brg_FdcanMsgT *pFdcanTxMsg, uint8_t *pDataTx, Brg_FdcanRxMsgT *pFdcanRxMsg, uint8_t *pDataRx, Brg_CanRxFifoT rxFifo, uint8_t size, uint8_t expFilter=0xFF);
	uint8_t FdcanLimitedDlcSize(uint8_t sizeInByte);
	// CAN
	Brg_StatusT CanTest(void);
	Brg_StatusT CanTestInit(void);
	Brg_StatusT CanTestLoopback(void);
	Brg_StatusT CanFilterDisable(Brg_CanFilterConfT* pFilterConf, uint8_t filterNb, Brg_CanMsgIdT filterIde);
	Brg_StatusT CanMsgTxRxVerif(Brg_CanTxMsgT *pCanTxMsg, uint8_t *pDataTx, Brg_CanRxMsgT *pCanRxMsg, uint8_t *pDataRx, Brg_CanRxFifoT rxFifo, uint8_t size);

protected:
private:
	Brg* m_pBrg;
	char m_serialNumber[SERIAL_NUM_STR_MAX_LEN];
};

cBrgExample::cBrgExample() : m_pBrg(NULL)
{
	for (int i=0; i<SERIAL_NUM_STR_MAX_LEN; i++) {
		m_serialNumber[i] = 0;
	}
}

cBrgExample::~cBrgExample()
{
}

/*****************************************************************************/
// STLINK USB management
/*****************************************************************************/
Brg_StatusT cBrgExample::SelectSTLink(STLinkInterface* pStlinkIf, int* pFirstDevNotInUse)
{
	uint32_t i, numDevices;
	TDeviceInfo2 devInfo2;
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;
	STLink_EnumStlinkInterfaceT stlinkIfId;

	if ((pStlinkIf == NULL) || (pFirstDevNotInUse == NULL)) {
		printf("Internal parameter error in SelectSTLink\n");
		return BRG_PARAM_ERR;
	}
	stlinkIfId = pStlinkIf->GetIfId();
	if (stlinkIfId != STLINK_BRIDGE) {
		printf("Wrong interface in SelectSTLink\n");
		return BRG_PARAM_ERR;
	}

	ifStatus = pStlinkIf->EnumDevices(&numDevices, FALSE);
	// Choose the first STLink Bridge available
	if ((ifStatus == STLINKIF_NO_ERR) || (ifStatus == STLINKIF_PERMISSION_ERR)) {
		printf("%d BRIDGE device found\n", (int)numDevices);

		for( i=0; i<numDevices; i++ ) {
			ifStatus = pStlinkIf->GetDeviceInfo2(i, &devInfo2, sizeof(devInfo2));
			printf("Bridge %d PID: 0X%04hx SN:%s\n", (int)i, (unsigned short)devInfo2.ProductId, devInfo2.EnumUniqueId);

			if( (*pFirstDevNotInUse==-1) && (devInfo2.DeviceUsed == false) ) {
				*pFirstDevNotInUse = i;
				memcpy(m_serialNumber, &devInfo2.EnumUniqueId, SERIAL_NUM_STR_MAX_LEN);
				printf(" SELECTED BRIDGE Stlink SN:%s\n", m_serialNumber);
			}
		}
	} else if (ifStatus == STLINKIF_CONNECT_ERR) {
		printf("No STLink BRIDGE device detected\n");
	} else {
		printf("Enum error (status = %d)\n", ifStatus);
		if (ifStatus == STLINKIF_NO_STLINK) {
			printf("No BRIDGE STLink available\n");
		}
	}

	return Brg::ConvSTLinkIfToBrgStatus(ifStatus);
}

Brg_StatusT cBrgExample::Connect(Brg* pBrg, int deviceNb)
{
	// The firmware may not be the very last one, but it may be OK like that (just inform)
	bool bOldFirmwareWarning=false;
	Brg_StatusT brgStat = BRG_NO_ERR;
	if (pBrg == NULL) {
		return BRG_CONNECT_ERR;
	}
	m_pBrg = pBrg;
	// Open the STLink connection
	if (brgStat == BRG_NO_ERR) {
		m_pBrg->SetOpenModeExclusive(true);

		brgStat = m_pBrg->OpenStlink(deviceNb);

		if (brgStat == BRG_NOT_SUPPORTED) {
			printf("BRIDGE not supported SN:%s\n", m_serialNumber);
		}
		if (brgStat == BRG_OLD_FIRMWARE_WARNING) {
			// Status to restore at the end if all is OK
			bOldFirmwareWarning = true;
			brgStat = BRG_NO_ERR;
		}
	}

	// Test Voltage command
	if (brgStat == BRG_NO_ERR) {
		float voltage = 0;
		// T_VCC pin must be connected to target voltage on bridge or debug connector
		// T_VCC input is mandatory for STLink using levelshifter (STLINK-V3PWR or STLINK-V3SET+B-STLINK-VOLT/ISOL),
		// else bridge signals are all 0
		brgStat = m_pBrg->GetTargetVoltage(&voltage);
		if (brgStat != BRG_NO_ERR) {
			printf("BRIDGE get voltage error \n");
		} else if (voltage < 1) {
			printf("BRIDGE get voltage WARNING: %fV < 1V, check if T_VCC pin is connected to target voltage on bridge (or debug) connector \n", (double)voltage);
		} else {
			printf("BRIDGE get voltage: %f V \n", (double)voltage);
		}
	}

	if ((brgStat == BRG_NO_ERR) && (bOldFirmwareWarning == true)) {
		// brgStat = BRG_OLD_FIRMWARE_WARNING;
		printf("BRG_OLD_FIRMWARE_WARNING: v%d B%d \n",(int)m_pBrg->m_Version.Major_Ver, (int)m_pBrg->m_Version.Bridge_Ver);
	}

	return brgStat;

}

void cBrgExample::Disconnect(void)
{
	if (m_pBrg != NULL) {
		//robustness in case not already done
		m_pBrg->CloseBridge(COM_UNDEF_ALL);
		// break link to current STLink BRIDGE
		m_pBrg = NULL;
	}
}

/*****************************************************************************/
// Test GET CLOCK command
/*****************************************************************************/
Brg_StatusT cBrgExample::ComClkTest(void)
{
	Brg_StatusT brgStat = BRG_NO_ERR;
	uint32_t StlHClkKHz, comInputClkKHz;
	if (m_pBrg == NULL) {
		return BRG_CONNECT_ERR;
	}
	// Get the current bridge input Clk for all com:
	brgStat = m_pBrg->GetClk(COM_SPI, (uint32_t*)&comInputClkKHz, (uint32_t*)&StlHClkKHz);
	printf( "SPI input CLK: %d KHz, ST-Link HCLK: %d KHz \n", (int)comInputClkKHz, (int)StlHClkKHz);
	if (brgStat == BRG_NO_ERR) {
		brgStat = m_pBrg->GetClk(COM_I2C, (uint32_t*)&comInputClkKHz, (uint32_t*)&StlHClkKHz);
		printf( "I2C input CLK: %d KHz, ST-Link HCLK: %d KHz \n", (int)comInputClkKHz, (int)StlHClkKHz);
	}
	if (brgStat == BRG_NO_ERR) {
		brgStat = m_pBrg->GetClk(COM_CAN, (uint32_t*)&comInputClkKHz, (uint32_t*)&StlHClkKHz);
		printf( "CAN input CLK: %d KHz, ST-Link HCLK: %d KHz \n", (int)comInputClkKHz, (int)StlHClkKHz);
	}
	if (brgStat == BRG_NO_ERR) {
		brgStat = m_pBrg->GetClk(COM_GPIO, (uint32_t*)&comInputClkKHz, (uint32_t*)&StlHClkKHz);
		printf( "GPIO input CLK: %d KHz, ST-Link HCLK: %d KHz \n", (int)comInputClkKHz, (int)StlHClkKHz);
	}
	if (brgStat == BRG_NO_ERR) {
		brgStat = m_pBrg->GetClk(COM_FDCAN, (uint32_t*)&comInputClkKHz, (uint32_t*)&StlHClkKHz);
		if ((brgStat == BRG_CMD_NOT_SUPPORTED) && (m_pBrg->IsFdcanSupport()==false)) {
			// expected FDCAN not supported by all STLINK
			brgStat = BRG_NO_ERR;
		} else {
			printf( "FDCAN input CLK: %d KHz, ST-Link HCLK: %d KHz \n", (int)comInputClkKHz, (int)StlHClkKHz);
		}
	}
	if (brgStat != BRG_NO_ERR) {
		printf( "Error in GetClk()\n" );
	}
	return brgStat;
}

/*****************************************************************************/
// Test GPIO command
/*****************************************************************************/
Brg_StatusT cBrgExample::GpioTest(void)
{
	Brg_StatusT brgStat = BRG_NO_ERR;

	Brg_GpioInitT gpioParams;
	Brg_GpioConfT gpioConf[BRG_GPIO_MAX_NB];
	Brg_GpioValT gpioReadVal[BRG_GPIO_MAX_NB];
	uint8_t gpioMsk=0, gpioErrMsk;
	int i;
	uint8_t bridgeCom = COM_GPIO;

	printf("Run BRIDGE GPIO test\n");
	if (m_pBrg == NULL) {
		return BRG_CONNECT_ERR;
	}
	// Bridge GPIO are supposed to be not connected during this test

	// read test
	if( brgStat == BRG_NO_ERR ) {
		gpioMsk = BRG_GPIO_ALL;
		gpioParams.GpioMask = gpioMsk; // BRG_GPIO_0 1 2 3
		gpioParams.ConfigNb = BRG_GPIO_MAX_NB; //must be BRG_GPIO_MAX_NB or 1 (if 1 pGpioConf[0] used for all gpios)
		gpioParams.pGpioConf = &gpioConf[0];
		for(i=0; i<BRG_GPIO_MAX_NB; i++) {
			gpioConf[i].Mode = GPIO_MODE_INPUT;
			gpioConf[i].Speed = GPIO_SPEED_MEDIUM;
			gpioConf[i].Pull = GPIO_PULL_DOWN;
			gpioConf[i].OutputType = GPIO_OUTPUT_PUSHPULL; // unused in input mode
		}
		brgStat = m_pBrg->InitGPIO(&gpioParams);
		if( brgStat != BRG_NO_ERR ) {
			printf("Bridge Gpio init failed (mask=%d, gpio0: mode= %d, pull = %d, ...)\n",
				    (int)gpioParams.GpioMask, (int)gpioConf[0].Mode, (int)gpioConf[0].Pull);
		}
	}
	if( brgStat == BRG_NO_ERR ) {
		brgStat = m_pBrg->ReadGPIO(gpioMsk, &gpioReadVal[0], &gpioErrMsk);
		if( (brgStat != BRG_NO_ERR) || (gpioErrMsk!=0) ) {
			printf(" Bridge Read error\n");
		} else {
			// verify all gpio read to 0 (input pull down for STLINK without levelshifter,
			// or 0 forced by level shifter for STLINK with levelshifter)
			for(i=0; i<BRG_GPIO_MAX_NB; i++) {
				if( gpioReadVal[i] != GPIO_RESET ) {
					brgStat = BRG_VERIF_ERR;
					printf(" Bridge Read Verif error ( gpio %d != SET)\n", i);
				}
			}
		}
	}

	// Close Bridge COM_GPIO, even in case of error
	m_pBrg->CloseBridge(bridgeCom);

	if( brgStat == BRG_NO_ERR ) {
		printf("GPIO Test OK \n");
	}

	return brgStat;
}

/*****************************************************************************/
// Test SPI commands
/*****************************************************************************/
Brg_StatusT cBrgExample::SpiTest(void)
{
	Brg_StatusT brgStat = BRG_NO_ERR;
	uint8_t bridgeCom = COM_SPI;

	if (m_pBrg == NULL) {
		return BRG_CONNECT_ERR;
	}

	printf("Run BRIDGE SPI test\n");
	brgStat = SpiTestInit();
	if( brgStat != BRG_NO_ERR ) {
		printf("SPI init error \n");
	}
	if (brgStat == BRG_NO_ERR) {
		brgStat = SpiTestTx();
	}

	// Close Bridge SPI COM, even in case of error
	m_pBrg->CloseBridge(bridgeCom);

	if( brgStat == BRG_NO_ERR ) {
		printf("SPI Test OK \n");
	}

	return brgStat;
}

// Test SPI commands Brg::InitSPI Brg::GetSPIbaudratePrescal Brg::SetSPIpinCS
Brg_StatusT cBrgExample::SpiTestInit(void)
{
	Brg_StatusT brgStat = BRG_NO_ERR;
	Brg_SpiInitT spiParam;
	Brg_SpiBaudrateT prescal;
	uint32_t reqFreqKHz = 6000; //6MHz
	uint32_t currFreqKHz = 0;
	bool bNssHigh = false;

	brgStat = m_pBrg->GetSPIbaudratePrescal(reqFreqKHz, &prescal, &currFreqKHz);
	if( brgStat == BRG_COM_FREQ_MODIFIED ) {
		brgStat = BRG_NO_ERR;
		printf("WARNING Bridge SPI init freq asked %d KHz but applied %d KHz \n", (int)reqFreqKHz, (int)currFreqKHz);
	}

	if( brgStat == BRG_NO_ERR ) {
		spiParam.Baudrate = prescal;
		spiParam.Cpha = SPI_CPHA_1EDGE;
		spiParam.Cpol = SPI_CPOL_LOW;
		spiParam.Crc = SPI_CRC_DISABLE;
		spiParam.CrcPoly = 0;
		spiParam.DataSize = SPI_DATASIZE_8B;
		spiParam.Direction = SPI_DIRECTION_2LINES_FULLDUPLEX;
		spiParam.FirstBit = SPI_FIRSTBIT_MSB;
		spiParam.FrameFormat = SPI_FRF_MOTOROLA;
		spiParam.Mode = SPI_MODE_MASTER;
		spiParam.Nss = SPI_NSS_SOFT;
		spiParam.NssPulse = SPI_NSS_NO_PULSE;
		spiParam.SpiDelay = DEFAULT_NO_DELAY;
		brgStat = m_pBrg->InitSPI(&spiParam);
		if( (brgStat == BRG_NO_ERR) && (spiParam.Nss==SPI_NSS_SOFT) ) {
			if( bNssHigh == true ) {
				brgStat = m_pBrg->SetSPIpinCS(SPI_NSS_HIGH);
				// for SPI slave detecting a falling edge of NSS; set it to 1 before SPI transfer
			} else {
				brgStat = m_pBrg->SetSPIpinCS(SPI_NSS_LOW);
			}
		}
	}
	return brgStat;
}

// Test SPI
Brg_StatusT cBrgExample::SpiTestTx(void)
{
	Brg_StatusT brgStat = BRG_NO_ERR;
	uint8_t dataRx[TEST_BUF_SIZE], dataTx[TEST_BUF_SIZE]; //max size must be aligned with target buffer
	uint16_t size;

	InitTestBuffer(0, dataRx, TEST_BUF_SIZE);
	InitTestBuffer(0, dataTx, TEST_BUF_SIZE);
	// test 2048 bytes
	if( brgStat == BRG_NO_ERR ) {
		size = 2048;
		if (size <= TEST_BUF_SIZE) {
			InitTestBuffer(1, dataTx, size);
			brgStat = BrgRxTxVerifData(dataRx, dataTx, size);
		}
	}
	return brgStat;
}

void cBrgExample::InitTestBuffer(uint8_t initType, uint8_t*pBuff, uint16_t size)
{
	int i;
	if (size <= TEST_BUF_SIZE) {
		switch(initType) {
		case 0:
			memset(pBuff, 0, size);
			break;
		default:
			for(i=0; i<size; i++) {
				pBuff[i] = (uint8_t)i%256;
			}
			break;
		}
	}
}

// activate TEST_SPI_RX define for a full SPI loopback tests with data check:
//  need to connect to a SPI slave that will answer this protocol:
// SPI master (STLINK):
// Send a 4 bytes size, receive it back, verify
// Send size bytes data, receive it back, verify
// Test SPI Brg::WriteSPI Brg::ReadSPI
Brg_StatusT cBrgExample::BrgRxTxVerifData(uint8_t*pRxBuff, uint8_t*pTxBuff, uint16_t size)
{
	Brg_StatusT brgStat = BRG_NO_ERR;
	int i, nb;
	uint16_t sizeWithoutErr =0;
	uint32_t txSize,rxSize=0;

	txSize = (uint32_t) size;

	// 1- send 4 bytes = data size,
	brgStat = m_pBrg->WriteSPI((uint8_t*)&txSize, 4, &sizeWithoutErr);
	if( brgStat != BRG_NO_ERR ) {
		printf("SPI Write data size error (sent %d instead of 4)\n", (int)sizeWithoutErr);
	}
	// 2- wait to receive it back
	if( brgStat == BRG_NO_ERR ) {
		sizeWithoutErr = 0;
		brgStat = m_pBrg->ReadSPI((uint8_t*)&rxSize, 4, &sizeWithoutErr);
		if( brgStat != BRG_NO_ERR ) {
			printf("BRG Read back data size error (read %d instead of 4)\n", (int)sizeWithoutErr);
		} else {
#ifdef TEST_SPI_RX //not activated as no target connected for basic test
			if( rxSize != txSize ) {
				brgStat = BRG_VERIF_ERR; not activated as no target connected to answer
				printf("SPI Read back RxSize = %d different from TxSize = %d \n", (int)rxSize, (int)txSize);
			}
#endif
		}
	}
	// 3- send data size bytes
	if (brgStat == BRG_NO_ERR) {
		sizeWithoutErr = 0;
		brgStat = m_pBrg->WriteSPI(pTxBuff, size, &sizeWithoutErr);
		if( brgStat != BRG_NO_ERR ) {
			printf("SPI Write data error (sent %d instead of %d)\n", (int)sizeWithoutErr, (int)size);
		}
	}
	// 4- wait to receive same data size bytes back.
	if (brgStat == BRG_NO_ERR) {
		sizeWithoutErr = 0;
		brgStat = m_pBrg->ReadSPI(pRxBuff, size, &sizeWithoutErr);
		if( brgStat != BRG_NO_ERR ) {
			printf("SPI Read back data error (read %d instead of %d)\n", (int)sizeWithoutErr, (int)size);
		} else {
#ifdef TEST_SPI_RX //not activated as no target connected for basic test
			if (memcmp(pRxBuff, pTxBuff, size) !=0) {
				brgStat = BRG_VERIF_ERR;
				nb=0;
				for( i=0; i<size; i++ ) {
					if( (pRxBuff[i] != pTxBuff[i]) ) { 
						if(nb==0) {
							printf("SPI ERROR data[%d] Rx: 0x%02hX Tx 0x%02hX \n", (int)i, (unsigned short)(unsigned char)pRxBuff[i], (unsigned short)(unsigned char)pTxBuff[i]);
						} else if(nb<4) {// print max 4 errors
							printf("SPI ERROR data[%d] Rx: 0x%02hX Tx 0x%02hX \n", (int)i, (unsigned short)(unsigned char)pRxBuff[i], (unsigned short)(unsigned char)pTxBuff[i]);
						}
						nb++;
					}
				}
				printf("SPI ERROR Read/Write verification %d error(s) for size %d bytes \n", (int)nb, (int)size);
			}
#endif
		}
	}
	return brgStat;
}

/*****************************************************************************/
// Test FDCAN commands
/*****************************************************************************/
Brg_StatusT cBrgExample::FdcanTest(void)
{
	Brg_StatusT brgStat = BRG_NO_ERR;
	uint8_t bridgeCom = COM_FDCAN;

	if (m_pBrg == NULL) {
		return BRG_CONNECT_ERR;
	}
	if (m_pBrg->IsFdcanSupport() == false) {
		printf("WARNING Bridge FDCAN test skipped (not supported by current STLINK) \n");
		return BRG_NO_ERR;
	}

	printf("Run BRIDGE FDCAN test\n");
	brgStat = FdcanTestInit();
	if( brgStat != BRG_NO_ERR ) {
		printf("FDCAN init error \n");
	}
	if( brgStat == BRG_NO_ERR ) {
		// bIsFdcanClassicCan = false full FDCAN test
		brgStat = FdcanTestLoopback(false);
	}
	if( brgStat == BRG_NO_ERR ) {
		// bIsFdcanClassicCan = true for classical CAN test
		brgStat = FdcanTestLoopback(true);
	}

	// Close Bridge FDCAN COM, even in case of error
	m_pBrg->CloseBridge(COM_FDCAN);

	if( brgStat == BRG_NO_ERR ) {
		printf("FDCAN Test OK \n");
	}

	return brgStat;
}

// Test FDCAN commands Brg::InitFDCAN Brg::GetFDCANbaudratePrescal
Brg_StatusT cBrgExample::FdcanTestInit(void)
{
	Brg_StatusT brgStat = BRG_NO_ERR;
	Brg_FdcanInitT fdcanParam;
	uint32_t nomPrescal=1, dataPrescal=1;
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

// Test FDCAN commands Brg::StartMsgReceptionFDCAN Brg::InitFilterFDCAN
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
				dataTx[i] = (uint8_t)(nb+i);
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
Brg_StatusT cBrgExample::FdcanMsgTxRxVerif(Brg_FdcanMsgT *pFdcanTxMsg, uint8_t *pDataTx, Brg_FdcanRxMsgT *pFdcanRxMsg, uint8_t *pDataRx, Brg_CanRxFifoT rxFifo, uint8_t size, uint8_t expFilter)
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

/*****************************************************************************/
// Test CAN commands
/*****************************************************************************/
Brg_StatusT cBrgExample::CanTest(void)
{
	Brg_StatusT brgStat = BRG_NO_ERR;
	uint8_t bridgeCom = COM_CAN;

	if (m_pBrg == NULL) {
		return BRG_CONNECT_ERR;
	}

	printf("Run BRIDGE CAN test\n");
	brgStat = CanTestInit();
	if( brgStat != BRG_NO_ERR ) {
		printf("CAN init error \n");
	}
	if (brgStat == BRG_NO_ERR) {
		brgStat = CanTestLoopback();
	} else if ((brgStat == BRG_CAN_ERR) && (m_pBrg->IsCanFilter16Support() == true)) {
		// STLINK-V3SET case: loopback requires an external CAN bus
		printf("CAN Loopback test Skipped\n STLINK-V3SET requires to be connected to a CAN bus (e.g.: V3SET ADAPTER board with CAN on) \n");
		brgStat = BRG_NO_ERR;
	}

	// Close Bridge CAN COM, even in case of error
	m_pBrg->CloseBridge(COM_CAN);

	if( brgStat == BRG_NO_ERR ) {
		printf("CAN Test OK \n");
	}

	return brgStat;
}

// Test CAN commands Brg::InitCAN Brg::GetCANbaudratePrescal
Brg_StatusT cBrgExample::CanTestInit(void)
{
	Brg_StatusT brgStat = BRG_NO_ERR;
	uint32_t prescal;
	uint32_t reqBaudrate = 125000; //125kbps
	uint32_t finalBaudrate = 0;
	Brg_CanInitT canParam;

	// Set baudrate to 125kbps
	// with default CAN bit Time: PropSegInTq = 1, PhaseSeg1InTq = 7, PhaseSeg2InTq = 7 SjwInTq=4
	// N=sync+prop+seg1+seg2= 1+1+7+7= 16, 125000 bps
	// -> prescal = 24 = (CanClk = 48MHz)/(16*125000) for STLINK-V3SET
	// -> prescal = 40 = (CanClk = 80MHz)/(16*125000) for STLINK-V3PWR
	canParam.BitTimeConf.PropSegInTq = 1;
	canParam.BitTimeConf.PhaseSeg1InTq = 7;
	canParam.BitTimeConf.PhaseSeg2InTq = 7;
	canParam.BitTimeConf.SjwInTq = 4; //min (4, PhaseSeg1InTq)

	brgStat = m_pBrg->GetCANbaudratePrescal(&canParam.BitTimeConf, reqBaudrate, (uint32_t*)&prescal, (uint32_t*)&finalBaudrate);
	if( brgStat == BRG_COM_FREQ_MODIFIED ) {
		brgStat = BRG_NO_ERR;
		printf("WARNING Bridge CAN init baudrate asked %d bps but applied %d bps \n", (int)reqBaudrate, (int)finalBaudrate);
	}

	if( brgStat == BRG_NO_ERR ) {
		canParam.Prescaler = prescal;
		// canParam.BitTimeConf already filled above
		canParam.Mode = CAN_MODE_LOOPBACK; // CAN_MODE_LOOPBACK (for internal CAN STLINK test) CAN_MODE_NORMAL for loopback with real target
		canParam.bIsTxfpEn = false;
		canParam.bIsRflmEn = false;
		canParam.bIsNartEn = false;
		canParam.bIsAwumEn = false;
		canParam.bIsAbomEn = false;
		brgStat = m_pBrg->InitCAN(&canParam, BRG_INIT_FULL);
	} else if( brgStat == BRG_COM_FREQ_NOT_SUPPORTED ) {
		printf("ERROR Bridge CAN init baudrate %d bps not possible (invalid prescaler: %d) change Bit Time or baudrate settings. \n", (int)reqBaudrate, (int)prescal);
	}
	return brgStat;
}

// Test CAN commands Brg::StartMsgReceptionCAN Brg::InitFilterCAN
Brg_StatusT cBrgExample::CanTestLoopback(void)
{
	Brg_StatusT brgStat = BRG_NO_ERR;
	uint8_t dataRx[8], dataTx[8]; // Classic CAN: DLC limited to 8
	int i, nb;
	Brg_CanFilterConfT filterConf;
	Brg_CanRxMsgT canRxMsg;
	Brg_CanTxMsgT canTxMsg;
	uint8_t size=0;
	uint8_t maxMsgSize = 8;

	brgStat =  m_pBrg->StartMsgReceptionCAN();
	if (brgStat != BRG_NO_ERR) {
		printf("CAN StartMsgReceptionCAN failed \n");
	}

	// robustess test
	if (brgStat == BRG_NO_ERR) {
		int maxLoop;
		maxLoop = 500;

		// Receive messages with specific ID with all Classic Standard CAN DLC possible size (0->8)
		// Filter0: CAN prepare receive (no filter: ID_MASK with Id =0 & Mask = 0) receive all in FIFO0
		filterConf.AssignedFifo = CAN_MSG_RX_FIFO0;
		filterConf.bIsFilterEn = true;
		filterConf.FilterBankNb = 0; //0 to 13
		filterConf.FilterMode = CAN_FILTER_ID_MASK;
		filterConf.FilterScale = CAN_FILTER_32BIT; // note: STLINK-V3PWR does not support CAN_FILTER_16BIT
		for( i = 0; i<4; i++) {
			filterConf.Id[i].ID = 0;
			filterConf.Id[i].IDE = CAN_ID_STANDARD;//CAN_ID_EXTENDED;
			filterConf.Id[i].RTR = CAN_DATA_FRAME;
		}
		for( i = 0; i<2; i++) {
			filterConf.Mask[i].ID = 0;
			filterConf.Mask[i].IDE = CAN_ID_STANDARD;//CAN_ID_EXTENDED;
			filterConf.Mask[i].RTR = CAN_DATA_FRAME;
		}

		brgStat = m_pBrg->InitFilterCAN(&filterConf);
		if (brgStat != BRG_NO_ERR) {
			printf("CAN filter0 init failed \n");
		}

		//Init Rx / Tx msg
		canRxMsg.ID = 0;
		canRxMsg.IDE = CAN_ID_EXTENDED; // must be = canTxMsg.IDE for the test
		canRxMsg.RTR = CAN_DATA_FRAME; // must be = canTxMsg.RTR for the test
		canRxMsg.DLC = 0;

		canTxMsg.ID = 0x678;//0x12345678; // must be <=0x7FF for CAN_ID_STANDARD, <=0x1FFFFFFF
		canTxMsg.IDE = CAN_ID_STANDARD;//CAN_ID_EXTENDED;, for STLINK-V3PWR must be the same as filter configuration
		canTxMsg.RTR = CAN_DATA_FRAME;
		canTxMsg.DLC = 0;

		nb = 0;
		while ( (brgStat == BRG_NO_ERR) && (nb<maxLoop) ) {
			for (i=0; i<maxMsgSize; i++) {
				dataRx[i] = 0;
				dataTx[i] = (uint8_t)(nb+i);
			}
			canRxMsg.DLC = 0;
			canTxMsg.DLC = 2; // unused in CAN_DATA_FRAME
			size = (uint8_t)(nb%(maxMsgSize+1)); // try 0 to 8

			if (brgStat == BRG_NO_ERR) {
				brgStat = CanMsgTxRxVerif(&canTxMsg, dataTx, &canRxMsg, dataRx, CAN_MSG_RX_FIFO0, size);
			}
			nb++;
		}
	}

	// disable used filter: std filter0
	if (brgStat == BRG_NO_ERR) {
		brgStat = CanFilterDisable(&filterConf, 0, CAN_ID_STANDARD);
	}

	if (brgStat == BRG_NO_ERR) {
		brgStat =  m_pBrg->StopMsgReceptionCAN();
		if( brgStat != BRG_NO_ERR) {
			printf("CAN StopMsgReceptionCAN failed \n");
		}
	} else { // stop anyway
		m_pBrg->StopMsgReceptionCAN();
	}

	if (brgStat == BRG_NO_ERR) {
		printf(" CanLoopBack test OK \n");
	}

	return brgStat;
}

Brg_StatusT cBrgExample::CanFilterDisable(Brg_CanFilterConfT* pFilterConf, uint8_t filterNb, Brg_CanMsgIdT filterIde)
{
	Brg_StatusT brgStat;
	int i;
	pFilterConf->FilterBankNb = filterNb;
	pFilterConf->bIsFilterEn = false;
	// ID unused but must be valid value (ID 0 valid for both std and ext)
	for (i = 0; i<4; i++) {
		pFilterConf->Id[i].ID = 0;
		pFilterConf->Id[i].IDE = filterIde; //must match enabled filter for STLINK-V3PWR
		pFilterConf->Id[i].RTR = CAN_DATA_FRAME;
	}
	for (i = 0; i<2; i++) { //unused
		pFilterConf->Mask[i].ID = 0;
		pFilterConf->Mask[i].IDE = CAN_ID_STANDARD;
		pFilterConf->Mask[i].RTR = CAN_DATA_FRAME;
	}
	//unused when disabling
	pFilterConf->FilterMode = CAN_FILTER_ID_LIST;
	pFilterConf->FilterScale = CAN_FILTER_32BIT;
	pFilterConf->AssignedFifo = CAN_MSG_RX_FIFO0;

	brgStat = m_pBrg->InitFilterCAN(pFilterConf);
	if( brgStat != BRG_NO_ERR ) {
		if (filterIde == CAN_ID_EXTENDED) {
			printf("CAN ext filter%d Deinit failed \n", (int)filterNb);
		} else {
			printf("CAN std filter%d Deinit failed \n", (int)filterNb);
		}
	}
	return brgStat;
}

// send a message and verify it is received and that TX = Rx, Test CAN commands Brg::WriteMsgCAN Brg::GetRxMsgNbCAN Brg::GetRxMsgCAN
Brg_StatusT cBrgExample::CanMsgTxRxVerif(Brg_CanTxMsgT *pCanTxMsg, uint8_t *pDataTx, Brg_CanRxMsgT *pCanRxMsg, uint8_t *pDataRx, Brg_CanRxFifoT rxFifo, uint8_t size)
{
	Brg_StatusT brgStat = BRG_NO_ERR;
	uint16_t msgNb = 0;
	// Send message
	if( brgStat == BRG_NO_ERR ) {
		brgStat = m_pBrg->WriteMsgCAN(pCanTxMsg, pDataTx, size);
		if( brgStat != BRG_NO_ERR ) {
			printf("CAN Write Message error (Tx ID: 0x%08X)\n", (unsigned int)pCanTxMsg->ID);
		}
	}
	// Receive message
	if( brgStat == BRG_NO_ERR ) {
		uint16_t dataSize;
		int retry = 100;
		while( (retry > 0)&&(msgNb==0) ) {
			brgStat = m_pBrg->GetRxMsgNbCAN(&msgNb);
			retry --;
		}
		if( msgNb == 0 ) { // check if enough messages available
			brgStat = BRG_TARGET_CMD_TIMEOUT;
			printf("CAN Rx error (not enough msg available: 0/1)\n");
		}
		if( brgStat == BRG_NO_ERR ) { // read only 1 msg even if more available
			brgStat = m_pBrg->GetRxMsgCAN(pCanRxMsg, 1, pDataRx, 8, &dataSize);
		}
		if( brgStat != BRG_NO_ERR ) {
			printf("CAN Read Message error (Tx ID: 0x%08X, nb of Rx msg available: %d)\n", (unsigned int)pCanTxMsg->ID, (int)msgNb);
		} else {
			if( pCanRxMsg->Fifo != rxFifo ) {
				printf("CAN Read Message FIFO error (Tx ID: 0x%08X in FIFO%d instead of %d)\n", (unsigned int)pCanTxMsg->ID, (int)pCanRxMsg->Fifo, (int)rxFifo);
				brgStat = BRG_VERIF_ERR;
			}
		}
	}
	// verif Rx = Tx
	if( brgStat == BRG_NO_ERR ) {
		if( (pCanRxMsg->ID != pCanTxMsg->ID) || (pCanRxMsg->IDE != pCanTxMsg->IDE) || (pCanRxMsg->DLC != size) ||
			(pCanRxMsg->Overrun != CAN_RX_NO_OVERRUN) ) {
			brgStat = BRG_CAN_ERR;
			printf("CAN ERROR ID Rx: 0x%08X Tx 0x%08X, IDE Rx %d Tx %d, DLC Rx %d size Tx %d\n", (unsigned int)pCanRxMsg->ID, (unsigned int)pCanTxMsg->ID, (int)pCanRxMsg->IDE, (int) pCanTxMsg->IDE, (int)pCanRxMsg->DLC, (int)size);
		} else {
			for(int i=0; i<size; i++) {
				if( pDataRx[i] != pDataTx[i] ) {
					printf("CAN ERROR data[%d] Rx: 0x%02hX Tx 0x%02hX \n", (int)i, (unsigned short)(unsigned char)pDataRx[i], (unsigned short)(unsigned char)pDataTx[i]);
					brgStat = BRG_VERIF_ERR;
				}
			}
		}
		if( brgStat != BRG_NO_ERR ) {
			printf("CAN ERROR Read/Write verification \n");
		}
	}
	return brgStat;
}
/*****************************************************************************/
// Test SPI commands
/*****************************************************************************/

/*****************************************************************************/
// Main example
/*****************************************************************************/
// main() Defines the entry point for the console application.
#ifdef WIN32 //Defined for applications for Win32 and Win64.
int _tmain(int argc, _TCHAR* argv[])

#else
using namespace std;

int main(int argc, char** argv)
#endif
{
	cBrgExample brgTest;
	// To disable a test set below variable to false
	bool bRunClkTest = true;
	bool bRunGpioTest = true;
	bool bRunSpiTest = true;
	bool bRunFdcanTest = true;
	bool bRunCanTest = true;

	Brg_StatusT brgStat = BRG_NO_ERR;
    STLinkIf_StatusT ifStat = STLINKIF_NO_ERR;
	TCHAR path[MAX_PATH];
#ifdef WIN32 //Defined for applications for Win32 and Win64.
	TCHAR *pEndOfPrefix;
#endif
	int firstDevNotInUse=-1;
	Brg* pBrg = NULL;
	STLinkInterface *m_pStlinkIf = NULL;

	// Note: cErrLog g_ErrLog; to be instanciated and initialized if used with USING_ERRORLOG

	// In case previously used, close the previous connection (not the case here)
	if (pBrg!=NULL) {
		pBrg->CloseBridge(COM_UNDEF_ALL);
		pBrg->CloseStlink();
		delete pBrg;
		pBrg = NULL;
	}
	if (m_pStlinkIf !=NULL) {// never delete STLinkInterface before Brg that is using it.
		delete m_pStlinkIf;
		m_pStlinkIf = NULL;
	}

	// USB interface initialization and device detection done using STLinkInterface

	// Create USB BRIDGE interface
	m_pStlinkIf = new STLinkInterface(STLINK_BRIDGE);
#ifdef USING_ERRORLOG
	m_pStlinkIf->BindErrLog(&g_ErrLog);
#endif

#ifdef WIN32 //Defined for applications for Win32 and Win64.
	GetModuleFileName(NULL, path, MAX_PATH); //may require shlwapi library in "Additionnal Dependencies Input" linker settings
	// Remove process file name from the path
	pEndOfPrefix = _tcsrchr(path,'\\');

	if (pEndOfPrefix != NULL)
	{
		*(pEndOfPrefix + 1) = '\0';
	}
#else
	strcpy(path, "");
#endif
	// Load STLinkUSBDriver library
	// In this example STLinkUSBdriver (dll on windows) must be copied near test executable
	// Copy last STLinkUSBDriver dll from  STSW-LINK007 package (STLINK firmware upgrade application),
	// choose correct library according to your project architecture
	// e.g.: Debug x64, stsw-link007\AllPlatforms\native\win_x64 to be copied in apiBridgeProject\Visual2019\x64\Debug
	ifStat = m_pStlinkIf->LoadStlinkLibrary(path);
	if( ifStat!=STLINKIF_NO_ERR ) {
		printf("STLinkUSBDriver library (dll) issue \n");
	}

	// Enumerate the STLink Bridge instance, and choose the first one in the list
	brgStat = brgTest.SelectSTLink(m_pStlinkIf, &firstDevNotInUse);

	// USB Connection to a given device done with Brg
	if (brgStat == BRG_NO_ERR) {
		pBrg = new Brg(*m_pStlinkIf);
#ifdef USING_ERRORLOG
		m_pBrg->BindErrLog(&g_ErrLog);
#endif
	}
	// Connect to the selected STLink
	if (brgStat == BRG_NO_ERR) {
		brgStat = brgTest.Connect(pBrg, firstDevNotInUse);
	}

	// below tests are basic examples that do not require connection to a target
	// (internal loopback)

	// Test GET CLOCK command
	if ((bRunClkTest == true) && (brgStat == BRG_NO_ERR)) {
		brgStat = brgTest.ComClkTest();
	}

	// Test GPIO
	if ((bRunGpioTest == true) && (brgStat == BRG_NO_ERR)) {
		brgStat = brgTest.GpioTest();
	}

	// Test SPI
	if ((bRunSpiTest == true) && (brgStat == BRG_NO_ERR)) {
		brgStat = brgTest.SpiTest();
	}

	// Test FDCAN
	if ((bRunFdcanTest == true) && (brgStat == BRG_NO_ERR)) {
		brgStat = brgTest.FdcanTest();
	}

	// Test CAN
	if ((bRunCanTest == true) && (brgStat == BRG_NO_ERR)) {
		brgStat = brgTest.CanTest();
	}

	// test disconnect
	brgTest.Disconnect();

	// STLink Disconnect
	if (pBrg!=NULL) {
		pBrg->CloseBridge(COM_UNDEF_ALL);
		pBrg->CloseStlink();
		delete pBrg;
		pBrg = NULL;
	}
	// unload STLinkUSBdriver library
	if (m_pStlinkIf!=NULL) {
		// always delete STLinkInterface after Brg (because Brg uses STLinkInterface)
		delete m_pStlinkIf;
		m_pStlinkIf = NULL;
	}

	if (brgStat == BRG_NO_ERR) 	{
		printf("TEST SUCCESS \n");
	} else {
		printf("TEST FAIL (Bridge error: %d) \n", (int)brgStat);
	}

	return 0;
}

