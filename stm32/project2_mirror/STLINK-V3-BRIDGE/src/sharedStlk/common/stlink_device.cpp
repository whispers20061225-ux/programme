/**
  ******************************************************************************
  * @file    stlink_device.cpp
  * @author  GPM-AppliTools-HWBoards Team
  * @brief   StlinkDevice parent class used for STLink device common functionalities.
  *          May be used to retrieve information about ST-Link firmware
  *          independantly from the child instance.
  ******************************************************************************
  * @attention
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "platform_include.h"
#include "stlink_device.h"

/* Private typedef -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/

/* Class Functions Definition ------------------------------------------------*/

/*
 * StlinkDevice constructor
 */
StlinkDevice::StlinkDevice(STLinkInterface &StlinkIf): m_bStlinkConnected(false), m_pStlinkInterface(&StlinkIf),
	m_handle(NULL), m_deviceIdTcp(0), m_bOpenExclusive(false)
{
	m_Version.Major_Ver = 0;
	m_Version.Jtag_Ver = 0;
	m_Version.Swim_Ver = 0;
	m_Version.Msc_Ver = 0;
	m_Version.Bridge_Ver = 0;
	m_Version.Power_Ver = 0;
	m_Version.VID = 0;
	m_Version.PID = 0;
	m_Version.Res1_Ver = 0;
	m_Version.Res2_Ver = 0;

#ifdef USING_ERRORLOG
	// Error log management
	m_pErrLog = NULL;
#endif
}
/*
 * StlinkDevice destructor
 */
StlinkDevice::~StlinkDevice(void)
{
	// Close STLink even if failure
	CloseStlink();

#ifdef WIN32 //Defined for applications for Win32 and Win64.
#ifdef USING_ERRORLOG
	// Flush the trace log system after closing
	if( m_pErrLog != NULL ) {
		m_pErrLog->Dump();
	}
#endif
#endif // WIN32
}

/*
 * Trace logging mechanism, under compilation switch USING_ERRORLOG (requiring ErrLog.h and ErrLog.cpp)
 */
void StlinkDevice::LogTrace(const char *pMessage, ...) const
{
#ifdef USING_ERRORLOG
	va_list args; // used to manage the variable argument list
	va_start(args, pMessage);
	if( m_pErrLog != NULL ) {
		m_pErrLog->LogTrace(pMessage, args);
	}
	va_end(args);
#endif
}

/**
 * @ingroup DEVICE
 * @brief Mode for open USB connection. Used by OpenStlink().
 * @param[in] bExclusive  false: shared between applications \n
 *                        true: exclusive to 1 application: recommended for Bridge
 */
void StlinkDevice::SetOpenModeExclusive(bool bExclusive)
{
	m_bOpenExclusive = bExclusive;
}
/*
 * @brief Open current USB connection with the STLink.
 * If not already done, STLinkInterface::OpenDevice will build the device list before opening.
 *
 * @param[in]  StlinkInstId  Instance ID in the list of enumerated STLink devices.
 *                           Create one instance of StlinkDevice by device.
 * @param[in]  StlinkIdTcp   Instance ID of the STLink device, for use in shared mode (ignored in direct mode), equal to #STLink_DeviceInfo2T.StLinkUsbId.
 * @return STLinkInterface::OpenDevice() errors
 * @retval #STLINKIF_CONNECT_ERR Wrong StlinkInstId or USB error
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT StlinkDevice::OpenStlink(int StlinkInstId, uint32_t StlinkIdTcp)
{
	STLinkIf_StatusT ifStatus=STLINKIF_NO_ERR;

	if( m_pStlinkInterface == NULL ) {
		return STLINKIF_DLL_ERR;
	}

	if( (m_pStlinkInterface->GetIfId() == STLINK_COM_PORT_DBG) || (m_pStlinkInterface->GetIfId() == STLINK_COM_PORT_PWR) ) {
		// The class is not adapted for COM port interfaces
		return STLINKIF_NOT_SUPPORTED;
	}

	if( m_bStlinkConnected == false ) {
		// Open the device
		ifStatus = m_pStlinkInterface->OpenDevice(StlinkInstId, StlinkIdTcp, m_bOpenExclusive, &m_handle);
		if( ifStatus != STLINKIF_NO_ERR ) {
			LogTrace("%s STLink device USB connection failure", m_pStlinkInterface->GetIfLogString());
			return STLINKIF_CONNECT_ERR;
		}
		if (m_pStlinkInterface->GetIfId() == STLINK_TCP ) {
			// Store TCP server cookie to communicate with the opened device
			m_deviceIdTcp = StlinkIdTcp;
		}
		m_bStlinkConnected = true;
		if( (m_pStlinkInterface->GetIfId() == STLINK_DBG_INTERFACE) || (m_pStlinkInterface->GetIfId() == STLINK_TCP) ) {
			// on BRIDGE only GetVersionExt supported
			ifStatus = StGetVersion(&m_Version);
			if (ifStatus != STLINKIF_NO_ERR) {
				LogTrace("STLink get version failure");
				CloseStlink();
				return ifStatus;
			}
		}
		// Get and keep the serial number for possible renumeration later
		ifStatus = GetSerialNumFromSystem(StlinkInstId);
		if( ifStatus != STLINKIF_NO_ERR ) {
			LogTrace("ST-Link get serial num failure");
			m_pStlinkInterface->CloseDevice(&m_handle, StlinkIdTcp);
			return ifStatus;
		}

		// Retrieve the real version for ST-Link/V3 and higher (on BRIDGE or debug interfaces)
		if ((m_pStlinkInterface->GetIfId() == STLINK_BRIDGE) ||
		    (((m_pStlinkInterface->GetIfId() == STLINK_DBG_INTERFACE) || (m_pStlinkInterface->GetIfId() == STLINK_TCP)) &&
		    ((m_Version.Major_Ver >= FIRMWARE_MIN_MAJOR_VER_STLINKV3) && (m_Version.Swim_Ver == 0) && (m_Version.Jtag_Ver == 0))) ) {
			// Check also jtag + swim ver =0  to be sure it is not a V1 or V2 with FW not programmed (blank)
			ifStatus = GetVersionExt(&m_Version);
			if (ifStatus != STLINKIF_NO_ERR) {
				LogTrace("ST-Link get Extended version failure");
				m_pStlinkInterface->CloseDevice(&m_handle, StlinkIdTcp);
				return ifStatus;
			}
		}
		LogTrace("STLink with %s interface detected", m_pStlinkInterface->GetIfLogString());
	}

	if( m_bStlinkConnected == false ) {
		return STLINKIF_CONNECT_ERR;
	}
	return ifStatus;
}
/*
 * @brief Opening routine based on the serial number of the STLink; to use in case StlinkInstId is no more
 * reliable (devices have been reenumerated between STLink instance selection and opening) or
 * to choose a specific STLink by its serial number.
 *
 * @param[in]  pPathOfProcess  Path, in ASCII, where STLinkUSBDriver library
 *                             is searched when not found in current dir, for Windows.
 * @param[in]  pSerialNumber  STLink serial number (ASCII).
 * @param[in]  bStrict 	 Used if STLink with required serial number is not found, but one other STLink is found:
 *                       choose if we open or not the other STLink. \n
 *                       If bStrict is true, the command will return #STLINKIF_STLINK_SN_NOT_FOUND if the
 *                       given SerialNumber is not found.\n
 *                       If bStrict is false and the given serial number is not found and one (and only one)
 *                       STLink is found, the command will attempt to open the STLink even if
 *                       the serial number does not match.
 * @param[in]  bForceRenum If true: force the renumeration (refresh device list after change
 *                         in the system). Otherwise the list remains as it was during the previous call.
 * @return STLinkInterface::OpenDevice() errors
 * @retval #STLINKIF_CONNECT_ERR USB error
 * @retval #STLINKIF_STLINK_SN_NOT_FOUND Serial number not found
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT StlinkDevice::OpenStlink(const char *pSerialNumber, bool bStrict, bool bForceRenum) {
	STLinkIf_StatusT ifStatus=STLINKIF_NO_ERR;
	int StlinkInstId;
	uint32_t StlinkIdTcp;

	if( pSerialNumber == NULL ) {
		LogTrace("NULL pointer for pSerialNumber in OpenStlink");
		return STLINKIF_PARAM_ERR;
	}

	if( m_pStlinkInterface == NULL ) {
		return STLINKIF_DLL_ERR;
	}

	ifStatus = m_pStlinkInterface->GetDeviceIdFromSerialNum(pSerialNumber, bStrict, &StlinkInstId, &StlinkIdTcp, bForceRenum);
	if (ifStatus == STLINKIF_NO_ERR) {
		ifStatus = OpenStlink(StlinkInstId, StlinkIdTcp);
	}
	return ifStatus;
}
/*
 * @brief Close STLink USB communication, with the device instance that was opened by StlinkDevice::OpenStlink()
 *
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT StlinkDevice::CloseStlink(void)
{
	if( m_bStlinkConnected == true ) {
		if( m_pStlinkInterface == NULL ) {
			return STLINKIF_DLL_ERR;
		}
		if( (m_handle != NULL) || ((m_deviceIdTcp !=0) && (m_pStlinkInterface->GetIfId() == STLINK_TCP))) {
			if( m_pStlinkInterface->CloseDevice(m_handle, m_deviceIdTcp) != STLINKIF_NO_ERR ) {
				LogTrace("Error closing %s USB communication", m_pStlinkInterface->GetIfLogString());
			}
		}
		// Consider the communication is closed even if error
		m_bStlinkConnected = false;
	}
	return STLINKIF_NO_ERR;
}

/*
 * @brief This routine gets USB VID and PID, and 3 legacy fields of STLink firmware version.
 * Available on all ST-Link generations, but Msc_Ver, Bridge_Ver and Power_Ver fields of
 * Stlk_VersionExtT are not retrieved. GetVersionExt must be called on ST-Link generation >=3 to get them.
 * @param[out] pVersion Pointer filled with version information.
 *
 * @retval #STLINKIF_NO_STLINK If StlinkDevice::OpenStlink() not called before
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT StlinkDevice::StGetVersion(Stlk_VersionExtT* pVersion)
{
	STLink_DeviceRequestT* pRq;
	STLinkIf_StatusT ifStatus;
	uint8_t version[6];

	if (m_bStlinkConnected == false) {
		// The function should be called at least after OpenStlink
		return STLINKIF_NO_STLINK;
	}
	if( (m_pStlinkInterface->GetIfId() != STLINK_DBG_INTERFACE) && (m_pStlinkInterface->GetIfId() != STLINK_TCP) ) {
		// This function is available only on the Debug interface
		return STLINKIF_NOT_SUPPORTED;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_CMD_SIZE_16;
	pRq->CDBByte[0] = ST_RBC_CMD;
	pRq->CDBByte[1] = 0x80;
	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = version;
	pRq->BufferLength = 6;
	pRq->SenseLength = DEFAULT_SENSE_LEN;

	// StGetVersion is called after m_bStlinkConnected=true, so we can call SendRequest
	// (preferable for semaphore management and status code analysis)
	ifStatus = SendRequest(pRq);
	delete pRq;

	if (ifStatus == STLINKIF_NO_ERR) {
		pVersion->Major_Ver = (version[0] >> 4) & 0x0F;
		pVersion->Jtag_Ver = ((version[0] << 2) & 0x3C) | ((version[1] >> 6) & 0x03);
		pVersion->Swim_Ver = version[1] & 0x3F;
		pVersion->VID = (((uint16_t)version[3]) << 8) + version[2];
		pVersion->PID = (((uint16_t)version[5]) << 8) + version[4];

		pVersion->Msc_Ver = 0;
		pVersion->Bridge_Ver = 0;
		pVersion->Power_Ver = 0;
	}
	return ifStatus;
}

/*
 * @brief This routine gets USB VID and PID, and firmware version of the STLink device.
 * @param[out] pVersion Pointer filled with version information.
 *
 * @retval #STLINKIF_NO_STLINK If StlinkDevice::OpenStlink() not called before
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT StlinkDevice::GetVersionExt(Stlk_VersionExtT* pVersion)
{
	STLink_DeviceRequestT *pRq;
	STLinkIf_StatusT ifStatus;
	uint8_t version[12];

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return STLINKIF_NO_STLINK;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_CMD_SIZE_16;
	pRq->CDBByte[0] = ST_GETVERSION_EXT;
	pRq->CDBByte[1] = 0x80;
	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = version;
	pRq->BufferLength = 12;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	// GetVersionExt is called after m_bStlinkConnected=true, so we can call SendRequest
	// (preferable for semaphore management and status code analysis)
	ifStatus = SendRequest(pRq);
	delete pRq;

	if( ifStatus == STLINKIF_NO_ERR ) {
		pVersion->Major_Ver = version[0];
		pVersion->Jtag_Ver = version[2];
		pVersion->Swim_Ver = version[1];
		pVersion->Msc_Ver = version[3];
		pVersion->Bridge_Ver = version[4];
		pVersion->Power_Ver = version[5];
		pVersion->VID = (((uint16_t)version[9])<<8) + version[8];
		pVersion->PID = (((uint16_t)version[11])<<8) + version[10];
	}
	return ifStatus;
}
/*
 * @brief Send a command over the USB and wait for answer.
 * Requires the STLink to be already connected (m_bStlinkConnected==true)
 * @param[in] pDevReq USB request to send.
 * @param[in] UsbTimeoutMs if 0 use default (5s) else use UsbTimeoutMs.
 *
 * @retval #STLINKIF_USB_COMM_ERR USB error when sending the request
 * @retval #STLINKIF_PARAM_ERR Null pointer
 * @retval #STLINKIF_NO_STLINK If StlinkDevice::OpenStlink() not called before
 * @retval #STLINKIF_DLL_ERR StlinkInterface not initialized
 * @retval #STLINKIF_NO_ERR If no error
*/
STLinkIf_StatusT StlinkDevice::SendRequest(STLink_DeviceRequestT *pDevReq,
											 const uint16_t UsbTimeoutMs) const
{
	STLinkIf_StatusT ifStatus;

	if( pDevReq == NULL ) {
		return STLINKIF_PARAM_ERR;
	}

	if( m_bStlinkConnected == false ) {
		return STLINKIF_NO_STLINK;
	}

	if( m_pStlinkInterface == NULL ) {
		return STLINKIF_DLL_ERR;
	}

	if( (m_pStlinkInterface->GetIfId() == STLINK_COM_PORT_DBG) || (m_pStlinkInterface->GetIfId() == STLINK_COM_PORT_PWR) ) {
		// This function is not available on COM port interfaces
		return STLINKIF_NOT_SUPPORTED;
	}

	ifStatus = m_pStlinkInterface->SendCommand(m_handle, m_deviceIdTcp, pDevReq, UsbTimeoutMs);
	if( ifStatus != STLINKIF_NO_ERR) {
		ifStatus = STLINKIF_USB_COMM_ERR;
	} else {
		ifStatus = STLINKIF_NO_ERR;
	}

	return ifStatus;
}

/**
 * @brief This routine gets target voltage in V, computed from STLink VREFINT value (typically
 * 1.2V at 25C).
 * @warning  Requires to have target voltage connected to T_VCC (not present on
 *           Bridge connector). If T_VCC is not connected return 0V.
 * @param[out] pVoltage  Target volatge in V.
 *
 * @retval #STLINKIF_NO_STLINK If StlinkDevice::OpenStlink() not called before
 * @retval #STLINKIF_PARAM_ERR If NULL pointer
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT StlinkDevice::GetTargetVoltage(float *pVoltage) const
{
	uint32_t adcMeasures[2];
	STLink_DeviceRequestT *pRq;
	STLinkIf_StatusT ifStatus;

	if( m_bStlinkConnected == false ) {
		// The function should be called at least after OpenStlink
		return STLINKIF_NO_STLINK;
	}

	pRq = new STLink_DeviceRequestT;
	memset(pRq, 0, sizeof(STLink_DeviceRequestT));

	pRq->CDBLength = STLINK_CMD_SIZE_16;
	pRq->CDBByte[0] = STLINK_GET_TARGET_VOLTAGE;

	pRq->BufferLength = 8;
	pRq->InputRequest = REQUEST_READ_1ST_EPIN;
	pRq->Buffer = adcMeasures;
	pRq->SenseLength=DEFAULT_SENSE_LEN;

	ifStatus = SendRequest(pRq);

	delete pRq;

	if( ifStatus == STLINKIF_NO_ERR ) {
		// First returned value is the ADC measure for VREFINT (according to datasheet: 1.2V);
		// the second value is Vtarget/2;
		if( (pVoltage != NULL) &&  (adcMeasures[0]!=0) ) {
			*pVoltage = 2*((float)adcMeasures[1])*(float)1.2/adcMeasures[0];
		}
	}

	return ifStatus;
}

STLinkIf_StatusT StlinkDevice::GetSerialNumFromSystem(int InstanceId)
{
	STLinkIf_StatusT ifStatus;
	TDeviceInfo devInfo;
	TDeviceInfo2 devInfo2;

	ifStatus = m_pStlinkInterface->GetDeviceInfo2(InstanceId, &devInfo2, sizeof(devInfo2));
	if( ifStatus == STLINKIF_NOT_SUPPORTED ) {
		ifStatus = m_pStlinkInterface->GetDeviceInfo(InstanceId, &devInfo, sizeof(devInfo));
		memcpy(m_SerialNum, devInfo.EnumUniqueId, SERIAL_NUM_STR_MAX_LEN);
	} else {
		memcpy(m_SerialNum, devInfo2.EnumUniqueId, SERIAL_NUM_STR_MAX_LEN);
	}
	return ifStatus;
}

#ifdef USING_ERRORLOG
/*
 * Associate files to be used for Error/Trace log (pErrLog must be initialized before)
 */
void StlinkDevice::BindErrLog(cErrLog *pErrLog)
{
	m_pErrLog = pErrLog;
	if( m_pStlinkInterface != NULL ) {
		m_pStlinkInterface->BindErrLog(pErrLog);
	}
}
#endif
/**********************************END OF FILE*********************************/

