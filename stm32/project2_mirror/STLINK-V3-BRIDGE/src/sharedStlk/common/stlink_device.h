/**
  ******************************************************************************
  * @file    stlink_device.h
  * @author  GPM-AppliTools-HWBoards Team
  * @brief   Header for stlink_device.cpp module
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
/** @addtogroup DEVICE
 * @{
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _STLINK_DEVICE_H
#define _STLINK_DEVICE_H
/* Includes ------------------------------------------------------------------*/
#include "stlink_type.h"
#include "stlink_if_common.h"
#include "stlink_interface.h"
#include "stlink_fw_api_common.h"

#ifdef USING_ERRORLOG
#include "ErrLog.h"
#endif

/* Exported types and constants ----------------------------------------------*/

// ------------------------------------------------------------------------- //
/* Class -------------------------------------------------------------------- */
/// Device Class
class StlinkDevice
{
public:

	StlinkDevice(STLinkInterface &StlinkIf);

	virtual ~StlinkDevice(void);

	void SetOpenModeExclusive(bool bExclusive);

#ifdef USING_ERRORLOG
	void BindErrLog(cErrLog *pErrLog);
#endif

	/*
	 * Getter for m_bStlinkConnected: True if connected to an STLink (OpenStlink() done)
	 */
	bool GetIsStlinkConnected(void) const {
		return m_bStlinkConnected;
	}
	/**
	 * @brief  Command returning the firmware major version of the connected ST-Link
	 * @retval firmware major version of the connected ST-Link
	 */
	uint8_t GetMajorVer(void) const {
		return m_Version.Major_Ver;
	}
	/**
	 * @brief  Command returning the firmware STM32 Debug version of the connected ST-Link
	 * @retval firmware STM32 Debug version of the connected ST-Link. 0 if STM32 debug not supported
	 */
	uint8_t GetStm32DbgVer(void) const {
		return m_Version.Jtag_Ver;
	}
	/**
	 * @brief  Command returning the firmware STM8 Debug version of the connected ST-Link
	 * @retval firmware STM8 Debug version of the connected ST-Link. 0 if STM8 debug not supported
	 */
	uint8_t GetStm8DbgVer(void) const {
		return m_Version.Swim_Ver;
	}
	/**
	 * @brief  Command returning the firmware mass storage version of the connected ST-Link
	 * @retval firmware mass storage version of the connected ST-Link. 0 if mass storage not supported
	 */
	uint8_t GetMscVcpVer(void) const {
		return m_Version.Msc_Ver;
	}
	/**
	 * @brief  Command returning the firmware Bridge version of the connected ST-Link
	 * @retval firmware Bridge version of the connected ST-Link. 0 if Bridge not supported
	 */
	uint8_t GetBridgeVer(void) const {
		return m_Version.Bridge_Ver;
	}
	/**
	 * @brief  Command returning the ST-Link USB VendorID
	 * @retval ST-Link USB VendorID
	 */
	uint16_t GetUsbVid(void) const {
		return m_Version.VID;
	}
	/**
	 * @brief  Command returning the ST-Link USB ProductID
	 * @retval ST-Link USB ProductID
	 */
	uint16_t GetUsbPid(void) const {
		return m_Version.PID;
	}

	// Flag indicating if a session is opened (public for legacy but use getter, may become private)
	bool m_bStlinkConnected;

	//STLink version (public for legacy but use getter, may become private)
	Stlk_VersionExtT  m_Version;

protected:
	STLinkIf_StatusT OpenStlink(int StlinkInstId=0, uint32_t StlinkIdTcp=0);
	STLinkIf_StatusT OpenStlink(const char *pSerialNumber, bool bStrict, bool bForceRenum);
	STLinkIf_StatusT CloseStlink(void);
	STLinkIf_StatusT StGetVersion(Stlk_VersionExtT* pVersion);
	STLinkIf_StatusT GetVersionExt(Stlk_VersionExtT* pVersion);
	STLinkIf_StatusT GetTargetVoltage(float *pVoltage) const;

	STLinkIf_StatusT SendRequest(STLink_DeviceRequestT *pDevReq, const uint16_t UsbTimeoutMs=0) const;
	void LogTrace(const char *pMessage, ...) const;

	char m_SerialNum[SERIAL_NUM_STR_MAX_LEN];

	STLinkInterface* m_pStlinkInterface;

private:
	// Private routine to get the ST-Link serial number from the system and to store it into m_SerialNum
	STLinkIf_StatusT GetSerialNumFromSystem(int InstanceId);

	// Opened device handles
	void    *m_handle;
	uint32_t m_deviceIdTcp;

	// Mode for device opening: shared or exclusive
	bool m_bOpenExclusive;

#ifdef USING_ERRORLOG
	// Error log management
	cErrLog *m_pErrLog;
#endif
};

#endif //_STLINK_DEVICE_H
// end group DEVICE
/** @} */
/**********************************END OF FILE*********************************/