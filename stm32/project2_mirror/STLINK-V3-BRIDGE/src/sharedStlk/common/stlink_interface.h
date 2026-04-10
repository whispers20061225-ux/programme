/**
  ******************************************************************************
  * @file    stlink_interface.h
  * @author  GPM-AppliTools-HWBoards Team
  * @brief   Header for stlink_interface.cpp module
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
/** @addtogroup INTERFACE
 * @{
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _STLINK_INTERFACE_H
#define _STLINK_INTERFACE_H
/* Includes ------------------------------------------------------------------*/
#include "STLinkUSBDriver.h"

#ifdef USING_ERRORLOG
#include "ErrLog.h"
#endif

#define MAX_TCP_CONN_PARAM_SIZE 50 // 49 characters + EOS @server:port
#define MAX_TCP_PORT_PARAM_SIZE 5  // 4 characters + EOS
#define MAX_TCP_SERVER_PARAM_SIZE (MAX_TCP_CONN_PARAM_SIZE  - MAX_TCP_PORT_PARAM_SIZE -2)
#define MAX_TCP_OPTION_PARAM_SIZE 50
#define DEFAULT_TCP_SERVER_OPTION "-a" //server autokill on last client
#define DEFAULT_TCP_SERVER        "localhost"
#define DEFAULT_TCP_PORT          "7184"

/* Exported types and constants ----------------------------------------------*/
// Warning if modified update also: ConvSTLinkIfToDbgDevStatus
/// Interface Error and Status
typedef enum {
	STLINKIF_NO_ERR = 0,         ///< OK (no error)
	STLINKIF_CONNECT_ERR,        ///< USB Connection error
	STLINKIF_DLL_ERR,            ///< USB DLL error
	STLINKIF_USB_COMM_ERR,       ///< USB Communication error
	STLINKIF_PARAM_ERR,          ///< Wrong parameters error
	STLINKIF_NO_STLINK,          ///< STLink device not opened error
	STLINKIF_NOT_SUPPORTED,      ///< Interface or command not supported
	STLINKIF_PERMISSION_ERR,     ///< STLink device already in use by another program error
	STLINKIF_ENUM_ERR,           ///< USB enumeration error
	STLINKIF_GET_INFO_ERR,       ///< Error getting STLink device information
	STLINKIF_STLINK_SN_NOT_FOUND,///< Required STLink serial number not found error
	STLINKIF_CLOSE_ERR,          ///< Error during device Close
	STLINKIF_TCP_BUSY            ///< Busy device in shared mode
} STLinkIf_StatusT;

/// Parameters used in ST-Link shared mode (STLINK_TCP interface)
typedef struct {
	char* ServerName; ///< TCP server name to be used when starting TCP server (e.g. "localhost", "127.0.0.1")
	char* PortName;   ///< Port name to be used when starting TCP server and when creating TCP client (e.g. "7184")
	char* CmdLineOptions; ///< Command line options to be used when launching TCP server executable (e.g. "-d3" for debug)
} STLinkIf_TcpServerParamT;


/* Class -------------------------------------------------------------------- */
/// STLinkInterface Class
class STLinkInterface
{
public:

	// Constructor selecting the ST-Link interface to enumerate and manage
	STLinkInterface(STLink_EnumStlinkInterfaceT IfId= STLINK_DBG_INTERFACE);

	virtual ~STLinkInterface(void);

	STLink_EnumStlinkInterfaceT GetIfId(void) const {return m_ifId;}

	const char* GetIfLogString(void) const;

	// Load STLinkUSBDriver library (if dynamic link) and get information about it.
	// Call mandatory before any other method
	STLinkIf_StatusT LoadStlinkLibrary(const TCHAR *pPathOfProcess);

	bool IsLibraryLoaded();

	STLinkIf_StatusT EnumDevices(uint32_t *pNumDevices, bool bClearList);

	STLinkIf_StatusT EnumDevicesIfRequired(uint32_t* pNumDevices, bool bForceRenum, bool bClearList);

	// Legacy management; prefer using GetDeviceInfo2
	STLinkIf_StatusT GetDeviceInfo(int StlinkInstId, STLink_DeviceInfoT* pInfo, uint32_t InfoSize);

	STLinkIf_StatusT GetDeviceInfo2(int StlinkInstId, STLink_DeviceInfo2T *pInfo, uint32_t InfoSize);

	STLinkIf_StatusT OpenDevice(int StlinkInstId, uint32_t StlinkIdTcp, bool bOpenExclusive, void **pHandle);

	STLinkIf_StatusT GetDeviceIdFromSerialNum(const char *pSerialNumber, bool bStrict, int *pStlinkInstId, uint32_t *pStlinkIdTcp, bool bForceRenum);

	STLinkIf_StatusT CloseDevice(void *pHandle, uint32_t StlinkIdTcp);

	STLinkIf_StatusT SendCommand(void *pHandle, uint32_t StlinkIdTcp, STLink_DeviceRequestT *pDevReq, const uint16_t UsbTimeoutMs);

	const TCHAR * GetPathOfProcess(void) const {return m_pathOfProcess;}
#ifdef USING_ERRORLOG
	void BindErrLog(cErrLog *pErrLog);
#endif

	/**
	 * @ingroup INTERFACE
	 * @brief Configure the TCP server parameters to be used when starting the ST-Link TCP client or
	 * starting the TCP server. They will be taken into account at next TCP client and/or server start.
	 * If a TCP connection is already running (client and/or server) parameters are only stored for
	 * use at next server/ client launch.
	 * Call to this function is optional if default values (server="localhost" port="7184" option="") are fine.
	 * Call to this function must be done before calling EnumerateDevInterface() or OpenStlink()
	 * to take into account params at TCP initializations.
	 *
	 * @param[in]  paramNb = 3 (greater value reserved for future use)
	 * @param[in]  pParams: pointer to the #STLinkIf_TcpServerParamT structure providing the parameters
	 *                      to use for TCP connection. If one field is unused set it to NULL (default value will be used).
	 */
	STLinkIf_StatusT SetTcpServerParam(uint8_t paramNb, STLinkIf_TcpServerParamT* pParams);

	/**
	 * @ingroup INTERFACE
	 * @brief Get the number of TCP clients connected to the given ST-Link device.
	 *
	 * @param[in]  stLinkUsbIdParam   ST-Link device identifier in shared mode
	 *                            (value from #STLink_DeviceInfo2T.StLinkUsbId)
	 * @param[out] pNum  Pointer to the caller-allocated variable receiving the number
	 *                   of clients currently connected to the given ST-Link instance.
	 *
	 * @retval #STLINKIF_NOT_SUPPORTED If STLinkUSBDriver is too old for STLINK_TCP interface or selected interface is not STLINK_TCP
	 * @retval #STLINKIF_NO_STLINK If stLinkUsbIdParam == 0 (not expected)
	 * @retval #STLINKIF_NO_ERR If no error. Only in this case the output param is reliable.
	 */
	STLinkIf_StatusT GetNumOfDeviceClientsTcp(uint32_t* pNum, uint32_t stLinkUsbIdParam = 0);

	uint32_t GetTcpServerApiVer(void) const { return m_tcpServerVer.ApiVer; }

private:

#ifdef WIN32 //Defined for applications for Win32 and Win64.
	// Added in DLL API v3
	pSTLink_GetLibApiVer     STLink_GetLibApiVer;
	// DLL API v2; should be used if available
	pSTLink_Reenumerate      STLink_Reenumerate;
	pSTLink_GetNbDevices     STLink_GetNbDevices;
	pSTLink_GetDeviceInfo    STLink_GetDeviceInfo;
	pSTLink_GetDeviceInfo2   STLink_GetDeviceInfo2;
	pSTLink_OpenDevice       STLink_OpenDevice;
	pSTLink_CloseDevice      STLink_CloseDevice;
	pSTLink_SendCommand      STLink_SendCommand;
	pSTLink_ReenumerateTcp   STLink_ReenumerateTcp;
	pSTLink_OpenDeviceTcp    STLink_OpenDeviceTcp;
	pSTLink_CloseDeviceTcp   STLink_CloseDeviceTcp;
	pSTLink_SendCommandTcp   STLink_SendCommandTcp;
	pSTLink_GetNumOfDeviceClientsTcp  STLink_GetNumOfDeviceClientsTcp;
	pSTLink_GetServerVersion STLink_GetServerVersion;
#endif
	// Old API of STLinkUSBDriver.dll; kept only for ascendant compatibility
	pSTMass_Enum_GetNbDevices    STMass_Enum_GetNbDevices;
	pSTMass_Enum_GetDevice       STMass_Enum_GetDevice;
	pSTMass_GetDeviceInfo        STMass_GetDeviceInfo;
	pSTMass_Enum_Reenumerate     STMass_Enum_Reenumerate;
	pSTMass_OpenDevice           STMass_OpenDevice;
	pSTMass_OpenDeviceExclusive  STMass_OpenDeviceExclusive;
	pSTMass_CloseDevice          STMass_CloseDevice;
	pSTMass_SendCommand          STMass_SendCommand;

	void LogTrace(const char *pMessage, ...);

#ifdef WIN32 //Defined for applications for Win32 and Win64.
	HMODULE m_hMod;
#endif
	STLink_EnumStlinkInterfaceT m_ifId;
	uint32_t m_nbEnumDevices;

	// stored path of process in ASCII
	TCHAR m_pathOfProcess[MAX_PATH];

	// API version of STLinkUSBDriver library;
	uint32_t m_libApiVer;

	// Flag for STLinkUSBDriver.dll loaded state
	bool m_bApiDllLoaded;

	// Flag for enumerating the Device (Bridge, ...) interface when required
	bool m_bDevInterfaceEnumerated;

	// ST-Link server version (when in shared mode)
	STLink_ServerVersionT m_tcpServerVer;

	// TCP server connection params use SetTcpServerParam() to set them
	char m_tcpConnectParams[MAX_TCP_CONN_PARAM_SIZE];
	char m_tcpServerOptionParams[MAX_TCP_OPTION_PARAM_SIZE];

	// Opened device handle (legacy management, put here to simplify the API and hide to upper layers)
	PDevice m_legacyDevice;

#ifdef USING_ERRORLOG
	// Error log management
	cErrLog *m_pErrLog;
#endif
};

#endif //_STLINK_INTERFACE_H
// end group INTERFACE
/** @} */
/**********************************END OF FILE*********************************/