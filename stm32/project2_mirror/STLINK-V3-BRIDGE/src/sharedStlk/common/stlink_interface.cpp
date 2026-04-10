/**
  ******************************************************************************
  * @file    stlink_interface.cpp
  * @author  GPM-AppliTools-HWBoards Team
  * @brief   Module to access the STLink device library (STLinkUSBDriver.h).
  *          Manage communication interfaces (USB) to connect to the STLink device. \n
  *          STLinkInterface class manages USB enumeration and STLink devices detection.
  *          STLinkInterface object to be initialized before being used by
  *          StlinkDevice (or derived) instance
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
#include <stdio.h>
#include "criticalsectionlock.h"
#include "stlink_interface.h"
#ifdef WIN32 //Defined for applications for Win32 and Win64.
#include "shlwapi.h"
#endif // WIN32
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#if defined(_MSC_VER) && (_MSC_VER >= 1400) /* VC8+ (VS2005) */
#define STRCPY_S(A,B,C) strcpy_s(A,B,C)
#define STRICMP(A,B) _stricmp(A,B)
#define STRNICMP(A,B,C) _strnicmp(A,B,C)
#else
#define STRCPY_S(A,B,C) strcpy(A,C)
#define STRICMP(A,B) strcasecmp(A,B)
#define STRNICMP(A,B,C) strncasecmp(A,B,C)
#endif
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Global variables ----------------------------------------------------------*/
static int g_nbInstances=0;
#ifdef USE_RECURSIVE_MUTEX
static std::recursive_mutex g_csInterface;
#else
#ifdef WIN32 //Defined for applications for Win32 and Win64.
// Create a critical section
static CRITICAL_SECTION g_csInterface;
#else
// critical section object (statically allocated)
static pthread_mutex_t g_csInterface =  PTHREAD_MUTEX_INITIALIZER;
#endif // WIN32
#endif // USE_RECURSIVE_MUTEX

// used for debug trace: interface name according to STLink_EnumStlinkInterfaceT value
const char * LogIfString[STLINK_NB_INTERFACES] =
{ "DBG", "DBG2", "DBG SERVER", "BRIDGE", "COM DBG", "COM PWR" };

/* Class Functions Definition ------------------------------------------------*/

/**
 * @ingroup INTERFACE
 * @brief STLinkInterface constructor
 * @param[in]  IfId  STLink USB interface to be used: #STLINK_BRIDGE for Bridge interface.
 *                   #STLINK_DBG_INTERFACE for ST Debug interface.
 *                   #STLINK_TCP for ST-Link shared mode (Debug only).
 *                   #STLINK_COM_PORT_PWR for Power interface (enumeration only).
 *                   Other interfaces not supported currently.
 */
STLinkInterface::STLinkInterface(STLink_EnumStlinkInterfaceT IfId): m_ifId(IfId), m_nbEnumDevices(0),
	m_libApiVer(STLINK_LIB_API_VERSION_LEGACY), m_bApiDllLoaded(false), m_bDevInterfaceEnumerated(false),
	m_legacyDevice(NULL)
{
#ifdef WIN32 //Defined for applications for Win32 and Win64.
	STLink_GetLibApiVer = NULL;
	STLink_Reenumerate = NULL;
	STLink_GetNbDevices = NULL;
	STLink_GetDeviceInfo = NULL;
	STLink_GetDeviceInfo2 = NULL;
	STLink_OpenDevice = NULL;
	STLink_CloseDevice = NULL;
	STLink_SendCommand = NULL;
	STLink_ReenumerateTcp = NULL;
	STLink_OpenDeviceTcp = NULL;
	STLink_CloseDeviceTcp = NULL;
	STLink_SendCommandTcp = NULL;
	STLink_GetNumOfDeviceClientsTcp = NULL;
	STLink_GetServerVersion = NULL;
	STMass_Enum_GetNbDevices = NULL;
	STMass_Enum_GetDevice = NULL;
	STMass_GetDeviceInfo = NULL;
	STMass_Enum_Reenumerate = NULL;
	STMass_OpenDevice = NULL;
	STMass_OpenDeviceExclusive = NULL;
	STMass_CloseDevice = NULL;
	STMass_SendCommand = NULL;
	m_hMod = NULL;
#endif
	m_pathOfProcess[0]='\0';

	m_tcpServerVer.ApiVer = 0;
	m_tcpServerVer.MainVer = 0;
	m_tcpServerVer.RevVer = 0;
	m_tcpServerVer.BuildVer = 0;

	m_tcpConnectParams[0] = '\0';

	STRCPY_S(m_tcpServerOptionParams, MAX_TCP_OPTION_PARAM_SIZE, DEFAULT_TCP_SERVER_OPTION);

#ifndef USE_RECURSIVE_MUTEX
#ifdef WIN32 //Defined for applications for Win32 and Win64.
	// Initialize the critical section
	if( g_nbInstances == 0 ) {
		// The first instance initializes the critical section for all instances
		InitializeCriticalSection(&g_csInterface);
	}
#else
	// nothing to do (statically allocated with PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP macro)
#endif
#endif
	g_nbInstances ++;
#ifdef USING_ERRORLOG
	// Error log management
	m_pErrLog = NULL;
#endif
}
/**
 * @ingroup INTERFACE
 * @brief STLinkInterface destructor
 */
STLinkInterface::~STLinkInterface(void)
{
	if( g_nbInstances > 0 ) {
		g_nbInstances --;
	}
#ifdef WIN32
	// Delete the critical section
	if( g_nbInstances == 0 ) {
#ifndef USE_RECURSIVE_MUTEX
		// The last instance really deletes the critical section
		DeleteCriticalSection(&g_csInterface);
#endif
	}

	//Defined for applications for Win32 and Win64.
#ifdef USING_ERRORLOG
	// Flush the trace log system after closing
	if( m_pErrLog != NULL ) {
		m_pErrLog->Dump();
	}
#endif

	if( (m_hMod != NULL) && (g_nbInstances == 0) ) {
		// The last instance unloads the library
		if( FreeLibrary(m_hMod) != 0 ) {
			// Successful
			m_hMod = NULL;
		}
	}
#else
	// critical section deletion not needed because static mutex
	if( g_nbInstances == 0 ) {
		STLink_FreeLibrary();
	}
#endif // WIN32
}
/*
 * Trace logging mechanism, under compilation switch USING_ERRORLOG (requiring ErrLog.h and ErrLog.cpp)
 */
void STLinkInterface::LogTrace(const char *pMessage, ...)
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

const char* STLinkInterface::GetIfLogString(void) const {
	return LogIfString[m_ifId];
}

/**
 * @ingroup INTERFACE
 * @brief If not already done: load the STLinkUSBDriver library.
 *
 * @param[in]  pPathOfProcess  Path, in ASCII, where STLinkUSBDriver library is searched
 *                             when not found in current dir, for Windows.
 *
 * @retval #STLINKIF_NOT_SUPPORTED m_ifId not supported yet
 * @retval #STLINKIF_DLL_ERR  STLinkUSBDriver library not loaded
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT  STLinkInterface::LoadStlinkLibrary(const TCHAR *pPathOfProcess)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;

	if ( (m_ifId == STLINK_RW_INTERFACE) || (m_ifId >= STLINK_NB_INTERFACES) ) {
		return STLINKIF_NOT_SUPPORTED;
	}

	if (m_bApiDllLoaded == false) {
		// Load the STLinkUSBDriver library only once by instance
		if (pPathOfProcess != NULL) {
			// Memorize the path of process
#ifdef WIN32
#if defined(_MSC_VER) &&  (_MSC_VER >= 1400) /* VC8+ (VS2005) */
			::_tcsncpy_s(m_pathOfProcess, MAX_PATH, pPathOfProcess, MAX_PATH - 1);
#else
			::_tcsncpy(m_pathOfProcess, pPathOfProcess, MAX_PATH - 1);
#endif
			// Copy up to N-1 char to leave room for NULL; otherwise if the input is too long the invalid parameter handler is invoked.
			// We could also pass _TRUNCATE
#else
			::strncpy(m_pathOfProcess, pPathOfProcess, MAX_PATH);
#endif
		}

#ifdef WIN32 //Defined for applications for Win32 and Win64.
		if (m_hMod == NULL) {
			// First try from this DLL path
			if (pPathOfProcess != NULL) {
				TCHAR szDllPath[_MAX_PATH];
#ifdef WIN32
#if defined(_MSC_VER) &&  (_MSC_VER >= 1400) /* VC8+ (VS2005) */
				::_tcsncpy_s(szDllPath, _MAX_PATH, m_pathOfProcess, _MAX_PATH - 1);
#else
				::_tcsncpy(szDllPath, m_pathOfProcess, MAX_PATH - 1);
#endif
#else
				::strncpy(szDllPath, m_pathOfProcess, _MAX_PATH);
#endif
				// Note: Unicode is not supported (would require T_CHAR szDllPath, to include <tchar.h>
				// and to use generic function PathAppend(szDllPath, _T("STLinkUSBDriver.dll"))
				::PathAppend(szDllPath, _T("STLinkUSBDriver.dll"));

				m_hMod = LoadLibrary(szDllPath);
			}
		}

		// Try loading the library using a predefined relative path
		if (m_hMod == NULL) {
#if __x86_64__
			const TCHAR* p = _T("native/win_x64/STLinkUSBDriver.dll");
#else
			const TCHAR* p = _T("native/win_x86/STLinkUSBDriver.dll");
#endif
			m_hMod = LoadLibrary(p);
		}

		if (m_hMod == NULL) {
			// Second try using the whole procedure for path resolution (including PATH environment variable)
			m_hMod = LoadLibrary(_T("STLinkUSBDriver.dll"));
		}

		if (m_hMod == NULL) {
			LogTrace("STLinkInterface Failure loading STLinkUSBDriver.dll");
			return STLINKIF_DLL_ERR;
		}

		// If here, the DLL was loaded. Check if required routines are present
		LogTrace("STLinkInterface STLinkUSBDriver.dll loaded");
		// Get the needed API
		STLink_GetLibApiVer = (pSTLink_GetLibApiVer)GetProcAddress(m_hMod, ("STLink_GetLibApiVer"));
		STLink_Reenumerate = (pSTLink_Reenumerate)GetProcAddress(m_hMod, ("STLink_Reenumerate"));
		STLink_GetNbDevices = (pSTLink_GetNbDevices)GetProcAddress(m_hMod, ("STLink_GetNbDevices"));
		STLink_GetDeviceInfo = (pSTLink_GetDeviceInfo)GetProcAddress(m_hMod, "STLink_GetDeviceInfo");
		STLink_GetDeviceInfo2 = (pSTLink_GetDeviceInfo2)GetProcAddress(m_hMod, ("STLink_GetDeviceInfo2"));
		STLink_OpenDevice = (pSTLink_OpenDevice)GetProcAddress(m_hMod, ("STLink_OpenDevice"));
		STLink_CloseDevice = (pSTLink_CloseDevice)GetProcAddress(m_hMod, ("STLink_CloseDevice"));
		STLink_SendCommand = (pSTLink_SendCommand)GetProcAddress(m_hMod, ("STLink_SendCommand"));
		STLink_ReenumerateTcp = (pSTLink_ReenumerateTcp)GetProcAddress(m_hMod, "STLink_ReenumerateTcp");
		STLink_OpenDeviceTcp = (pSTLink_OpenDeviceTcp)GetProcAddress(m_hMod, "STLink_OpenDeviceTcp");
		STLink_CloseDeviceTcp = (pSTLink_CloseDeviceTcp)GetProcAddress(m_hMod, "STLink_CloseDeviceTcp");
		STLink_SendCommandTcp = (pSTLink_SendCommandTcp)GetProcAddress(m_hMod, "STLink_SendCommandTcp");
		STLink_GetNumOfDeviceClientsTcp = (pSTLink_GetNumOfDeviceClientsTcp)GetProcAddress(m_hMod, "STLink_GetNumOfDeviceClientsTcp");
		STLink_GetServerVersion = (pSTLink_GetServerVersion)GetProcAddress(m_hMod, "STLink_GetServerVersion");

		// Check if TCP routines are present if required
		if ((m_ifId == STLINK_TCP)
			&& (!STLink_SendCommandTcp || !STLink_CloseDeviceTcp || !STLink_GetDeviceInfo2 || !STLink_OpenDeviceTcp
				|| !STLink_ReenumerateTcp || !STLink_GetNumOfDeviceClientsTcp)) {
			// TCP routines are required and missing
			ifStatus = STLINKIF_DLL_ERR;
		}

		// Check if all routines for BRIDGE are present if required. This is mandatory but not enough for BRIDGE support
		// STLink_Reenumerate will return SS_BAD_PARAMETER if DLL is too old and Bridge interface is not supported.
		if ((m_ifId == STLINK_BRIDGE)
			&& (!STLink_Reenumerate || !STLink_GetNbDevices || !STLink_GetDeviceInfo2 || !STLink_OpenDevice
				|| !STLink_CloseDevice || !STLink_SendCommand)) {
			// Some required routines are missing
			ifStatus = STLINKIF_DLL_ERR;
		}

		// Manage legacy cases for DEBUG interface
		if (!STLink_Reenumerate || !STLink_GetNbDevices || !STLink_GetDeviceInfo ||
			!STLink_OpenDevice || !STLink_CloseDevice || !STLink_SendCommand) {
			// At least one mandatory command of the dll API v2 was not found: try the API v1 one
			m_libApiVer = STLINK_LIB_API_VERSION_LEGACY;
			STMass_Enum_Reenumerate = (pSTMass_Enum_Reenumerate)GetProcAddress(m_hMod, "STMass_Enum_Reenumerate");
			STMass_Enum_GetNbDevices = (pSTMass_Enum_GetNbDevices)GetProcAddress(m_hMod, "STMass_Enum_GetNbDevices");
			STMass_Enum_GetDevice = (pSTMass_Enum_GetDevice)GetProcAddress(m_hMod, "STMass_Enum_GetDevice");
			STMass_GetDeviceInfo = (pSTMass_GetDeviceInfo)GetProcAddress(m_hMod, "STMass_GetDeviceInfo");
			STMass_OpenDevice = (pSTMass_OpenDevice)GetProcAddress(m_hMod, "STMass_OpenDevice");
			STMass_OpenDeviceExclusive = (pSTMass_OpenDeviceExclusive)GetProcAddress(m_hMod, "STMass_OpenDeviceExclusive");
			STMass_CloseDevice = (pSTMass_CloseDevice)GetProcAddress(m_hMod, "STMass_CloseDevice");
			STMass_SendCommand = (pSTMass_SendCommand)GetProcAddress(m_hMod, "STMass_SendCommand");
			if (!STMass_Enum_Reenumerate || !STMass_Enum_GetNbDevices || !STMass_Enum_GetDevice
				|| !STMass_GetDeviceInfo || !STMass_OpenDevice || !STMass_CloseDevice || !STMass_SendCommand) {
				ifStatus = STLINKIF_DLL_ERR;
				// Note that for easier ascendant compatibilty, STMass_OpenDeviceExclusive is not absolutely required,
				// and might be replaced by STMass_OpenDevice when used with old dll
			}
		} else {
			if (STLink_GetLibApiVer) {
				m_libApiVer = STLink_GetLibApiVer();
			} else {
				// The "new" API is exported but not STLink_GetLibApiVer already
				m_libApiVer = STLINK_LIB_API_VERSION_MIN_FOR_TCP;
			}
		}
#else // !WIN32
		m_libApiVer = STLink_GetLibApiVer();
#endif
	}
	if (ifStatus == STLINKIF_NO_ERR) {
		m_bApiDllLoaded = true;
	}
	return ifStatus;
}

/**
 * @ingroup INTERFACE
 *
 * @brief Return true if STLinkInterface::LoadStlinkLibrary() has been called successfully.
 */
bool STLinkInterface::IsLibraryLoaded() {
	return m_bApiDllLoaded;
}

/**
 * @ingroup INTERFACE
 * @brief USB enumeration routine.
 * Enumerate all devices presenting the given USB STLink interface
 * (or refresh device list after change in the system).
 *
 * @param[in]  bClearList  If true:\n
 *               - All opened devices are closed and handles are deleted.\n
 *               - The use of a previously returned handle is UNPREDICTABLE (might either fail
 *               because the handle is no more known, or might address a newly renumerated
 *               device that has been granted the same handle by chance ... \n
 *               - Useful for closing all devices is case returned handles have been lost by the
 *               caller. In standard case, bClearList == 0 has to be preferred.
 * @param[out] pNumDevices Pointer where the function returns the number of connected STLink m_ifId interfaces.
 *                          Can be NULL if not wanted.
 *
 * @retval #STLINKIF_NOT_SUPPORTED If STLinkUSBDriver is too old for given m_ifId or m_ifId not supported yet
 * @retval #STLINKIF_NO_STLINK No STLink with given m_ifId connected.
 * @retval #STLINKIF_PERMISSION_ERR Lack of permission during enumeration
 * @retval #STLINKIF_ENUM_ERR Error during enumeration
 * @retval #STLINKIF_DLL_ERR  STLinkUSBDriver library not loaded
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface::EnumDevices(uint32_t *pNumDevices, bool bClearList)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NOT_SUPPORTED;
	uint32_t status = SS_OK;

	if( pNumDevices != NULL ) {
		*pNumDevices=0; // default if error
	}

	if( IsLibraryLoaded() == true ) {
		if (m_libApiVer >= STLINK_LIB_API_VERSION_MIN_FOR_TCP) {
			if (m_ifId == STLINK_TCP) {
				status = STLink_ReenumerateTcp(m_ifId, bClearList, m_tcpConnectParams, m_tcpServerOptionParams);
			} else {
				status = STLink_Reenumerate(m_ifId, bClearList);
			}
			// Note that STLink_Reenumerate might fail because of issue during serial number retrieving
			// which is not a blocking error here; but TCP error is
			if ((status == SS_TCP_ERROR) || (status == SS_TCP_CANT_CONNECT)) {
				LogTrace("Error (0x%06lx) during ST-Link enumeration through TCP", (unsigned long)status);
				return STLINKIF_ENUM_ERR;
			}
			m_nbEnumDevices = STLink_GetNbDevices(m_ifId);

			// Get the TCP server version if possible;
			// First clear the value possibly retrieved sooner
			m_tcpServerVer.ApiVer = 0;
			m_tcpServerVer.MainVer = 0;
			m_tcpServerVer.RevVer = 0;
			m_tcpServerVer.BuildVer = 0;
#ifdef WIN32
			if ((status == SS_OK) && (m_ifId == STLINK_TCP) && STLink_GetServerVersion) {
#else
			if ((status == SS_OK) && (m_ifId == STLINK_TCP)) {
#endif
				status = STLink_GetServerVersion(&m_tcpServerVer);
				if (status != SS_OK) {
					LogTrace("Error (0x%06lx) during STLink_GetServerVersion", (unsigned long)status);
					// Do not consider this as a fatal error
					status = SS_OK;
				}
			}
		} else {
			STMass_Enum_Reenumerate();
			m_nbEnumDevices = STMass_Enum_GetNbDevices();
		}
		if( status == SS_BAD_PARAMETER ) {
			// DLL is too old and does not support BRIDGE interface
			m_bApiDllLoaded = false;
			return STLINKIF_DLL_ERR;
		}

		if( m_nbEnumDevices == 0 ) {
			LogTrace("No STLink device with %s interface detected on the USB", LogIfString[m_ifId]);
			return STLINKIF_NO_STLINK;
		}

		if( status == SS_OK ) {
			m_bDevInterfaceEnumerated = true;
			ifStatus = STLINKIF_NO_ERR;
		} else {
			if( status == SS_PERMISSION_ERR ) {
				LogTrace("STLinkInterface Lack of permission during enumeration");
				ifStatus = STLINKIF_PERMISSION_ERR;
			} else {
				LogTrace("STLinkInterface Error during enumeration");
				ifStatus = STLINKIF_ENUM_ERR;
			}
		}

		if( pNumDevices != NULL ) {
			*pNumDevices=(int)m_nbEnumDevices;
		}
	} else { // IsLibraryLoaded()
		ifStatus = STLINKIF_DLL_ERR;
	}
	return ifStatus;
}
/*
 * @brief  Private enumeration routine called by other functions. USB enumeration routine.
 * Enumerate all devices presenting the current USB STLink interface
 * (or refresh device list after change in the system).
 *
 * @param[in]  bForceRenum If true: force the renumeration (refresh device list after change
 *                         in the system). Otherwise the list remains as it was during the previous call.
 * @param[in]  bClearList  If true:\n
 *               - All opened devices are closed and handles are deleted.\n
 *               - The use of a previously returned handle is UNPREDICTABLE (might either fail
 *               because the handle is no more known, or might address a newly renumerated
 *               device that has been granted the same handle by chance ... \n
 *               - Useful for closing all devices is case returned handles have been lost by the
 *               caller. In standard case, bClearList == 0 has to be preferred.
 * @param[out] pNumDevices Pointer where the function returns the number of connected STLink devices
 *                         with the required interface. Can be NULL if not wanted.
 * @warning #STLINKIF_PERMISSION_ERR can be returned if one device is already opened by another program,
 *          however the device is listed in pNumDevices.
 *
 * @retval #STLINKIF_NOT_SUPPORTED Parameter(s) error
 * @retval #STLINKIF_DLL_ERR Error in loading STLinkUSBDriver library or too old not supporting current Interface
 * @retval #STLINKIF_PERMISSION_ERR Lack of permission during USB enumeration
 * @retval #STLINKIF_ENUM_ERR USB enumeration error
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface::EnumDevicesIfRequired(uint32_t *pNumDevices, bool bForceRenum, bool bClearList)
{
	STLinkIf_StatusT ifStatus=STLINKIF_NO_ERR;

	if( pNumDevices != NULL ) {
		// Will return 0 in case of error
		*pNumDevices=0;
	}

	if( (m_bDevInterfaceEnumerated == false) || (bForceRenum==true) ) {

		ifStatus = EnumDevices(pNumDevices, bClearList);
		if( m_nbEnumDevices == 0 ) {
			return STLINKIF_NO_STLINK;
		}
		if( ifStatus == STLINKIF_NO_ERR ) {
			// All is OK; no more necessary to do it again
			m_bDevInterfaceEnumerated = true;
		}
	}
	return ifStatus;
}

// Legacy management; prefer using GetDeviceInfo2
STLinkIf_StatusT STLinkInterface::GetDeviceInfo(int StlinkInstId, STLink_DeviceInfoT* pInfo, uint32_t InfoSize)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;

	if (IsLibraryLoaded() == true) {
		if (m_ifId == STLINK_TCP) {
			LogTrace("GetDeviceInfo2 must be used instead of GetDeviceInfo");
			return STLINKIF_NOT_SUPPORTED;
		}
#ifdef WIN32
		if (STLink_GetDeviceInfo == NULL) {
			// STLinkUSBDriver is too old
			return STLINKIF_NOT_SUPPORTED;
		}
#endif

		// Enumerate the current STLink interface if not already done
		ifStatus = EnumDevicesIfRequired(NULL, false, false);
		if (ifStatus != STLINKIF_NO_ERR) {
			return ifStatus;
		}

		if ((StlinkInstId < 0) || (((unsigned int)StlinkInstId) >= m_nbEnumDevices)) {
			LogTrace("%s Bad STLink instance id (%d > %d)", LogIfString[m_ifId], StlinkInstId, m_nbEnumDevices - 1);
			return STLINKIF_PARAM_ERR;
		}
		if (pInfo == NULL) {
			LogTrace("%s Bad parameter in GetDeviceInfo2 (NULL pointer)", LogIfString[m_ifId]);
			return STLINKIF_PARAM_ERR;
		}

		if (STLink_GetDeviceInfo(m_ifId, StlinkInstId, pInfo, InfoSize) != SS_OK) {
			return STLINKIF_GET_INFO_ERR;
		}
	}
	else {
		ifStatus = STLINKIF_DLL_ERR;
	}
	return ifStatus;
}

/**
 * @ingroup INTERFACE
 * @brief Return the #STLink_DeviceInfo2T data of the specified STLink device.
 *
 * @param[in]  StlinkInstId   STLink device index, from 0 to *pNumDevices-1
 *                            (value returned by STLinkInterface::EnumDevices())
 * @param[in]  InfoSize   Size of the allocated #STLink_DeviceInfo2T instance.
 *               Required for ascendant compatibility in case this structure grows in the future.
 *               If the allocated size is smaller than the size managed by STLinkUSBDriver library,
 *               returned data are limited to the given size. In case the allocated size
 *               is greater (caller more recent than STLinkUSBDriver library), exceeding fields are set to 0.
 * @param[out] pInfo  Pointer to the caller-allocated #STLink_DeviceInfo2T instance.
 *
 * @return STLinkInterface::EnumDevices() errors
 * @retval #STLINKIF_NOT_SUPPORTED If STLinkUSBDriver is too old for given m_ifId or m_ifId not supported yet
 * @retval #STLINKIF_GET_INFO_ERR Error in STLinkUSBDriver to retrieve the information
 * @retval #STLINKIF_PARAM_ERR Wrong StlinkInstId or NULL pInfo pointer
 * @retval #STLINKIF_DLL_ERR  STLinkUSBDriver library not loaded
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface::GetDeviceInfo2(int StlinkInstId, STLink_DeviceInfo2T *pInfo, uint32_t InfoSize)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;
	uint32_t Ret;

	if( IsLibraryLoaded() == true ) {
#ifdef WIN32
		if( STLink_GetDeviceInfo2 == NULL ) {
			// STLinkUSBDriver is too old
			return STLINKIF_NOT_SUPPORTED;
		}
#endif

		// Enumerate the current STLink interface if not already done
		ifStatus = EnumDevicesIfRequired(NULL, false, false);
		if( ifStatus != STLINKIF_NO_ERR ) {
			return ifStatus;
		}

		if( (StlinkInstId<0) || (((unsigned int)StlinkInstId) >= m_nbEnumDevices) ) {
			LogTrace("%s Bad STLink instance id (%d > %d)", LogIfString[m_ifId], StlinkInstId, m_nbEnumDevices-1);
			return STLINKIF_PARAM_ERR;
		}
		if( pInfo == NULL ) {
			LogTrace("%s Bad parameter in GetDeviceInfo2 (NULL pointer)", LogIfString[m_ifId]);
			return STLINKIF_PARAM_ERR;
		}

		Ret = STLink_GetDeviceInfo2(m_ifId, StlinkInstId, pInfo, InfoSize);
		if( (Ret != SS_OK) && (Ret != SS_TRUNCATED_DATA) ) {
			// SS_TRUNCATED_DATA returned in case STlinkUSBDriver API ver <4; ignore it at present
			// (as long as we don't use COM ports, we don't absolutely need the up-to-date library).
			// Instead, the use of COM ports should check m_libApiVer>=4
			return STLINKIF_GET_INFO_ERR;
		}
	} else {
		ifStatus = STLINKIF_DLL_ERR;
	}
	return ifStatus;
}
/*
 * @brief Called by StlinkDevice object, do not use directly.
 * Open USB connection with the STLink for the given USB interface.
 *
 * @param[in]  StlinkInstId   Instance ID in the list of enumerated STLink devices
 * @param[in]  StlinkIdTcp    Instance ID of the STLink device, for use in shared mode (ignored in direct mode),
 *                            equal to #STLink_DeviceInfo2T.StLinkUsbId.
 * @param[in]  bOpenExclusive false: shared between applications \n
 *                            true: exclusive to 1 application
 * @param[out] pHandle        Handle of the opened STLink device
 *
 * @return STLinkInterface::EnumDevices() errors
 * @retval #STLINKIF_CONNECT_ERR Wrong StlinkInstId or USB error
 * @retval #STLINKIF_NOT_SUPPORTED m_ifId not supported yet
 * @retval #STLINKIF_DLL_ERR STLinkUSBDriver library not loaded
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface::OpenDevice(int StlinkInstId, uint32_t StlinkIdTcp, bool bOpenExclusive, void **pHandle)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;

	if( IsLibraryLoaded() == true ) {
		// Enumerate the STLink interface if not already done
		ifStatus = EnumDevicesIfRequired(NULL, false, false);
		if( ifStatus != STLINKIF_NO_ERR ) {
			return ifStatus;
		}

		if( (m_ifId != STLINK_TCP) && 
		    ((StlinkInstId<0) || (((unsigned int)StlinkInstId) >= m_nbEnumDevices)) ) {
			LogTrace("%s Bad STLink instance id (%d > %d)", LogIfString[m_ifId], StlinkInstId, m_nbEnumDevices-1);
			return STLINKIF_PARAM_ERR;
		}
		// Open the device
		if (m_libApiVer >= STLINK_LIB_API_VERSION_MIN_FOR_TCP) {
			uint32_t status = SS_OK;
			if (m_ifId == STLINK_TCP) {
#ifdef WIN32
				if ((STLink_OpenDeviceTcp != NULL) && (StlinkIdTcp != 0)) {
#else
				if (StlinkIdTcp != 0) {
#endif
					status = STLink_OpenDeviceTcp(m_ifId, StlinkIdTcp, (bOpenExclusive == true) ? 1 : 0);
				} else if (StlinkIdTcp == 0) {
					LogTrace("GetDeviceInfo2 not called before OpenDeviceTcp");
					ifStatus = STLINKIF_CONNECT_ERR;
				} else { // STLink_OpenDeviceTcp == NULL
					ifStatus = STLINKIF_DLL_ERR;
				}
			} else {
				status = STLink_OpenDevice(m_ifId, StlinkInstId, (bOpenExclusive == true) ? 1 : 0, pHandle);
			}

			if (status != SS_OK) {
				// Dedicated error for V1 devices on Mac and Linux
				if (status == SS_DEVICE_NOT_SUPPORTED) {
					LogTrace("Device not supported on current platform");
					return STLINKIF_NOT_SUPPORTED;
				}
				LogTrace("% s STLink device USB connection failure", LogIfString[m_ifId]);
				return STLINKIF_CONNECT_ERR;
			}
		} else {
			STMass_Enum_GetDevice(StlinkInstId, &m_legacyDevice);
			if ((bOpenExclusive == true) && (STMass_OpenDeviceExclusive != NULL)) {
				// If asked and possible, open exclusively
				if (STMass_OpenDeviceExclusive(m_legacyDevice, pHandle) != SS_OK) {
					LogTrace("Failed opening ST-Link device in exclusive mode");
					return STLINKIF_CONNECT_ERR;
				}
			} else {
				// Otherwise open in shared mode (default historical case)
				if (STMass_OpenDevice(m_legacyDevice, pHandle) != SS_OK) {
					LogTrace("ST-Link device USB connection failure");
					return STLINKIF_CONNECT_ERR;
				}
			}
		}
	} else {
		ifStatus = STLINKIF_DLL_ERR;
	}
	return ifStatus;
}
/*
 * @brief Called by StlinkDevice object, do not use directly.
 * Opening routine based on the serial number of the STLink; to use in case StlinkInstId is no more
 * reliable (devices have been reenumerated between STLink instance selection and opening) or
 * to choose a specific STLink by its serial number.
 *
 * @param[in]  pSerialNumber  STLink serial number (ASCII).
 * @param[in]  bStrict 	 Used if STLink with required serial number is not found, but one other STLink is found:
 *                       choose if we open or not the other STLink. \n
 *                       If bStrict is true, the command will return #STLINKIF_STLINK_SN_NOT_FOUND if the
 *                       given SerialNumber is not found.\n
 *                       If bStrict is false and the given serial number is not found and one (and only one)
 *                       STLink is found, the command will attempt to open the STLink even if
 *                       the serial number does not match.
 * @return STLinkInterface::EnumDevices() errors
 * @retval #STLINKIF_CONNECT_ERR USB error
 * @retval #STLINKIF_STLINK_SN_NOT_FOUND Serial number not found
 * @retval #STLINKIF_PARAM_ERR if NULL pointer
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface::GetDeviceIdFromSerialNum(const char* pSerialNumber, bool bStrict, int* pStlinkInstId, uint32_t* pStlinkIdTcp, bool bForceRenum) {
	STLinkIf_StatusT ifStatus=STLINKIF_NO_ERR;
	int stlinkInstId;
	STLink_DeviceInfo2T devInfo2;
	char * pEnumUniqueId = devInfo2.EnumUniqueId;
	uint32_t stlinkIdTcp = 0; // Default unused

	if( !pSerialNumber || !pStlinkInstId || !pStlinkIdTcp ) {
		LogTrace("NULL pointer in GetDeviceIdFromSerialNum");
		return STLINKIF_PARAM_ERR;
	}

	// Enumerate the current STLink interface if not already done
	ifStatus = EnumDevicesIfRequired(NULL, bForceRenum, false);
	if( ifStatus != STLINKIF_NO_ERR ) {
		return ifStatus;
	}

	// Look for the given serialNumber
	for( stlinkInstId=0; (uint32_t)stlinkInstId<m_nbEnumDevices; stlinkInstId++ ) {
#ifdef WIN32
		if (STLink_GetDeviceInfo2 == NULL) {
			// Kept for backward compatibility with old dll not having GetDeviceInfo2
			STLink_DeviceInfoT devInfo;
			ifStatus = GetDeviceInfo(stlinkInstId, &devInfo, sizeof(devInfo));
			if (ifStatus == STLINKIF_NO_ERR) {
				// Change pointer to old struct
				pEnumUniqueId = devInfo.EnumUniqueId;
			}
		} else { //normal case use the new API GetDeviceInfo2
#endif
		ifStatus = GetDeviceInfo2(stlinkInstId, &devInfo2, sizeof(devInfo2));
		if (m_ifId == STLINK_TCP) {
			stlinkIdTcp = devInfo2.StLinkUsbId;
		}
#ifdef WIN32
		}
#endif
		if( ifStatus != STLINKIF_NO_ERR ) {
			// We failed retrieving the required information for this device ... but we can explore the other ones
			continue;
		}
		if( strcmp(pSerialNumber, pEnumUniqueId) == 0 ) {
			// Right STLink found
			*pStlinkInstId = stlinkInstId;
			*pStlinkIdTcp = stlinkIdTcp;
			return STLINKIF_NO_ERR;
		}
	}
	// If there, the asked serial number was not found
	if( (bStrict == false) && (m_nbEnumDevices==1) ) {
		// There is currently only one device connected, and the caller did not expected a full matching
		LogTrace("STLink serial number (%s) not found; opening the (lonely) connected STLink (SN=%s)",
			pSerialNumber, pEnumUniqueId);
		*pStlinkInstId = 0;
		*pStlinkIdTcp = stlinkIdTcp;
		return STLINKIF_NO_ERR;
	}
	LogTrace("STLink serial number (%s) not found; can not open.", pSerialNumber);
	return STLINKIF_STLINK_SN_NOT_FOUND;
}
/*
 * @brief Called by StlinkDevice object, do not use directly.
 * Close STLink USB communication, with the device instance that was opened by STLinkInterface::OpenDevice()
 *
 * @param[in]  pHandle        Handle of the opened STLink device (returned by STLinkInterface::OpenDevice())
 * @param[in]  stlinkIdTcp    0 (unused)
 *
 * @retval #STLINKIF_CLOSE_ERR Error at USB side
 * @retval #STLINKIF_NOT_SUPPORTED m_ifId not supported yet
 * @retval #STLINKIF_DLL_ERR
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface::CloseDevice(void *pHandle, uint32_t StlinkIdTcp)
{
	uint32_t status=SS_OK;
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;

	if( IsLibraryLoaded() == true ) {
		if (m_libApiVer >= STLINK_LIB_API_VERSION_MIN_FOR_TCP) {
			if (m_ifId == STLINK_TCP) {
#ifdef WIN32
				if (STLink_CloseDeviceTcp != NULL) {
#endif
					status = STLink_CloseDeviceTcp(StlinkIdTcp, CLOSE_TCP_AUTO);
					// When we close the ST-Link Device, TCP server may have been closed if we are the last
					// server TCP client, when server is closed enumeration is no more valid.
					m_bDevInterfaceEnumerated = false;
#ifdef WIN32
				} else {
					status = STLINKIF_DLL_ERR;
				}
#endif
			} else {
				if (pHandle != NULL) {
					status = STLink_CloseDevice(pHandle);
				}
			}
		} else {
			if (m_legacyDevice != NULL) {
				status = STMass_CloseDevice(m_legacyDevice, pHandle);
			}
		}
		if (status != SS_OK) {
				LogTrace("%s Error closing USB communication", LogIfString[m_ifId]);
				ifStatus = STLINKIF_CLOSE_ERR;
		}
	} else {
		ifStatus = STLINKIF_DLL_ERR;
	}
	return ifStatus;
}
/*
 * @brief Called by StlinkDevice object, do not use directly.
 * Send a command over the USB, wait for answer, requires the STLink to be already connected.
 *
 * @param[in]  pHandle        Handle of the opened STLink device (returned by STLinkInterface::OpenDevice())
 * @param[in]  stlinkIdTcp    0 (unused)
 * @param[in,out]  pDevReq    Command to send (contains a pointer to answer buffer if any)
 * @param[in]  UsbTimeoutMs   if 0 use default (5s) else use UsbTimeoutMs.
 *
 * @retval #STLINKIF_PARAM_ERR Null pointer error
 * @retval #STLINKIF_USB_COMM_ERR USB communication error
 * @retval #STLINKIF_NOT_SUPPORTED m_ifId not supported yet
 * @retval #STLINKIF_DLL_ERR
 * @retval #STLINKIF_NO_ERR If no error
 */
STLinkIf_StatusT STLinkInterface::SendCommand(void *pHandle,
                                              uint32_t StlinkIdTcp, STLink_DeviceRequestT *pDevReq,
                                              const uint16_t UsbTimeoutMs)
{
	uint32_t ret;
	uint32_t usbTimeout = DEFAULT_TIMEOUT;
	STLinkIf_StatusT ifStatus = STLINKIF_PARAM_ERR;

	if( pDevReq == NULL ) {
		return STLINKIF_PARAM_ERR;
	}

	// Create a Mutex to avoid concurrent access to STLink_SendCommand
	CSLocker locker(g_csInterface);

	if( IsLibraryLoaded() == true ) {
		// UsbTimeoutMs if 0 use default (5s) else use UsbTimeoutMs.
		if( UsbTimeoutMs != 0 ) {
			usbTimeout = (uint32_t) UsbTimeoutMs;
		}
		if (m_libApiVer >= STLINK_LIB_API_VERSION_MIN_FOR_TCP) {
			if (m_ifId == STLINK_TCP) {
#ifdef WIN32
				if ((STLink_SendCommandTcp != NULL) && (StlinkIdTcp != 0)) {
#else
				if (StlinkIdTcp != 0) {
#endif
					ret = STLink_SendCommandTcp(StlinkIdTcp, pDevReq, usbTimeout);
				} else if (StlinkIdTcp == 0) {
					LogTrace(" GetDeviceInfo2/OpenDevice must be called before sending command via stlink-TCP ");
					return STLINKIF_CONNECT_ERR;
				} else {
					//STLink_SendCommandTcp == NULL, tcp API missing in DLL
					return STLINKIF_DLL_ERR;
				}
			} else {
				ret = STLink_SendCommand(pHandle, pDevReq, usbTimeout);
			}
		} else {
			ret = STMass_SendCommand(m_legacyDevice, pHandle, pDevReq, usbTimeout);
		}

		if( ret != SS_OK ) {
			if (ret == SS_TCP_BUSY) {
				LogTrace("Shared command not done because resource owned by another client");
				return STLINKIF_TCP_BUSY;
			} else {
				LogTrace("%s USB communication error (%d) after target cmd %02hX %02hX %02hX %02hX %02hX %02hX %02hX %02hX %02hX %02hX",
					LogIfString[m_ifId], (int)ret,
					(unsigned short)pDevReq->CDBByte[0], (unsigned short)pDevReq->CDBByte[1], (unsigned short)pDevReq->CDBByte[2],
					(unsigned short)pDevReq->CDBByte[3], (unsigned short)pDevReq->CDBByte[4], (unsigned short)pDevReq->CDBByte[5],
					(unsigned short)pDevReq->CDBByte[6], (unsigned short)pDevReq->CDBByte[7], (unsigned short)pDevReq->CDBByte[8],
					(unsigned short)pDevReq->CDBByte[9]);
				ifStatus = STLINKIF_USB_COMM_ERR;
			}
		} else {
			ifStatus = STLINKIF_NO_ERR;
		}
	} else {
		ifStatus = STLINKIF_DLL_ERR;
	}
	return ifStatus;
}

#ifdef USING_ERRORLOG
/*
 * Associate files to be used for Error/Trace log (pErrLog must be initialized before)
 */
void STLinkInterface::BindErrLog(cErrLog *pErrLog)
{
	m_pErrLog = pErrLog;
}
#endif

// Set TCP server parameters used to start and/or connect to the TCP server (STLINK_TCP case)
// Note: TCP server supported on old FW but multiclient only starting FIRMWARE_JTAG_MIN_VER_FOR_TCP_AND_AP
STLinkIf_StatusT STLinkInterface::SetTcpServerParam(uint8_t paramNb, STLinkIf_TcpServerParamT* pParams)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;

	if ((paramNb < 3) || (pParams == NULL)) {
		//at least 3 params are used: ServerName PortName CmdLineOptions;
		ifStatus = STLINKIF_PARAM_ERR;
	}
	else {
		if (pParams->CmdLineOptions != NULL) {
#if defined(_MSC_VER) &&  (_MSC_VER >= 1400) /* VC8+ (VS2005) */
			strncpy_s(m_tcpServerOptionParams, MAX_TCP_OPTION_PARAM_SIZE, pParams->CmdLineOptions, MAX_TCP_OPTION_PARAM_SIZE - 1);
#else
			strncpy(m_tcpServerOptionParams, pParams->CmdLineOptions, MAX_TCP_OPTION_PARAM_SIZE);
#endif
		} else {
			STRCPY_S(m_tcpServerOptionParams, MAX_TCP_OPTION_PARAM_SIZE, DEFAULT_TCP_SERVER_OPTION);
		}
		// m_tcpConnectParams format is @serverName:portName or @serverName or :portName or null
		if ((pParams->ServerName != NULL) && (pParams->ServerName[0] != '\0')) {
			// write server name
			if (strlen(pParams->ServerName) > MAX_TCP_SERVER_PARAM_SIZE) {
				// The given name will be truncated: return an error
				ifStatus = STLINKIF_PARAM_ERR;
			}
#if defined(_MSC_VER) &&  (_MSC_VER >= 1400) /* VC8+ (VS2005) */
			// +1 for the added '@'; on Visual the given size does not include the space for EOS
			_snprintf_s(m_tcpConnectParams, MAX_TCP_CONN_PARAM_SIZE, MAX_TCP_SERVER_PARAM_SIZE + 1, "@%s", pParams->ServerName);
#else
			// snprintf automatically adds the EOS, but we must include it in the given size: +1 for the added '@', +1 for the EOS
			snprintf(m_tcpConnectParams, MAX_TCP_SERVER_PARAM_SIZE + 2, "@%s", pParams->ServerName);
#endif
		} else {
			m_tcpConnectParams[0] = '\0';
		}

		if ((pParams->PortName != NULL) && (pParams->PortName[0] != '\0')) {
			if (strlen(pParams->PortName) >= MAX_TCP_CONN_PARAM_SIZE - 1 - strlen(m_tcpConnectParams)) {
				// The given name will be truncated if we append it after a ':' => return an error
				ifStatus = STLINKIF_PARAM_ERR;
			}
			// intermediate string to avoid m_tcpConnectParams used as parameter and destination in snprintf
			char tmpStr[MAX_TCP_SERVER_PARAM_SIZE + 2]; // +2 for the added '@' and EOS
			//concatenate server and port
#if defined(_MSC_VER) &&  (_MSC_VER >= 1400) /* VC8+ (VS2005) */
			strcpy_s(tmpStr, MAX_TCP_SERVER_PARAM_SIZE + 2, m_tcpConnectParams);
			_snprintf_s(m_tcpConnectParams, MAX_TCP_CONN_PARAM_SIZE, MAX_TCP_CONN_PARAM_SIZE - 1, "%s:%s", tmpStr, pParams->PortName);
#else
			strcpy(tmpStr, m_tcpConnectParams);
			snprintf(m_tcpConnectParams, MAX_TCP_CONN_PARAM_SIZE, "%s:%s", tmpStr, pParams->PortName);
			m_tcpConnectParams[MAX_TCP_CONN_PARAM_SIZE - 1] = '\0'; // Not required, because snprintf added the EOS
#endif
		}
	}

	return ifStatus;
}

// Get the number of connected TCP client to the given ST-Link device (if stLinkUsbIdParam !=0).
STLinkIf_StatusT STLinkInterface::GetNumOfDeviceClientsTcp(uint32_t* pNum, uint32_t stLinkUsbIdParam)
{
	STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;
	// Set num of device to 0 in case of error
	*pNum = 0;

#ifdef WIN32 //Defined for applications for Win32 and Win64.
	if ((m_ifId == STLINK_TCP) && (STLink_GetNumOfDeviceClientsTcp != NULL)) {
#else
	if (m_ifId == STLINK_TCP) {
#endif
		if (stLinkUsbIdParam != 0) {
			*pNum = STLink_GetNumOfDeviceClientsTcp(stLinkUsbIdParam);
		} else {
			ifStatus = STLINKIF_NO_STLINK;
		}
	} else {
		ifStatus = STLINKIF_NOT_SUPPORTED;
	}
	return ifStatus;
}


/**********************************END OF FILE*********************************/

