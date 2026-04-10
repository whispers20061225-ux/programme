/**
  ******************************************************************************
  * @file    stlink_type.h
  * @author  MCD Development tools
  * @brief   Include file for standard typedef according to platform
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _STLINK_TYPE_H
#define _STLINK_TYPE_H

#ifdef WIN32  //Defined for applications for Win32 and Win64.
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// Insert your headers here
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers

// Including SDKDDKVer.h defines the highest available Windows platform.

// If you wish to build your application for a previous Windows platform, include WinSDKVer.h and
// set the _WIN32_WINNT macro to the platform you wish to support before including SDKDDKVer.h.
//#include <SDKDDKVer.h>


#include <windows.h>
#include <time.h>

//MSVC++ 10.0  _MSC_VER == 1600 (Visual Studio 2010 version 10.0)
//MSVC++ 11.0  _MSC_VER == 1700 (Visual Studio 2012 version 11.0)
//MSVC++ 12.0  _MSC_VER == 1800 (Visual Studio 2013 version 12.0)
#if defined(_MSC_VER) &&  (_MSC_VER >= 1800) // VC++ 12.0 (VS2013)
#include <cstdint>
#else
typedef signed char int8_t;
typedef short int16_t;
typedef int int32_t;

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;

// bool; In C++, the type bool is built in (although it is a 1-byte type in C++ mode, rather than a 4-byte type like in C mode).
// BOOL defined in windef.h
#endif

#define EXPORTED_API extern "C"

#else // !WIN32: Linux/MacOS
#include <stdint.h>
#include "limits.h"
#include <string.h>
#include "time.h"
#include <unistd.h>
#define Sleep(delay) usleep(1000*delay)

#define EXPORTED_API __attribute__ ((visibility ("default")))
#define MAX_PATH PATH_MAX
typedef void * HANDLE;
typedef void ** PHANDLE;
typedef void * PVOID;
typedef uint8_t BYTE;
typedef char CHAR;
typedef char TCHAR;
typedef uint16_t WORD; // unsigned 16
typedef uint32_t DWORD;  // unsigned 32 bits
typedef int32_t LONG;  // int for signed 32 bits; Note that "long" is signed 32 bits on Windows, signed 64 bits on Linux/MacOS => dangerous
typedef char * LPSTR;
#define INVALID_HANDLE_VALUE 0xFFFFFFFF
typedef bool BOOL;
#define TRUE true
#define FALSE false
#endif // WIN32

#endif //_STLINK_TYPE_H
/**********************************END OF FILE*********************************/