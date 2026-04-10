/**
  ******************************************************************************
  * @file    platform_include.h
  * @author  MCD Development tools
  * @brief   Include precompiled header according to platform and its version
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


#ifndef _PLATFORM_INCLUDE_H
#define _PLATFORM_INCLUDE_H

#if defined(_MSC_VER) &&  (_MSC_VER >= 1000)
// Management of Microsoft librairies by Visual Studio
#if(_MSC_VER < 1920)
#include "stdafx.h"  // first include for windows visual studio older than 2019, comment this line if no stdafx.h is required in the project
#else
// If MFC used, first include for windows visual studio from 2019's version and newer is now pch.h
// define INCLUDE_PCH or INCLUDE_STDAFX in your preprocessor definition depending on your project MFC configuration
#if defined(INCLUDE_PCH)
#include "pch.h"
#elif defined(INCLUDE_STDAFX)
#include "stdafx.h"
#else
// Management of character set in Visual Studio without MFC
#include <tchar.h>
#endif
#endif
#else
#ifdef WIN32
// Management of character set outside Visual Studio
#include <tchar.h>
#else
// Redefinitions for Linux or OSX
#define _T(x) x
#endif
#endif// end _MSC_VER

#endif //_PLATFORM_INCLUDE_H

/**********************************END OF FILE*********************************/
