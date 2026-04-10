/**
  ******************************************************************************
  * @file    criticalsectionlock.h
  * @author  GPM-AppliTools-HWBoards Team
  * @brief   Critical section header.
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
#ifndef _CRITICALSECTIONLOCK_H
#define _CRITICALSECTIONLOCK_H

// Uncomment the next line if the compiler supports C++11
//#define USE_RECURSIVE_MUTEX

/* Includes ------------------------------------------------------------------*/
#ifdef USE_RECURSIVE_MUTEX
#include <thread>
#include <mutex>
#else
#ifdef WIN32 //Defined for applications for Win32 and Win64.
#include "windows.h"
#else
#include <pthread.h>
#endif
#endif // USE_RECURSIVE_MUTEX

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Class -------------------------------------------------------------------- */
/*
 * Critical sections locker class.
 **/

#ifndef USE_RECURSIVE_MUTEX
// Critical section or Mutex Object type according to OS platform
#ifdef WIN32 // windows platform
#define CriticalSection_ObjectT CRITICAL_SECTION
#else // other platform
#define CriticalSection_ObjectT pthread_mutex_t
#endif
#endif // !USE_RECURSIVE_MUTEX

class CSLocker
{
	public:
#ifdef USE_RECURSIVE_MUTEX
	CSLocker(std::recursive_mutex& mutex);
#else
	CSLocker(CriticalSection_ObjectT& cs);
#endif

	~CSLocker(void);

	private:
#ifdef USE_RECURSIVE_MUTEX
	std::recursive_mutex&  m_mutex;
#else
	CriticalSection_ObjectT&  m_cs;
#endif
};

#endif //_CRITICALSECTIONLOCK_H
/**********************************END OF FILE*********************************/