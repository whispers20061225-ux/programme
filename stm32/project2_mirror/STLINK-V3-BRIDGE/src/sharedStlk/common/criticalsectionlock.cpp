/**
  ******************************************************************************
  * @file    criticalsectionlock.cpp
  * @author  GPM-AppliTools-HWBoards Team
  * @brief   Critical section implementation.
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
#ifndef USE_RECURSIVE_MUTEX
#include "platform_include.h"
#endif

#include "criticalsectionlock.h"
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Class Functions Definition ------------------------------------------------*/
/*
 * Critical sections locker class.
 */
#ifdef USE_RECURSIVE_MUTEX
CSLocker::CSLocker(std::recursive_mutex& mutex): m_mutex(mutex)
{
	m_mutex.lock();
}

CSLocker::~CSLocker()
{
	m_mutex.unlock();
}
#else
CSLocker::CSLocker(CriticalSection_ObjectT& cs): m_cs(cs)
{
#ifdef WIN32 //Defined for applications for Win32 and Win64.
	EnterCriticalSection(&m_cs);
#else // other platform
	// Enter the critical section -- other threads are locked out
	pthread_mutex_lock(&m_cs);
#endif


}

CSLocker::~CSLocker()
{
#ifdef WIN32 //Defined for applications for Win32 and Win64.
	LeaveCriticalSection(&m_cs);
#else  // other platform
	// Leave the critical section -- other threads can now pthread_mutex_lock()
	pthread_mutex_unlock(&m_cs);
#endif
}
#endif // USE_RECURSIVE_MUTEX

/**********************************END OF FILE*********************************/