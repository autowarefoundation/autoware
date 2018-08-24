/**
 * \file Mutex.cpp
 */

#include "Mutex.hpp"
#include "errorhandler.hpp"

// ****************************************************************************
//  ScopedLock
// ****************************************************************************

// Konstruktor.
// Lock sperren.
ScopedLock::ScopedLock(Mutex* mutexPtr)
{
	m_mutexPtr = mutexPtr;
	if (m_mutexPtr != NULL)
	{
		m_mutexPtr->lock();
	}
}

// Destruktor.
// Lock wieder freigeben.
ScopedLock::~ScopedLock()
{
	if (m_mutexPtr != NULL)
	{
		m_mutexPtr->unlock();
	}
}



// ****************************************************************************
//  Mutex
// ****************************************************************************
Mutex::Mutex()
{
	pthread_mutex_init (&m_mutex, NULL);
}

Mutex::~Mutex()
{
}

void Mutex::lock()
{
	pthread_mutex_lock(&m_mutex);
}

void Mutex::unlock()
{
	pthread_mutex_unlock(&m_mutex);
}
