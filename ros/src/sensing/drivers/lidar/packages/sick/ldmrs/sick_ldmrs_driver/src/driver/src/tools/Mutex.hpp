/**
 * \file Mutex.hpp
 */

#ifndef MUTEX_HPP
#define MUTEX_HPP

#include "../BasicDatatypes.hpp"
#include <pthread.h>



//
// Mutex class
//
class Mutex
{
public:
	Mutex();
	~Mutex();

	void lock();
	void unlock();

private:
	pthread_mutex_t m_mutex;
};



//
// Scoped Lock.
// Zerstoert das Mutex automatisch.
//
class ScopedLock
{
public:
	ScopedLock(Mutex* mutexPtr);
	~ScopedLock();
private:
	Mutex* m_mutexPtr;
};


#endif // MUTEX_HPP
