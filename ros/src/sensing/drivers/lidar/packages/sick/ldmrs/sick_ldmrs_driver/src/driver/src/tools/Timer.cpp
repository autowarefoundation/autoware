//
// Timer.cpp
//


#include "Timer.hpp"
//#include "Time.hpp"
#include <sys/time.h>


Timer::Timer()
{
	restart();
}

Timer::~Timer ()
{
}

//
// Startet den Watchdog-Timer, und legt das Timeout-Intervall fest.
//
void Timer::startWatchdog(TimeDuration watchdogTime)
{
	m_watchdogTime = watchdogTime;
	restart();
}

//
//
//
bool Timer::isElapsed()
{
	Time t = Time::now();
	
	if (t >= (m_startTime + m_watchdogTime))
	{
		// Timer ist abgelaufen
		return true;
	}
	
	return false;
}

//
// Alias fuer restart().
//
void Timer::reset()
{
	restart();
}

//
// Restart the timer. Also serves to reset the watchdog.
//
void Timer::restart()
{
	timeval t;
	gettimeofday(&t, NULL);
	m_startTime.set(t);
}

/**
 * "Jetzt"
 */
/*
timeval Timer::now()
{
	timeval now;
	gettimeofday(&now, NULL);
	return now;
}
*/

/**
 * Elapsed time since last start or restart, in [s].
 */
TimeDuration Timer::elapsed () const
{
//	timeval now;
//	UINT32 seconds, useconds;
	TimeDuration duration;
	
	Time now = Time::now();	// gettimeofday(&now, NULL);
	Time diff = now - m_startTime;
//	seconds  = now.tv_sec  - m_startTime.m_time.tv.sec;
//	useconds = now.tv_usec - m_startTime.tv_usec;
	duration = diff.seconds();	//  (double)seconds + ((double)useconds / 1000000.0);
	return duration;
}

/**
 * Elapsed time since last restart, in [ms]
 */
UINT32 Timer::elapsedMilliseconds () const
{
	TimeDuration elapsedTime = elapsed();
	UINT32 ms = elapsedTime.total_milliseconds();
	return ms;
}

/**
 * Elapsed time since last restart, in [us]
 */
UINT32 Timer::elapsedMicroseconds () const
{
	TimeDuration elapsedTime = elapsed();
	UINT32 us = (UINT32)((elapsedTime.m_duration * 1000000.0) + 0.5);
	return us;
}


/**
 * Elapsed time since last call to this function, in [ms].
 * This function may be called without restarts as it measures the time between consecutive calls
 * to this function.
 * Usage:
 *
 *		t;
 *		// ...
 *		t.elapsedMillisecondsSinceLastCall();
 *		// ... functions to be timed ...
 *		UINT32 time1 = t.elapsedMillisecondsSinceLastCall();
 *		// ... next functions to be timed ...
 *		UINT32 time2 = t.elapsedMillisecondsSinceLastCall();
 *      // ...
 *
 * Thats it!
 */
UINT32 Timer::elapsedMillisecondsSinceLastCall()
{
	UINT32 ms = elapsedMilliseconds();
	restart();
	return ms;
}


/**
 * Documentation see "elapsedMillisecondsSinceLastCall()", but it measures microseconds instead.
 */
UINT32 Timer::elapsedMicrosecondsSinceLastCall()
{
	UINT32 us = elapsedMicroseconds();
	restart();
	return us;
}

