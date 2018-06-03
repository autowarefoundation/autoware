// WatchdogTimer.hpp
// Copyright (c) SICK AG
// created: 06.06.2011

#ifndef WATCHDOGTIMER_HPP
#define WATCHDOGTIMER_HPP

#include "../BasicDatatypes.hpp"
//#include <sys/time.h>
#include "Time.hpp"

/// A timer object to compare elapsed time against a timeout value.
//
//
//
class WatchdogTimer
{
public:
	/// Constructor
	WatchdogTimer();

	/// Destructor
	virtual ~WatchdogTimer();

	void reset();	/// Restarts the timer. Call this as "ok" signal.

	void start(UINT32 seconds);	// Initializes and starts the timer.

	/// Initializes and starts the timer. interval = time in [s]
	void start(const TimeDuration interval);

	bool isElapsed();

private:
	Time m_nextElapseTime;
	TimeDuration m_timeoutInterval;
};


#endif // WATCHDOGTIMER_HPP
