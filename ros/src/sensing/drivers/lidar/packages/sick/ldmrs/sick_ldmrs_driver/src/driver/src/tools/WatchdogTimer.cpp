// WatchdogTimer.cpp
// Copyright (c) SICK AG
// created: 06.06.2011, Willhoeft

#include "WatchdogTimer.hpp"
#include "Timer.hpp"


/**
 * Konstruktor.
 *
 * Der eigentliche Ablauf wird mit start() gestartet!
 */
WatchdogTimer::WatchdogTimer()
{
	m_timeoutInterval.m_duration = 0.0;
	m_nextElapseTime = Time::now() + m_timeoutInterval;
}

WatchdogTimer::~WatchdogTimer()
{
}


/**
 * Start the Timer.
 */
void WatchdogTimer::start(const TimeDuration interval)
{
	m_timeoutInterval = interval;
	reset();
}

/**
 * Abfrage, ob der Timer abgelaufen ist.
 */
bool WatchdogTimer::isElapsed()
{
	if (Time::now() >= m_nextElapseTime)
	{
		// Timer ist abgelaufen
		return true;
	}

	// Timer ist noch nicht abgelaufen
	return false;
}

/**
 * OK-Meldung der Applikation zum Ruecksetzen des Timers.
 */
void WatchdogTimer::reset()
{
	// Naechste Ablaufzeit setzen
	m_nextElapseTime = Time::now() + m_timeoutInterval;
}

