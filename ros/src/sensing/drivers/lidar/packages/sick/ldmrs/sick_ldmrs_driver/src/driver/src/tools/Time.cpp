//
// Time.cpp
//


#include "Time.hpp"
#include <sys/time.h>
#include <time.h>
#include "toolbox.hpp"	// fuer "::toString()"

//
// Duration
//
TimeDuration::TimeDuration()
{
	m_duration = 0.0;
}


const double Time::m_secondFractionNTPtoNanoseconds (0.2328306436538696); // = 2^-32 * 1e9
const double Time::m_nanosecondsToSecondFractionNTP (4.29496729600000);   // = 2^32 * 1e-9
const UINT64 Time::secondsFrom1900to1970 (2208988800UL);

//
// Time
//
Time::Time()
{
	set(0.0);
}

Time::~Time()
{
}

//
// Returns the time as a fractional value with up to 4 fraction digits.
// Example: "1393926457.2141"
//
std::string Time::toString() const
{
	double t = (double)m_time.tv_sec + ((double)m_time.tv_usec / 1000000.0);
	return ::toString(t, 4);
}

//
// Returns the time as a readble string, in local time. This includes
// the microseconds.
// Example: "Tue Mar  4 10:47:37 2014 165751 us"
//
std::string Time::toLongString() const
{
	time_t seconds = m_time.tv_sec;
	struct tm* seconds_tm = localtime(&seconds);
	char* text = asctime(seconds_tm);
	std::string s = text;
	
	// Microseconds
	std::string us = "000000" + ::toString((UINT32)m_time.tv_usec);
	us = us.substr(us.length() - 6, 6);
	s += " " + us + " us";
	
	return s;
}

//
// 
//
void Time::set(double time)
{
	m_time.tv_sec = (UINT32)time;
	m_time.tv_usec = (time - (double)((UINT32)time)) * 1000000;
}

//
//
//
void Time::set(timeval time)
{
	m_time = time;
}


void Time::set(UINT64 ntpTime)
{
	set(static_cast<UINT64>(ntpTime >> 32), static_cast<UINT32>(ntpTime));
}

//
//
//
void Time::set(UINT64 ntpSeconds, UINT32 ntpFractionalSeconds)
{
//	const UINT32 startUnixEpoche = 2208988800; // seconds from 1.1.1900 to 1.1.1900
	m_time.tv_sec = ntpSeconds - secondsFrom1900to1970;
	m_time.tv_usec = ntpFractionalSeconds * m_secondFractionNTPtoNanoseconds / 1000;
}

/**
 * Zeit, in [s].
 */
double Time::seconds()
{
	double s = (double)m_time.tv_sec + ((double)m_time.tv_usec / 1000000);
	return s;
}

//
// Zeit(spanne), in [ms].
//
UINT32 Time::total_milliseconds()
{
	UINT32 ms = (m_time.tv_sec * 1000) + (m_time.tv_usec / 1000);
	return ms;
}



/**
 *
 */
Time& Time::operator+=(const Time& other)
{
	m_time.tv_usec += other.m_time.tv_usec;
	if (m_time.tv_usec > 1000000)
	{
		m_time.tv_sec++;
		m_time.tv_usec -= 1000000;
	}
	m_time.tv_sec += other.m_time.tv_sec;
	return *this;
}

/**
 *
 */
Time Time::operator+(const Time& other) const
{
	Time t;
	t.m_time.tv_sec = m_time.tv_sec + other.m_time.tv_sec;
	t.m_time.tv_usec = (m_time.tv_usec + other.m_time.tv_usec);
	if (t.m_time.tv_usec > 1000000)
	{
		t.m_time.tv_sec++;
		t.m_time.tv_usec -= 1000000;
	}
	return t;
}

/**
 * Result = (this) + Duration
 */
Time Time::operator+(const TimeDuration& dur) const
{
	Time td;
	td.set(dur.m_duration);
	
	Time t;
	t = *this + td;
	
	return t;
}


/**
 * 
 */
Time Time::now()
{
	Time t;
	gettimeofday(&(t.m_time), NULL);
	
	return t;
}

//
// True, wenn unser Zeitpunkt gleich oder spaeter als der andere ist.
//
bool Time::operator>=(const Time& other) const
{
	if (m_time.tv_sec > other.m_time.tv_sec)
	{
		return true;
	}
	else if ((m_time.tv_sec == other.m_time.tv_sec) &&
			 (m_time.tv_usec >= other.m_time.tv_usec))
	{
		return true;
	}
	
	return false;
}

//
// True, wenn unser Zeitpunkt frueher als der andere ist.
//
bool Time::operator<(const Time& other) const
{
	if (m_time.tv_sec < other.m_time.tv_sec)
	{
		// Unsere Sekunden sind kleiner
		return true;
	}
	else if ((m_time.tv_sec == other.m_time.tv_sec) &&
			 (m_time.tv_usec < other.m_time.tv_usec))
	{
		// Sekunden sind gleich, aber unsere usec sind kleiner
		return true;
	}
	
	return false;
}

//
// True, wenn unser Zeitpunkt gleich dem anderen ist.
//
bool Time::operator==(const Time& other) const
{
	if ((m_time.tv_sec == other.m_time.tv_sec) &&
		(m_time.tv_usec == other.m_time.tv_usec))
	{
		// Gleich
		return true;
	}
	
	return false;
}


Time Time::operator-(const Time& other) const
{
	Time t;
	if (m_time.tv_sec > other.m_time.tv_sec)
	{
		t.m_time.tv_sec = m_time.tv_sec - other.m_time.tv_sec;
		UINT32 offset = 0;
		if (m_time.tv_usec < other.m_time.tv_usec)
		{
			t.m_time.tv_sec -= 1;
			offset = 1000000;
		}
		t.m_time.tv_usec = (m_time.tv_usec + offset) - other.m_time.tv_usec;
	}
	else if (m_time.tv_sec == other.m_time.tv_sec)
	{
		t.m_time.tv_sec = 0;
		if (m_time.tv_usec < other.m_time.tv_usec)
		{
			t.m_time.tv_usec = 0;
		}
		else
		{
			t.m_time.tv_usec = m_time.tv_usec - other.m_time.tv_usec;
		}
	}
	else
	{
		t.m_time.tv_sec = 0;
		t.m_time.tv_usec = 0;
	}
	return t;
}

Time Time::operator-(const double& seconds) const
{
	Time t1, t2;
	t1.set(seconds);
	t2 = *this - t1;
	return t2;
/*	
	if (m_time.tv_sec > other.m_time.tv_sec)
	{
		t.m_time.tv_sec = m_time.tv_sec - other.m_time.tv_sec;
		UINT32 offset = 0;
		if (m_time.tv_usec < other.m_time.tv_usec)
		{
			t.m_time.tv_sec -= 1;
			offset = 1000000;
		}
		t.m_time.tv_usec = (m_time.tv_usec + offset) - other.m_time.tv_usec;
	}
	else if (m_time.tv_sec == other.m_time.tv_sec)
	{
		t.m_time.tv_sec = 0;
		if (m_time.tv_usec < other.m_time.tv_usec)
		{
			t.m_time.tv_usec = 0;
		}
		else
		{
			t.m_time.tv_usec = m_time.tv_usec - other.m_time.tv_usec;
		}
	}
	else
	{
		t.m_time.tv_sec = 0;
		t.m_time.tv_usec = 0;
	}
	return t;
*/
}

