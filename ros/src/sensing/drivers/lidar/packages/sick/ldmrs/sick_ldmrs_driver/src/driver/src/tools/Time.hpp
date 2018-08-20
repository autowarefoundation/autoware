// Time.hpp
//
// created: June 6, 2011
//
// Mit "-lrt" linken!!
//

#ifndef TIME_HPP
#define TIME_HPP

#include "../BasicDatatypes.hpp"
#include <sys/time.h>
#include <time.h>

// Eine Zeitspanne, in [s]
class TimeDuration
{
public:
	TimeDuration();
	TimeDuration(double seconds) { m_duration = seconds; }
	
	void set(double seconds) { m_duration = seconds; }
	inline UINT32 total_milliseconds();
	inline TimeDuration& operator=(const double& seconds);
	
	double m_duration;	// Zeit, in [s]
};

// Fuer td = x;
inline TimeDuration& TimeDuration::operator=(const double& seconds)
{
	m_duration = seconds;
	return *this;
}

// Zeitspanne als [ms]
inline UINT32 TimeDuration::total_milliseconds()
{
	UINT32 ms = (UINT32)((m_duration * 1000.0) + 0.5);
	return ms;
}


class Time
{
public:
	Time();
	Time(timeval time);
	~Time();
	
	void set(double time);
	void set(timeval time);
	void set(UINT64 ntpSeconds, UINT32 ntpFractionalSeconds);
	void set(UINT64 ntpTime);
	double seconds();
	UINT32 total_milliseconds();
	
	static Time now();	// Returns the current time
	
	Time operator+(const TimeDuration& dur) const;
	Time& operator+=(const Time& other);
	Time operator+(const Time& other) const;
	Time operator-(const Time& other) const;
	Time operator-(const double& seconds) const;
	bool operator>=(const Time& other) const;
	bool operator<(const Time& other) const;
	bool operator==(const Time& other) const;
	
	std::string toString() const;
	std::string toLongString() const;

	static const UINT64 secondsFrom1900to1970;
	
private:
	timeval m_time;	// Zeit, in [s]

	static const double m_secondFractionNTPtoNanoseconds; // = 2^-32 * 1e9
	static const double m_nanosecondsToSecondFractionNTP;   // = 2^32 * 1e-9
};


#endif // TIME_HPP
