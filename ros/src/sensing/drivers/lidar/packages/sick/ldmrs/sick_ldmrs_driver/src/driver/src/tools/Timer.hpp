// Timer.hpp
//
// created: June 6, 2011
//
// Mit "-lrt" linken!!
//

#ifndef TIMER_HPP
#define TIMER_HPP

#include "../BasicDatatypes.hpp"
#include "Time.hpp"

/// A timer object to measure elapsed time.
/**
 * This class is intended for simple timing diagnostics
 * during debug phase. For a more accurate profiling of
 * your code use a profiler like \em valgrind or similar.
 *
 * Due to accuracy differences on various platforms for
 * Windows PCs and Linux PCs implementations and accuracy
 * may differ.
 */
class Timer
{
public:
	/// Constructor
	/** The timer is started, with the construction of an instance. */
	Timer();

	/// Destructor
	virtual ~Timer ();

	/// Restarts the timer.
	void restart();

	// Starts the watchdog timer, with the given timeout interval.
	// Note that no action is taken, but the timer has to be queried using
	// isElapsed().
	void startWatchdog(TimeDuration watchdogTime);

	// Returns true if the given timeout interval has elapsed.
	bool isElapsed();

	// Restarts the timer.
	void reset();

	// Returns the elapsed time in seconds since the timer was started or restarted.
	TimeDuration elapsed () const;
//	double elapsed () const;

	/// Returns the elapsed time in milliseconds since the timer was started or restarted.
	UINT32 elapsedMilliseconds () const;

	/// Returns the elapsed time in microseconds since the timer was started or restarted.
	UINT32 elapsedMicroseconds () const;

	UINT32 elapsedMillisecondsSinceLastCall();	///< Elapsed ms since the last call of this function.
	UINT32 elapsedMicrosecondsSinceLastCall();	///< Elapsed us since the last call of this function.
	
//	static timeval now();	// Returns the current time

private:
	Time m_startTime;
	TimeDuration m_watchdogTime;	// Zeitintervall des Watchdog
};

#endif // TIMER_HPP
