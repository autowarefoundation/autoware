/*
 * File: Timestamp.h
 * Author: Dorian Galvez-Lopez
 * Date: March 2009
 * Description: timestamping functions
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_TIMESTAMP__
#define __D_TIMESTAMP__

#include <iostream>
using namespace std;

namespace DUtils {

/// Timestamp
class Timestamp
{
public:

  /// Options to initiate a timestamp
  enum tOptions
  {
    NONE = 0,
    CURRENT_TIME = 0x1,
    ZERO = 0x2
  };
  
public:
  
  /**
   * Creates a timestamp 
   * @param option option to set the initial time stamp
   */
	Timestamp(Timestamp::tOptions option = NONE);
	
	/**
	 * Destructor
	 */
	virtual ~Timestamp(void);

  /**
   * Says if the timestamp is "empty": seconds and usecs are both 0, as 
   * when initiated with the ZERO flag
   * @return true iif secs == usecs == 0
   */
  bool empty() const;

	/**
	 * Sets this instance to the current time
	 */
	void setToCurrentTime();

	/**
	 * Sets the timestamp from seconds and microseconds
	 * @param secs: seconds
	 * @param usecs: microseconds
	 */
	inline void setTime(unsigned long secs, unsigned long usecs){
		m_secs = secs;
		m_usecs = usecs;
	}
	
	/**
	 * Returns the timestamp in seconds and microseconds
	 * @param secs seconds
	 * @param usecs microseconds
	 */
	inline void getTime(unsigned long &secs, unsigned long &usecs) const
	{
	  secs = m_secs;
	  usecs = m_usecs;
	}

	/**
	 * Sets the timestamp from a string with the time in seconds
	 * @param stime: string such as "1235603336.036609"
	 */
	void setTime(const string &stime);
	
	/**
	 * Sets the timestamp from a number of seconds from the epoch
	 * @param s seconds from the epoch
	 */
	void setTime(double s);
	
	/**
	 * Returns this timestamp as the number of seconds in (long) float format
	 */
	double getFloatTime() const;

	/**
	 * Returns this timestamp as the number of seconds in fixed length string format
	 */
	string getStringTime() const;

	/**
	 * Returns the difference in seconds between this timestamp (greater) and t (smaller)
	 * If the order is swapped, a negative number is returned
	 * @param t: timestamp to subtract from this timestamp
	 * @return difference in seconds
	 */
	double operator- (const Timestamp &t) const;

	/** 
	 * Returns a copy of this timestamp + s seconds + us microseconds
	 * @param s seconds
	 * @param us microseconds
	 */
	Timestamp plus(unsigned long s, unsigned long us) const;

  /**
   * Returns a copy of this timestamp - s seconds - us microseconds
   * @param s seconds
   * @param us microseconds
   */
  Timestamp minus(unsigned long s, unsigned long us) const;

  /**
   * Adds s seconds to this timestamp and returns a reference to itself
   * @param s seconds
   * @return reference to this timestamp
   */
  Timestamp& operator+= (double s);
  
  /**
   * Substracts s seconds to this timestamp and returns a reference to itself
   * @param s seconds
   * @return reference to this timestamp
   */
  Timestamp& operator-= (double s);

	/**
	 * Returns a copy of this timestamp + s seconds
	 * @param s: seconds
	 */
	Timestamp operator+ (double s) const;

	/**
	 * Returns a copy of this timestamp - s seconds
	 * @param s: seconds
	 */
	Timestamp operator- (double s) const;

	/**
	 * Returns whether this timestamp is at the future of t
	 * @param t
	 */
	bool operator> (const Timestamp &t) const;

	/**
	 * Returns whether this timestamp is at the future of (or is the same as) t
	 * @param t
	 */
	bool operator>= (const Timestamp &t) const;

	/** 
	 * Returns whether this timestamp and t represent the same instant
	 * @param t
	 */
	bool operator== (const Timestamp &t) const;

	/**
	 * Returns whether this timestamp is at the past of t
	 * @param t
	 */
	bool operator< (const Timestamp &t) const;

	/**
	 * Returns whether this timestamp is at the past of (or is the same as) t
	 * @param t
	 */
	bool operator<= (const Timestamp &t) const;

  /**
   * Returns the timestamp in a human-readable string
   * @param machine_friendly if true, the returned string is formatted
   *   to yyyymmdd_hhmmss, without weekday or spaces
   * @note This has not been tested under Windows
   * @note The timestamp is truncated to seconds
   */
  string Format(bool machine_friendly = false) const;

	/**
	 * Returns a string version of the elapsed time in seconds, with the format
	 * xd hh:mm:ss, hh:mm:ss, mm:ss or s.us
	 * @param s: elapsed seconds (given by getFloatTime) to format
	 */
	static string Format(double s);
	

protected:
  /// Seconds
	unsigned long m_secs;	// seconds
	/// Microseconds
	unsigned long m_usecs;	// microseconds
};

}

#endif

