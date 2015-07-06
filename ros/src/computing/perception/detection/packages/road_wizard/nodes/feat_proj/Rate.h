/*
 * Rate.h
 *
 *  Created on: Mar 19, 2015
 *      Author: sujiwo
 */

#ifndef RATE_H_
#define RATE_H_

#include <unistd.h>
#include <string>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace boost::posix_time;


class Rate {
public:
	inline Rate (int hz)
	{
		assert (hz >= 1);
		float microsec = 1e6 / (float)hz;
		sleeptime = microseconds (microsec);
		lastUpdate = microsec_clock::local_time();
	}

	inline void sleep()
	{
		ptime curtime = microsec_clock::local_time();
		time_duration delay = curtime - lastUpdate;
		if (delay < sleeptime) {
			time_duration realSleepTime = sleeptime - delay;
			usleep (realSleepTime.total_microseconds());
		}
		lastUpdate = microsec_clock::local_time();
	}

private:
	ptime lastUpdate;
	time_duration sleeptime;
};

#endif /* RATE_H_ */
