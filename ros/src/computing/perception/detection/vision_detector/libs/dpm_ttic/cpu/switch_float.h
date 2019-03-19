#ifndef _SWITCH_FLOAT_H_
#define _SWITCH_FLOAT_H_

/* typedef to switch float and double */
#define FLOAT_IS_float
typedef float FLOAT;
//typedef double FLOAT;

#ifndef TVSUB
#define TVSUB

/* for measurement */
#include <sys/time.h>
/* tvsub: ret = x - y. */
static inline void tvsub(
	struct timeval *x,
	struct timeval *y,
	struct timeval *ret)
{
	ret->tv_sec = x->tv_sec - y->tv_sec;
	ret->tv_usec = x->tv_usec - y->tv_usec;
	if (ret->tv_usec < 0) {
		ret->tv_sec--;
		ret->tv_usec += 1000000;
	}
}
/* for measurement */
#endif

#endif /* _SWITCH_FLOAT_H_ */
