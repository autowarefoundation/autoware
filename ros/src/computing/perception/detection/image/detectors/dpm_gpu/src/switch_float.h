/* typedef to switch float and double */
#define USE_FLOAT_AS_DECIMAL

#ifdef USE_FLOAT_AS_DECIMAL
typedef float FLOAT;
#ifdef FROM_GPU
#define sqrt sqrtf
#endif
#else
typedef double FLOAT;
#endif

#ifndef TVSUB
#define TVSUB
/*************************************/
/* for measurement */
#include <sys/time.h>
/* tvsub: ret = x - y. */
static inline void tvsub
(
 struct timeval *x,
 struct timeval *y,
 struct timeval *ret
 )
{
  ret->tv_sec = x->tv_sec - y->tv_sec;
  ret->tv_usec = x->tv_usec - y->tv_usec;
  if (ret->tv_usec < 0) {
    ret->tv_sec--;
    ret->tv_usec += 1000000;
  }
}
/* for measurement */
/*************************************/
#endif

extern struct timeval tv_memcpy_start, tv_memcpy_end;
extern float time_memcpy;
extern struct timeval tv_kernel_start, tv_kernel_end;
extern float time_kernel;















