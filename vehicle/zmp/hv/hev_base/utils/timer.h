#ifndef __MYTIME_H__
#define __MYTIME_H__

#include <math.h>
#include <sys/time.h>
#include <pthread.h>

class Timer
{
private:
  
  pthread_mutex_t timer_mutex;
  double sum;
  int count;
  double startt;
  double stopt;
public:
  Timer();
  ~Timer() {}
  double getTime();
  void start();
  double stop();
  double getAve();
  void reset();
  void lock() {}// pthread_mutex_lock(&timer_mutex);}
  void unlock() {}// pthread_mutex_unlock(&timer_mutex);}
};

long long int TSTAMP();

#endif //__MYTIME_H__
