#include "timer.h"
#include <stdio.h>
#include <math.h>
#include <sys/time.h>


extern long long int TSTAMP() { Timer t; return ((long long int)(t.getTime()));}


Timer::Timer() {
 
  reset();
 
}

void Timer::start(){
  // fprintf(stderr,"Timer::start()\n");

  // fprintf(stderr,"getting time\n");
  startt = getTime();
  // fprintf(stderr,"after getting time\n");
 
}

double Timer::stop() {
  double t;
   
  stopt = getTime();
  // lock();
  t = stopt - startt;
  // fprintf(stderr,"time = %.3f\n",t);
  sum+=t;
  count++;
  // unlock();
  return(t);
}

double Timer::getAve(){
  double res;
  
  // lock();
  res = sum/((double)(count));
  // unlock();
  return res;
}

void Timer::reset(){

   //  lock();
 
  count = 0;
  sum = 0.0; 
 

  // unlock();

}

double Timer::getTime()
{
  /* returns time in milliseconds */
  struct timeval cur_time;
  struct timezone ttz;
  double t;
  // lock();
  gettimeofday(&cur_time, &ttz);
  t = ((cur_time.tv_sec * 1000.0) +
       (cur_time.tv_usec) / 1000.0);
  // unlock();
  return t;
}

/*****************************************************************/
