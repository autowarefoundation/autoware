#include "angles.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


double Angle::norm(double a)
{
    a -= trunc(a / (ANGLE_2PI)) * (ANGLE_2PI);
    if (a < 0) a += (ANGLE_2PI);
    return a;
}
double Angle::diff(double a, double b)
{
  
  a = norm(a);
  b = norm(b);
  // fprintf(stderr,"a, b, = %.2f %.2f\n", a, b);
  double diff = a - b;
  if (fabs(diff) > M_PI) {
    diff -= copysign(2.0*M_PI,diff);
  }
  return diff;
}

double Angle::add(double a, double b)
{ 
  a = norm(a);
  b = norm(b);
  double sum = a + b;
  if (sum >= 2.0*M_PI) sum -=2.0*M_PI;
  
  return sum;
}



double Angle::R2D(double angle_rad)
{
  return ((angle_rad * 180.0) / M_PI);
}

double Angle::D2R(double angle_deg) 
{
  return ((angle_deg *M_PI) / 180.0);
}

