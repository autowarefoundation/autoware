#ifndef __AMATHUTILS_HPP
#define __AMATHUTILS_HPP

#include <cmath>
#include <iostream>

using namespace std;

namespace amathutils
{

class point
{
private:
public:
  double x, y, z;

  point(void)
  {
    x = y = z = 0.0;
  }

  point(double _x, double _y, double _z)
  {
    x = _x;
    y = _y;
    z = _z;
  }
};
double find_distance(point *_a, point *_b);
double find_angle(point *_a, point *_b);

inline double mps2kmph(double _mpsval)
{
  return (_mpsval * 0.36 );// mps * 60secs * 60minutes / 1000m
}

#define G_MPSS 9.80665 // m/s^2


inline double getGravityAcceleration(double acceleration_mpss)
{
	return acceleration_mpss /  G_MPSS;
}

inline double getAcceleration(double v0, double v, double x)
{
	return  (v * v - v0 * v0) / 2 / x;
}

inline double getTimefromAcceleration(double v0, double v, double a)
{
	return  (v - v0) / a;
} 
}
#endif
