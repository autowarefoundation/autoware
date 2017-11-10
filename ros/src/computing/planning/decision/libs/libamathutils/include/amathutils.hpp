#ifndef __EUCLIDEAN_SPACE_HPP
#define __EUCLIDEAN_SPACE_HPP

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

} 

#endif
