#ifndef __EUCLIDEAN_SPACE_HPP
#define __EUCLIDEAN_SPACE_HPP

#include <cmath>
#include <iostream>

using namespace std;

namespace euclidean_space
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
class EuclideanSpace
{
private:
  EuclideanSpace(void){};

public:
  static double find_distance(point *_a, point *_b)
  {
    return std::hypot(std::hypot(_a->x - _b->x, _a->y - _b->y), _a->z - _b->z);
  }

  static double find_angle(point *_a, point *_b)
  {
    double _angle = std::atan2(_b->y - _a->y, _b->x - _a->x);
    if (_angle < 0.0)
      _angle = _angle + 2 * M_PI;

    return _angle * 360 / (2 * M_PI);
  }
};
}  // namespace euclidean_space

#endif
