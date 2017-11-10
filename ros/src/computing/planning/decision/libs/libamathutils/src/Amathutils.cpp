#include "amathutils.hpp"

namespace amathutils
{
  double find_distance(point *_a, point *_b)
  {
    return std::hypot(std::hypot(_a->x - _b->x, _a->y - _b->y), _a->z - _b->z);
  }

  double find_angle(point *_a, point *_b)
  {
    double _angle = std::atan2(_b->y - _a->y, _b->x - _a->x);
    if (_angle < 0.0)
      _angle = _angle + 2 * M_PI;

    return _angle * 360 / (2 * M_PI);
  }



} 
