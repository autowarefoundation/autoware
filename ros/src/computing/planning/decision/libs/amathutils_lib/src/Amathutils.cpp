#include "amathutils_lib/amathutils.hpp"

namespace amathutils
{
double find_distance(point *_a, point *_b)
{
  return std::hypot(std::hypot(_a->x - _b->x, _a->y - _b->y), _a->z - _b->z);
}
double find_distance(point &_a, point &_b)
{
  return std::hypot(std::hypot(_a.x - _b.x, _a.y - _b.y), _a.z - _b.z);
}

double find_angle(point *_a, point *_b)
{
  double _angle = std::atan2(_b->y - _a->y, _b->x - _a->x);
  if (_angle < 0.0)
    _angle = _angle + 2 * M_PI;

  return _angle * 360 / (2 * M_PI);
}

bool isIntersectLine(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y, double p4x, double p4y)
{
  double ta = (p3x - p4x) * (p1y - p3y) + (p3y - p4y) * (p3x - p1x);
  double tb = (p3x - p4x) * (p2y - p3y) + (p3y - p4y) * (p3x - p2x);
  double tc = (p1x - p2x) * (p3y - p1y) + (p1y - p2y) * (p1x - p3x);
  double td = (p1x - p2x) * (p4y - p1y) + (p1y - p2y) * (p1x - p4x);

  if (tc * td < 0 && ta * tb < 0)
    return true;
  else
    return false;
}
#define LEFT 1
#define RIGHT -1
#define ONLINE 0

int isPointLeftFromLine(double p1x, double p1y, double line_p1x, double line_p1y, double line_p2x, double line_p2y)
{
  double n = p1x * (line_p1y - line_p2y) + line_p1x * (line_p2y - p1y) + line_p2x * (p1y - line_p1y);

  return n > 0 ? 1 : n < 0 ? -1 : 0;
}
}
