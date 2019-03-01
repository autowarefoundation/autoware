#pragma once

#include <cmath>
#include <amathutils_lib/vector2d.hpp>
#include <vector>

namespace amathutils
{
class Polygon2d
{
private:
  std::vector<Vector2d> v_point_;

public:
  Polygon2d();
  Polygon2d(const std::vector<Vector2d> &_v_point);
  virtual ~Polygon2d(){};
  static bool isInPolygon(const std::vector<Vector2d> &v_point, const Vector2d &point, const bool boundary = true);
  void setPolygon(const std::vector<Vector2d> &v_point);
  bool isInPolygon(const Vector2d &point, const bool boundary = true);
};
} // namespace amathutils