// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/universe_utils/geometry/sat_2d.hpp"

namespace autoware::universe_utils::sat
{

namespace
{
/// @brief calculate the edge normal of two points
Point2d edge_normal(const Point2d & p1, const Point2d & p2)
{
  return {p2.y() - p1.y(), p1.x() - p2.x()};
}

/// @brief project a polygon onto an axis and return the minimum and maximum values
std::pair<double, double> project_polygon(const Polygon2d & poly, const Point2d & axis)
{
  double min = poly.outer()[0].dot(axis);
  double max = min;
  for (const auto & point : poly.outer()) {
    double projection = point.dot(axis);
    if (projection < min) {
      min = projection;
    }
    if (projection > max) {
      max = projection;
    }
  }
  return {min, max};
}

/// @brief check if two projections overlap
bool projections_overlap(
  const std::pair<double, double> & proj1, const std::pair<double, double> & proj2)
{
  return proj1.second >= proj2.first && proj2.second >= proj1.first;
}

/// @brief check is all edges of a polygon can be separated from the other polygon with a separating
/// axis
bool has_no_separating_axis(const Polygon2d & polygon, const Polygon2d & other)
{
  for (size_t i = 0; i < polygon.outer().size(); ++i) {
    const size_t next_i = (i + 1) % polygon.outer().size();
    const Point2d edge = edge_normal(polygon.outer()[i], polygon.outer()[next_i]);
    const auto projection1 = project_polygon(polygon, edge);
    const auto projection2 = project_polygon(other, edge);
    if (!projections_overlap(projection1, projection2)) {
      return false;
    }
  }
  return true;
}
}  // namespace

/// @brief check if two convex polygons intersect using the SAT algorithm
/// @details this function uses the Separating Axis Theorem (SAT) to determine if two convex
/// polygons intersect. If projections overlap on all tested axes, the function returns `true`;
/// otherwise, it returns `false`. Note that touching polygons (e.g., at a point or along an edge)
/// will be considered as not intersecting.
bool intersects(const Polygon2d & convex_polygon1, const Polygon2d & convex_polygon2)
{
  return has_no_separating_axis(convex_polygon1, convex_polygon2) &&
         has_no_separating_axis(convex_polygon2, convex_polygon1);
}

}  // namespace autoware::universe_utils::sat
