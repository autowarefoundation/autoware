// Copyright 2024 Tier IV, Inc.
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

#include "autoware/universe_utils/geometry/gjk_2d.hpp"

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <boost/geometry/algorithms/equals.hpp>

namespace autoware::universe_utils::gjk
{

namespace
{
/// @brief structure with all variables updated during the GJK loop
/// @details for performance we only want to reserve their space in memory once
struct SimplexSearch
{
  // current triangle simplex
  Point2d a;
  Point2d b;
  Point2d c;
  Point2d co;                // vector from C to the origin
  Point2d ca;                // vector from C to A
  Point2d cb;                // vector from C to B
  Point2d ca_perpendicular;  // perpendicular to CA
  Point2d cb_perpendicular;  // perpendicular to CB
  Point2d direction;         // current search direction
};

/// @brief calculate the dot product between 2 points
double dot_product(const Point2d & p1, const Point2d & p2)
{
  return p1.x() * p2.x() + p1.y() * p2.y();
}

/// @brief calculate the index of the furthest polygon vertex in the given direction
size_t furthest_vertex_idx(const Polygon2d & poly, const Point2d & direction)
{
  auto furthest_distance = dot_product(poly.outer()[0], direction);
  size_t furthest_idx = 0UL;
  for (auto i = 1UL; i < poly.outer().size(); ++i) {
    const auto distance = dot_product(poly.outer()[i], direction);
    if (distance > furthest_distance) {
      furthest_distance = distance;
      furthest_idx = i;
    }
  }
  return furthest_idx;
}

/// @brief calculate the next Minkowski difference vertex in the given direction
Point2d support_vertex(const Polygon2d & poly1, const Polygon2d & poly2, const Point2d & direction)
{
  const auto opposite_direction = Point2d(-direction.x(), -direction.y());
  const auto idx1 = furthest_vertex_idx(poly1, direction);
  const auto idx2 = furthest_vertex_idx(poly2, opposite_direction);
  return Point2d(
    poly1.outer()[idx1].x() - poly2.outer()[idx2].x(),
    poly1.outer()[idx1].y() - poly2.outer()[idx2].y());
}

/// @brief return true if both points are in the same direction
bool same_direction(const Point2d & p1, const Point2d & p2)
{
  return dot_product(p1, p2) > 0.0;
}

/// @brief return the triple cross product of the given points
Point2d cross_product(const Point2d & p1, const Point2d & p2, const Point2d & p3)
{
  const auto tmp = p1.x() * p2.y() - p1.y() * p2.x();
  return Point2d(-p3.y() * tmp, p3.x() * tmp);
}

/// @brief update the search simplex and search direction to try to surround the origin
bool update_search_simplex_and_direction(SimplexSearch & search)
{
  bool continue_search = false;
  search.co.x() = -search.c.x();
  search.co.y() = -search.c.y();
  search.ca.x() = search.a.x() - search.c.x();
  search.ca.y() = search.a.y() - search.c.y();
  search.cb.x() = search.b.x() - search.c.x();
  search.cb.y() = search.b.y() - search.c.y();
  search.ca_perpendicular = cross_product(search.cb, search.ca, search.ca);
  search.cb_perpendicular = cross_product(search.ca, search.cb, search.cb);
  if (same_direction(search.ca_perpendicular, search.co)) {
    search.b.x() = search.c.x();
    search.b.y() = search.c.y();
    search.direction.x() = search.ca_perpendicular.x();
    search.direction.y() = search.ca_perpendicular.y();
    continue_search = true;
  } else if (same_direction(search.cb_perpendicular, search.co)) {
    search.a.x() = search.c.x();
    search.a.y() = search.c.y();
    search.direction.x() = search.cb_perpendicular.x();
    search.direction.y() = search.cb_perpendicular.y();
    continue_search = true;
  }
  return continue_search;
}
}  // namespace

/// @brief return true if the two given polygons intersect
/// @details if the intersection area is 0 (e.g., only one point or one edge intersect), the return
/// value is false
bool intersects(const Polygon2d & convex_polygon1, const Polygon2d & convex_polygon2)
{
  if (convex_polygon1.outer().empty() || convex_polygon2.outer().empty()) {
    return false;
  }
  if (boost::geometry::equals(convex_polygon1, convex_polygon2)) {
    return true;
  }

  SimplexSearch search;
  search.direction = {1.0, 0.0};
  search.a = support_vertex(convex_polygon1, convex_polygon2, search.direction);
  search.direction = {-search.a.x(), -search.a.y()};
  search.b = support_vertex(convex_polygon1, convex_polygon2, search.direction);
  if (dot_product(search.b, search.direction) <= 0.0) {  // the Minkowski difference does not cross
                                                         // the origin
    return false;                                        // no collision
  }
  Point2d ab = {search.b.x() - search.a.x(), search.b.y() - search.a.y()};
  Point2d ao = {-search.a.x(), -search.a.y()};
  search.direction = cross_product(ab, ao, ab);
  bool continue_search = true;
  while (continue_search) {
    search.c = support_vertex(convex_polygon1, convex_polygon2, search.direction);
    if (!same_direction(search.c, search.direction)) {  // no more vertex in the search direction
      return false;                                     // no collision
    }
    continue_search = update_search_simplex_and_direction(search);
  }
  return true;
}
}  // namespace autoware::universe_utils::gjk
