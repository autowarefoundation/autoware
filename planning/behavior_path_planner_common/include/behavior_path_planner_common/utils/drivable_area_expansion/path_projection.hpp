// Copyright 2023 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__PATH_PROJECTION_HPP_
#define BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__PATH_PROJECTION_HPP_

#include "behavior_path_planner_common/utils/drivable_area_expansion/types.hpp"

#include <interpolation/linear_interpolation.hpp>

#include <boost/geometry/algorithms/distance.hpp>

#include <limits>

namespace drivable_area_expansion
{
/// @brief project a point to a segment
/// @param p point to project on the segment
/// @param p1 first segment point
/// @param p2 second segment point
/// @return projected point and corresponding distance
inline PointDistance point_to_segment_projection(
  const Point2d & p, const Point2d & p1, const Point2d & p2)
{
  const Point2d p2_vec = {p2.x() - p1.x(), p2.y() - p1.y()};
  const Point2d p_vec = {p.x() - p1.x(), p.y() - p1.y()};

  const auto c1 = boost::geometry::dot_product(p_vec, p2_vec);
  if (c1 <= 0) return {p1, boost::geometry::distance(p, p1)};

  const auto c2 = boost::geometry::dot_product(p2_vec, p2_vec);
  if (c2 <= c1) return {p2, boost::geometry::distance(p, p2)};

  const auto projection = p1 + (p2_vec * c1 / c2);
  const auto projection_point = Point2d{projection.x(), projection.y()};
  return {projection_point, boost::geometry::distance(p, projection_point)};
}

/// @brief project a point to a line
/// @param p point to project on the line
/// @param p1 first line point
/// @param p2 second line point
/// @return projected point and corresponding distance
inline PointDistance point_to_line_projection(
  const Point2d & p, const Point2d & p1, const Point2d & p2)
{
  const Point2d p2_vec = {p2.x() - p1.x(), p2.y() - p1.y()};
  const Point2d p_vec = {p.x() - p1.x(), p.y() - p1.y()};

  const auto c1 = boost::geometry::dot_product(p_vec, p2_vec);
  const auto c2 = boost::geometry::dot_product(p2_vec, p2_vec);
  const auto projection = p1 + (p2_vec * c1 / c2);
  const auto projection_point = Point2d{projection.x(), projection.y()};
  return {projection_point, boost::geometry::distance(p, projection_point)};
}

/// @brief project a point to a linestring
/// @param p point to project
/// @param ls linestring
/// @return projected point, corresponding distance, and arc length along the linestring
inline Projection point_to_linestring_projection(const Point2d & p, const LineString2d & ls)
{
  Projection closest;
  closest.distance = std::numeric_limits<double>::max();
  auto arc_length = 0.0;
  for (auto ls_it = ls.begin(); std::next(ls_it) != ls.end(); ++ls_it) {
    const auto pd = point_to_segment_projection(p, *ls_it, *(ls_it + 1));
    if (pd.distance < closest.distance) {
      closest.projected_point = pd.point;
      closest.distance = pd.distance;
      closest.arc_length = arc_length + boost::geometry::distance(*ls_it, pd.point);
    }
    arc_length += boost::geometry::distance(*ls_it, *std::next(ls_it));
  }
  return closest;
}

/// @brief calculate the normal to a vector at a given distance
/// @param p1 first vector point
/// @param p2 second vector point
/// @param dist distance
/// @return point p such that (p1,p) is orthogonal to (p1,p2) at the given distance
inline Point2d normal_at_distance(const Point2d & p1, const Point2d & p2, const double dist)
{
  auto base = p1;
  auto normal_vector = p2;
  boost::geometry::subtract_point(normal_vector, base);
  boost::geometry::detail::vec_normalize(normal_vector);
  boost::geometry::multiply_value(normal_vector, dist);
  return Point2d{base.x() - normal_vector.y(), base.y() + normal_vector.x()};
}

/// @brief interpolate between two points
/// @param a first point
/// @param b second point
/// @param ratio interpolation ratio such that 0 yields a, and 1 yields b
/// @return point interpolated between a and b as per the given ratio
inline Point2d lerp_point(const Point2d & a, const Point2d & b, const double ratio)
{
  return {interpolation::lerp(a.x(), b.x(), ratio), interpolation::lerp(a.y(), b.y(), ratio)};
}

/// @brief calculate the point with distance and arc length relative to a linestring
/// @param ls reference linestring
/// @param arc_length arc length along the reference linestring of the resulting point
/// @param distance distance from the reference linestring of the resulting point
/// @return point at the distance and arc length relative to the reference linestring
inline Segment2d linestring_to_point_projection(
  const LineString2d & ls, const double arc_length, const double distance)
{
  if (ls.empty()) return Segment2d{};
  if (ls.size() == 1lu) return {ls.front(), ls.front()};
  auto ls_iterator = ls.begin();
  auto prev_arc_length = 0.0;
  auto curr_arc_length = boost::geometry::distance(*ls_iterator, *std::next(ls_iterator));
  ++ls_iterator;
  while (curr_arc_length < arc_length && std::next(ls_iterator) != ls.end()) {
    prev_arc_length = curr_arc_length;
    curr_arc_length += boost::geometry::distance(*ls_iterator, *std::next(ls_iterator));
    ++ls_iterator;
  }

  const auto lerp_ratio = (arc_length - prev_arc_length) / (curr_arc_length - prev_arc_length);
  const auto base_point = lerp_point(*std::prev(ls_iterator), *ls_iterator, lerp_ratio);
  if (distance == 0.0) return {base_point, base_point};
  if (lerp_ratio >= 1.0)  // base point is beyond the 2nd segment point -> calculate normal in
                          // the other direction
    return {base_point, normal_at_distance(base_point, *std::prev(ls_iterator), -distance)};
  return {base_point, normal_at_distance(base_point, *ls_iterator, distance)};
}

/// @brief create a sub linestring between the given arc lengths
/// @param ls input linestring
/// @param from_arc_length arc length of the first point of the sub linestring
/// @param to_arc_length arc length of the last point of the sub linestring
/// @return sub linestring
inline LineString2d sub_linestring(
  const LineString2d & ls, const double from_arc_length, const double to_arc_length)
{
  LineString2d sub_ls;
  if (from_arc_length >= to_arc_length || ls.empty())
    throw(std::runtime_error("sub_linestring: bad inputs"));

  auto ls_iterator = ls.begin();
  auto prev_arc_length = 0.0;
  auto curr_arc_length = boost::geometry::distance(*ls_iterator, *std::next(ls_iterator));
  ++ls_iterator;
  while (curr_arc_length < from_arc_length && std::next(ls_iterator) != ls.end()) {
    prev_arc_length = curr_arc_length;
    curr_arc_length += boost::geometry::distance(*ls_iterator, *std::next(ls_iterator));
    ++ls_iterator;
  }
  if (from_arc_length < curr_arc_length) {
    const auto lerp_ratio =
      (from_arc_length - prev_arc_length) / (curr_arc_length - prev_arc_length);
    const auto from_point = lerp_point(*std::prev(ls_iterator), *ls_iterator, lerp_ratio);
    sub_ls.push_back(from_point);
  }
  while (curr_arc_length < to_arc_length && std::next(ls_iterator) != ls.end()) {
    sub_ls.push_back(*ls_iterator);
    prev_arc_length = curr_arc_length;
    curr_arc_length += boost::geometry::distance(*ls_iterator, *std::next(ls_iterator));
    ++ls_iterator;
  }
  const auto lerp_ratio = (to_arc_length - prev_arc_length) / (curr_arc_length - prev_arc_length);
  const auto to_point = lerp_point(*std::prev(ls_iterator), *ls_iterator, lerp_ratio);
  sub_ls.push_back(to_point);
  return sub_ls;
}
}  // namespace drivable_area_expansion

#endif  // BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__PATH_PROJECTION_HPP_
