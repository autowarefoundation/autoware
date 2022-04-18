// Copyright 2020 Embotech AG, Zurich, Switzerland
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef GEOMETRY__INTERSECTION_HPP_
#define GEOMETRY__INTERSECTION_HPP_

#include <geometry/common_2d.hpp>
#include <geometry/convex_hull.hpp>

#include <autoware_auto_perception_msgs/msg/bounding_box.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>

#include <algorithm>
#include <iostream>
#include <limits>
#include <list>
#include <type_traits>
#include <utility>
#include <vector>

namespace autoware
{
namespace common
{
namespace geometry
{
using autoware::common::geometry::closest_line_point_2d;
using autoware::common::geometry::convex_hull;
using autoware::common::geometry::dot_2d;
using autoware::common::geometry::get_normal;
using autoware::common::geometry::minus_2d;
using autoware::common::geometry::norm_2d;
using autoware::common::geometry::times_2d;
using autoware_auto_perception_msgs::msg::BoundingBox;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

using Point = geometry_msgs::msg::Point32;

namespace details
{

/// Alias for a std::pair of two points
using Line = std::pair<Point, Point>;

/// \tparam Iter1 Iterator over point-types that must have point adapters
//      defined or have float members x and y
/// \brief Compute a sorted list of faces of a polyhedron given a list of points
/// \param[in] start Start iterator of the list of points
/// \param[in] end End iterator of the list of points
/// \return The list of faces
template <typename Iter>
std::vector<Line> get_sorted_face_list(const Iter start, const Iter end)
{
  // First get a sorted list of points - convex_hull does that by modifying its argument
  auto corner_list = std::list<Point>(start, end);
  const auto first_interior_point = convex_hull(corner_list);

  std::vector<Line> face_list{};
  auto itLast = corner_list.begin();
  auto itNext = std::next(itLast, 1);
  do {
    face_list.emplace_back(Line{*itLast, *itNext});
    itLast = itNext;
    itNext = std::next(itNext, 1);
  } while ((itNext != first_interior_point) && (itNext != corner_list.end()));

  face_list.emplace_back(Line{*itLast, corner_list.front()});

  return face_list;
}

/// \brief Append points of the polygon `internal` that are contained in the polygon `exernal`.
template <
  template <typename...> class Iterable1T, template <typename...> class Iterable2T, typename PointT>
void append_contained_points(
  const Iterable1T<PointT> & external, const Iterable2T<PointT> & internal,
  std::list<PointT> & result)
{
  std::copy_if(
    internal.begin(), internal.end(), std::back_inserter(result), [&external](const auto & pt) {
      return common::geometry::is_point_inside_polygon_2d(external.begin(), external.end(), pt);
    });
}

/// \brief Append the intersecting points between two polygons into the output list.
template <
  template <typename...> class Iterable1T, template <typename...> class Iterable2T, typename PointT>
void append_intersection_points(
  const Iterable1T<PointT> & polygon1, const Iterable2T<PointT> & polygon2,
  std::list<PointT> & result)
{
  using FloatT = decltype(point_adapter::x_(std::declval<PointT>()));
  using Interval = common::geometry::Interval<float32_t>;

  auto get_edge = [](const auto & list, const auto & iterator) {
    const auto next_it = std::next(iterator);
    const auto & next_pt = (next_it != list.end()) ? *next_it : list.front();
    return std::make_pair(*iterator, next_pt);
  };

  // Get the max absolute value out of two intervals and a scalar.
  auto compute_eps_scale = [](const auto & i1, const auto & i2, const auto val) {
    auto get_abs_max = [](const auto & interval) {
      return std::max(std::fabs(Interval::min(interval)), std::fabs(Interval::max(interval)));
    };
    return std::max(std::fabs(val), std::max(get_abs_max(i1), get_abs_max(i2)));
  };

  // Compare each edge from polygon1 to each edge from polygon2
  for (auto corner1_it = polygon1.begin(); corner1_it != polygon1.end(); ++corner1_it) {
    const auto & edge1 = get_edge(polygon1, corner1_it);

    Interval edge1_x_interval{
      std::min(point_adapter::x_(edge1.first), point_adapter::x_(edge1.second)),
      std::max(point_adapter::x_(edge1.first), point_adapter::x_(edge1.second))};

    Interval edge1_y_interval{
      std::min(point_adapter::y_(edge1.first), point_adapter::y_(edge1.second)),
      std::max(point_adapter::y_(edge1.first), point_adapter::y_(edge1.second))};

    for (auto corner2_it = polygon2.begin(); corner2_it != polygon2.end(); ++corner2_it) {
      try {
        const auto & edge2 = get_edge(polygon2, corner2_it);
        const auto & intersection = common::geometry::intersection_2d(
          edge1.first, minus_2d(edge1.second, edge1.first), edge2.first,
          minus_2d(edge2.second, edge2.first));

        Interval edge2_x_interval{
          std::min(point_adapter::x_(edge2.first), point_adapter::x_(edge2.second)),
          std::max(point_adapter::x_(edge2.first), point_adapter::x_(edge2.second))};

        Interval edge2_y_interval{
          std::min(point_adapter::y_(edge2.first), point_adapter::y_(edge2.second)),
          std::max(point_adapter::y_(edge2.first), point_adapter::y_(edge2.second))};

        // The accumulated floating point error depends on the magnitudes of each end of the
        // intervals. Hence the upper bound of the absolute magnitude should be taken into account
        // while computing the epsilon.
        const auto max_feps_scale = std::max(
          compute_eps_scale(edge1_x_interval, edge2_x_interval, point_adapter::x_(intersection)),
          compute_eps_scale(edge1_y_interval, edge2_y_interval, point_adapter::y_(intersection)));
        const auto feps = max_feps_scale * std::numeric_limits<FloatT>::epsilon();
        // Only accept intersections that lie on both of the line segments (edges)
        if (
          Interval::contains(edge1_x_interval, point_adapter::x_(intersection), feps) &&
          Interval::contains(edge2_x_interval, point_adapter::x_(intersection), feps) &&
          Interval::contains(edge1_y_interval, point_adapter::y_(intersection), feps) &&
          Interval::contains(edge2_y_interval, point_adapter::y_(intersection), feps)) {
          result.push_back(intersection);
        }
      } catch (const std::runtime_error &) {
        // Parallel lines. TODO(yunus.caliskan): #1229
        continue;
      }
    }
  }
}

}  // namespace details

// TODO(s.me) implement GJK(+EPA) algorithm as well as per Chris Ho's suggestion
/// \tparam Iter Iterator over point-types that must have point adapters
//      defined or have float members x and y
/// \brief Check if polyhedra defined by two given sets of points intersect
//    This uses SAT for doing the actual checking
//    (https://en.wikipedia.org/wiki/Hyperplane_separation_theorem#Use_in_collision_detection)
/// \param[in] begin1 Start iterator to first list of point types
/// \param[in] end1   End iterator to first list of point types
/// \param[in] begin2 Start iterator to first list of point types
/// \param[in] end2   End iterator to first list of point types
/// \return true if the boxes collide, false otherwise.
template <typename Iter>
bool intersect(const Iter begin1, const Iter end1, const Iter begin2, const Iter end2)
{
  // Obtain sorted lists of faces of both boxes, merge them into one big list of faces
  auto faces = details::get_sorted_face_list(begin1, end1);
  const auto faces_2 = details::get_sorted_face_list(begin2, end2);
  faces.reserve(faces.size() + faces_2.size());
  faces.insert(faces.end(), faces_2.begin(), faces_2.end());

  // Also look at last line
  for (const auto & face : faces) {
    // Compute normal vector to the face and define a closure to get progress along it
    const auto normal = get_normal(minus_2d(face.second, face.first));
    auto get_position_along_line = [&normal](auto point) {
      return dot_2d(normal, minus_2d(point, Point{}));
    };

    // Define a function to get the minimum and maximum projected position of the corners
    // of a given bounding box along the normal line of the face
    auto get_projected_min_max = [&get_position_along_line, &normal](Iter begin, Iter end) {
      const auto zero_point = Point{};
      auto min_corners = get_position_along_line(closest_line_point_2d(normal, zero_point, *begin));
      auto max_corners = min_corners;

      for (auto & point = begin; point != end; ++point) {
        const auto point_projected = closest_line_point_2d(normal, zero_point, *point);
        const auto position_along_line = get_position_along_line(point_projected);
        min_corners = std::min(min_corners, position_along_line);
        max_corners = std::max(max_corners, position_along_line);
      }
      return std::pair<float, float>{min_corners, max_corners};
    };

    // Perform the actual computations for the extent computation
    auto minmax_1 = get_projected_min_max(begin1, end1);
    auto minmax_2 = get_projected_min_max(begin2, end2);

    // Check for any intersections
    const auto eps = std::numeric_limits<decltype(minmax_1.first)>::epsilon();
    if (minmax_1.first > minmax_2.second + eps || minmax_2.first > minmax_1.second + eps) {
      // Found separating hyperplane, stop
      return false;
    }
  }

  // No separating hyperplane found, boxes collide
  return true;
}

/// \brief Get the intersection between two polygons. The polygons should be provided in an
/// identical format to the output of `convex_hull` function as in the corners should be ordered
/// in a CCW fashion.
/// The computation is done by:
/// * Find the corners of each polygon that are contained by the other polygon.
/// * Find the intersection points between two polygons
/// * Combine these points and order CCW to get the final polygon.
/// The criteria for intersection is better explained in:
/// "Area of intersection of arbitrary polygons" (Livermore, Calif, 1977)
/// See https://www.osti.gov/servlets/purl/7309916/, chapter II - B
/// TODO(yunus.caliskan): This is a naive implementation. We should scan for a more efficient
///  algorithm: #1230
/// \tparam Iterable1T A container class that has stl style iterators defined.
/// \tparam Iterable2T A container class that has stl style iterators defined.
/// \tparam PointT Point type that have the adapters for the x and y fields.
/// set to `Point1T`
/// \param polygon1 A convex polygon
/// \param polygon2 A convex polygon
/// \return The resulting conv
template <
  template <typename...> class Iterable1T, template <typename...> class Iterable2T, typename PointT>
std::list<PointT> convex_polygon_intersection2d(
  const Iterable1T<PointT> & polygon1, const Iterable2T<PointT> & polygon2)
{
  std::list<PointT> result;
  details::append_contained_points(polygon1, polygon2, result);
  details::append_contained_points(polygon2, polygon1, result);
  details::append_intersection_points(polygon1, polygon2, result);
  const auto end_it = common::geometry::convex_hull(result);
  result.resize(static_cast<uint32_t>(std::distance(result.cbegin(), end_it)));
  return result;
}

/// \brief Compute the intersection over union of two 2d convex polygons. If any of the polygons
/// span a zero area, the result is 0.0.
/// \tparam Iterable1T A container class that has stl style iterators defined.
/// \tparam Iterable2T A container class that has stl style iterators defined.
/// \tparam Point1T Point type that have the adapters for the x and y fields.
/// \tparam Point2T Point type that have the adapters for the x and y fields.
/// \param polygon1 A convex polygon
/// \param polygon2 A convex polygon
/// \return (Intersection / Union) between two given polygons.
/// \throws std::domain_error If there is any inconsistency on the undderlying geometrical
/// computation.
template <
  template <typename...> class Iterable1T, template <typename...> class Iterable2T, typename PointT>
common::types::float32_t convex_intersection_over_union_2d(
  const Iterable1T<PointT> & polygon1, const Iterable2T<PointT> & polygon2)
{
  constexpr auto eps = std::numeric_limits<float32_t>::epsilon();
  const auto intersection = convex_polygon_intersection2d(polygon1, polygon2);

  const auto intersection_area =
    common::geometry::area_2d(intersection.begin(), intersection.end());

  if (intersection_area < eps) {
    return 0.0F;  // There's either no intersection or the points are collinear
  }

  const auto polygon1_area = common::geometry::area_2d(polygon1.begin(), polygon1.end());
  const auto polygon2_area = common::geometry::area_2d(polygon2.begin(), polygon2.end());

  const auto union_area = polygon1_area + polygon2_area - intersection_area;
  if (union_area < eps) {
    throw std::domain_error("IoU is undefined for polygons with a zero union area");
  }

  return intersection_area / union_area;
}

}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // GEOMETRY__INTERSECTION_HPP_
