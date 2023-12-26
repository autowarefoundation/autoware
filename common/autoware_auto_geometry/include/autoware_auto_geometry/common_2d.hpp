// Copyright 2017-2021 the Autoware Foundation
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
/// \file
/// \brief This file includes common functionality for 2D geometry, such as dot products

#ifndef AUTOWARE_AUTO_GEOMETRY__COMMON_2D_HPP_
#define AUTOWARE_AUTO_GEOMETRY__COMMON_2D_HPP_

#include "autoware_auto_geometry/interval.hpp"

#include <common/types.hpp>

#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace comparison = autoware::common::helper_functions::comparisons;

namespace autoware
{
namespace common
{
namespace geometry
{

/// \brief Temporary namespace for point adapter methods, for use with nonstandard point types
namespace point_adapter
{
/// \brief Gets the x value for a point
/// \return The x value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template <typename PointT>
inline auto x_(const PointT & pt)
{
  return pt.x;
}
/// \brief Gets the x value for a TrajectoryPoint message
/// \return The x value of the point
/// \param[in] pt The point
inline auto x_(const autoware_auto_planning_msgs::msg::TrajectoryPoint & pt)
{
  return pt.pose.position.x;
}
/// \brief Gets the y value for a point
/// \return The y value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template <typename PointT>
inline auto y_(const PointT & pt)
{
  return pt.y;
}
/// \brief Gets the y value for a TrajectoryPoint message
/// \return The y value of the point
/// \param[in] pt The point
inline auto y_(const autoware_auto_planning_msgs::msg::TrajectoryPoint & pt)
{
  return pt.pose.position.y;
}
/// \brief Gets the z value for a point
/// \return The z value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template <typename PointT>
inline auto z_(const PointT & pt)
{
  return pt.z;
}
/// \brief Gets the z value for a TrajectoryPoint message
/// \return The z value of the point
/// \param[in] pt The point
inline auto z_(const autoware_auto_planning_msgs::msg::TrajectoryPoint & pt)
{
  return pt.pose.position.z;
}
/// \brief Gets a reference to the x value for a point
/// \return A reference to the x value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template <typename PointT>
inline auto & xr_(PointT & pt)
{
  return pt.x;
}
/// \brief Gets a reference to the x value for a TrajectoryPoint
/// \return A reference to the x value of the TrajectoryPoint
/// \param[in] pt The TrajectoryPoint
inline auto & xr_(autoware_auto_planning_msgs::msg::TrajectoryPoint & pt)
{
  return pt.pose.position.x;
}
/// \brief Gets a reference to the y value for a point
/// \return A reference to The y value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template <typename PointT>
inline auto & yr_(PointT & pt)
{
  return pt.y;
}
/// \brief Gets a reference to the y value for a TrajectoryPoint
/// \return A reference to the y value of the TrajectoryPoint
/// \param[in] pt The TrajectoryPoint
inline auto & yr_(autoware_auto_planning_msgs::msg::TrajectoryPoint & pt)
{
  return pt.pose.position.y;
}
/// \brief Gets a reference to the z value for a point
/// \return A reference to the z value of the point
/// \param[in] pt The point
/// \tparam PointT The point type
template <typename PointT>
inline auto & zr_(PointT & pt)
{
  return pt.z;
}
/// \brief Gets a reference to the z value for a TrajectoryPoint
/// \return A reference to the z value of the TrajectoryPoint
/// \param[in] pt The TrajectoryPoint
inline auto & zr_(autoware_auto_planning_msgs::msg::TrajectoryPoint & pt)
{
  return pt.pose.position.z;
}
}  // namespace point_adapter

namespace details
{
// Return the next iterator, cycling back to the beginning of the list if you hit the end
template <typename IT>
IT circular_next(const IT begin, const IT end, const IT current) noexcept
{
  auto next = std::next(current);
  if (end == next) {
    next = begin;
  }
  return next;
}

}  // namespace details

/// \tparam T1, T2, T3 point type. Must have point adapters defined or have float members x and y
/// \brief compute whether line segment rp is counter clockwise relative to line segment qp
/// \param[in] pt shared point for both line segments
/// \param[in] r point to check if it forms a ccw angle
/// \param[in] q reference point
/// \return whether angle formed is ccw. Three collinear points is considered ccw
template <typename T1, typename T2, typename T3>
inline auto ccw(const T1 & pt, const T2 & q, const T3 & r)
{
  using point_adapter::x_;
  using point_adapter::y_;
  return (((x_(q) - x_(pt)) * (y_(r) - y_(pt))) - ((y_(q) - y_(pt)) * (x_(r) - x_(pt)))) <= 0.0F;
}

/// \tparam T1, T2 point type. Must have point adapters defined or have float members x and y
/// \brief compute p x q = p1 * q2 - p2 * q1
/// \param[in] pt first point
/// \param[in] q second point
/// \return 2d cross product
template <typename T1, typename T2>
inline auto cross_2d(const T1 & pt, const T2 & q)
{
  using point_adapter::x_;
  using point_adapter::y_;
  return (x_(pt) * y_(q)) - (y_(pt) * x_(q));
}

/// \tparam T1, T2 point type. Must have point adapters defined or have float members x and y
/// \brief compute p * q = p1 * q1 + p2 * q2
/// \param[in] pt first point
/// \param[in] q second point
/// \return 2d scalar dot product
template <typename T1, typename T2>
inline auto dot_2d(const T1 & pt, const T2 & q)
{
  using point_adapter::x_;
  using point_adapter::y_;
  return (x_(pt) * x_(q)) + (y_(pt) * y_(q));
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief Compute the 2d difference between two points, p - q
/// \param[in] p The left hand side
/// \param[in] q The right hand side
/// \return A point with the difference in the x and y fields, all other fields are default
///         initialized
template <typename T>
T minus_2d(const T & p, const T & q)
{
  T r;
  using point_adapter::x_;
  using point_adapter::y_;
  point_adapter::xr_(r) = x_(p) - x_(q);
  point_adapter::yr_(r) = y_(p) - y_(q);
  return r;
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief The unary minus or negation operator applied to a single point's 2d fields
/// \param[in] p The left hand side
/// \return A point with the negation in the x and y fields, all other fields are default
///         initialized
template <typename T>
T minus_2d(const T & p)
{
  T r;
  point_adapter::xr_(r) = -point_adapter::x_(p);
  point_adapter::yr_(r) = -point_adapter::y_(p);
  return r;
}
/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief The 2d addition operation, p + q
/// \param[in] p The left hand side
/// \param[in] q The right hand side
/// \return A point with the sum in the x and y fields, all other fields are default
///         initialized
template <typename T>
T plus_2d(const T & p, const T & q)
{
  T r;
  using point_adapter::x_;
  using point_adapter::y_;
  point_adapter::xr_(r) = x_(p) + x_(q);
  point_adapter::yr_(r) = y_(p) + y_(q);
  return r;
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief The scalar multiplication operation, p * a
/// \param[in] p The point value
/// \param[in] a The scalar value
/// \return A point with the scaled x and y fields, all other fields are default
///         initialized
template <typename T>
T times_2d(const T & p, const float32_t a)
{
  T r;
  point_adapter::xr_(r) = static_cast<float32_t>(point_adapter::x_(p)) * a;
  point_adapter::yr_(r) = static_cast<float32_t>(point_adapter::y_(p)) * a;
  return r;
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief solve p + t * u = q + s * v
///        Ref: https://stackoverflow.com/questions/563198/
/// \param[in] pt anchor point of first line
/// \param[in] u direction of first line
/// \param[in] q anchor point of second line
/// \param[in] v direction of second line
/// \return intersection point
/// \throw std::runtime_error if lines are (nearly) collinear or parallel
template <typename T>
inline T intersection_2d(const T & pt, const T & u, const T & q, const T & v)
{
  const float32_t num = cross_2d(minus_2d(pt, q), u);
  float32_t den = cross_2d(v, u);
  // cspell: ignore FEPS
  // FEPS means "Float EPSilon"
  constexpr auto FEPS = std::numeric_limits<float32_t>::epsilon();
  if (fabsf(den) < FEPS) {
    if (fabsf(num) < FEPS) {
      // collinear case, anything is ok
      den = 1.0F;
    } else {
      // parallel case, no valid output
      throw std::runtime_error(
        "intersection_2d: no unique solution (either collinear or parallel)");
    }
  }
  return plus_2d(q, times_2d(v, num / den));
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief rotate point given precomputed sin and cos
/// \param[inout] pt point to rotate
/// \param[in] cos_th precomputed cosine value
/// \param[in] sin_th precomputed sine value
template <typename T>
inline void rotate_2d(T & pt, const float32_t cos_th, const float32_t sin_th)
{
  const float32_t x = point_adapter::x_(pt);
  const float32_t y = point_adapter::y_(pt);
  point_adapter::xr_(pt) = (cos_th * x) - (sin_th * y);
  point_adapter::yr_(pt) = (sin_th * x) + (cos_th * y);
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief rotate by radian angle th in z direction with ccw positive
/// \param[in] pt reference point to rotate
/// \param[in] th_rad angle by which to rotate point
/// \return rotated point
template <typename T>
inline T rotate_2d(const T & pt, const float32_t th_rad)
{
  T q(pt);  // It's reasonable to expect a copy constructor
  const float32_t s = sinf(th_rad);
  const float32_t c = cosf(th_rad);
  rotate_2d(q, c, s);
  return q;
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief compute q s.t. p T q, or p * q = 0
///        This is the equivalent of a 90 degree ccw rotation
/// \param[in] pt point to get normal point of
/// \return point normal to p (un-normalized)
template <typename T>
inline T get_normal(const T & pt)
{
  T q(pt);
  point_adapter::xr_(q) = -point_adapter::y_(pt);
  point_adapter::yr_(q) = point_adapter::x_(pt);
  return q;
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief get magnitude of x and y components:
/// \param[in] pt point to get magnitude of
/// \return magnitude of x and y components together
template <typename T>
inline auto norm_2d(const T & pt)
{
  return sqrtf(static_cast<float32_t>(dot_2d(pt, pt)));
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief Compute the closest point on line segment p-q to point r
///        Based on equations from https://stackoverflow.com/a/1501725 and
///        http://paulbourke.net/geometry/pointlineplane/
/// \param[in] p First point defining the line segment
/// \param[in] q Second point defining the line segment
/// \param[in] r Reference point to find the closest point to
/// \return Closest point on line segment p-q to point r
template <typename T>
inline T closest_segment_point_2d(const T & p, const T & q, const T & r)
{
  const T qp = minus_2d(q, p);
  const float32_t len2 = static_cast<float32_t>(dot_2d(qp, qp));
  T ret = p;
  if (len2 > std::numeric_limits<float32_t>::epsilon()) {
    const Interval_f unit_interval(0.0f, 1.0f);
    const float32_t val = static_cast<float32_t>(dot_2d(minus_2d(r, p), qp)) / len2;
    const float32_t t = Interval_f::clamp_to(unit_interval, val);
    ret = plus_2d(p, times_2d(qp, t));
  }
  return ret;
}
//
/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief Compute the closest point on the line going through p-q to point r
//         Obtained by simplifying closest_segment_point_2d.
/// \param[in] p First point defining the line
/// \param[in] q Second point defining the line
/// \param[in] r Reference point to find the closest point to
/// \return Closest point on line p-q to point r
/// \throw std::runtime_error if the two points coincide and hence don't uniquely
//         define a line
template <typename T>
inline T closest_line_point_2d(const T & p, const T & q, const T & r)
{
  const T qp = minus_2d(q, p);
  const float32_t len2 = dot_2d(qp, qp);
  T ret = p;
  if (len2 > std::numeric_limits<float32_t>::epsilon()) {
    const float32_t t = dot_2d(minus_2d(r, p), qp) / len2;
    ret = plus_2d(p, times_2d(qp, t));
  } else {
    throw std::runtime_error(
      "closet_line_point_2d: line ill-defined because given points coincide");
  }
  return ret;
}

/// \tparam T point type. Must have point adapters defined or have float members x and y
/// \brief Compute the distance from line segment p-q to point r
/// \param[in] p First point defining the line segment
/// \param[in] q Second point defining the line segment
/// \param[in] r Reference point to find the distance from the line segment to
/// \return Distance from point r to line segment p-q
template <typename T>
inline auto point_line_segment_distance_2d(const T & p, const T & q, const T & r)
{
  const T pq_r = minus_2d(closest_segment_point_2d(p, q, r), r);
  return norm_2d(pq_r);
}

/// \brief Make a 2D unit vector given an angle.
/// \tparam T Point type. Must have point adapters defined or have float members x and y
/// \param th Angle in radians
/// \return Unit vector in the direction of the given angle.
template <typename T>
inline T make_unit_vector2d(float th)
{
  T r;
  point_adapter::xr_(r) = std::cos(th);
  point_adapter::yr_(r) = std::sin(th);
  return r;
}

/// \brief Compute squared euclidean distance between two points
/// \tparam OUT return type. Type of the returned distance.
/// \tparam T1 point type. Must have point adapters defined or have float members x and y
/// \tparam T2 point type. Must have point adapters defined or have float members x and y
/// \param a point 1
/// \param b point 2
/// \return squared euclidean distance
template <typename OUT = float32_t, typename T1, typename T2>
inline OUT squared_distance_2d(const T1 & a, const T2 & b)
{
  const auto x = static_cast<OUT>(point_adapter::x_(a)) - static_cast<OUT>(point_adapter::x_(b));
  const auto y = static_cast<OUT>(point_adapter::y_(a)) - static_cast<OUT>(point_adapter::y_(b));
  return (x * x) + (y * y);
}

/// \brief Compute euclidean distance between two points
/// \tparam OUT return type. Type of the returned distance.
/// \tparam T1 point type. Must have point adapters defined or have float members x and y
/// \tparam T2 point type. Must have point adapters defined or have float members x and y
/// \param a point 1
/// \param b point 2
/// \return euclidean distance
template <typename OUT = float32_t, typename T1, typename T2>
inline OUT distance_2d(const T1 & a, const T2 & b)
{
  return std::sqrt(squared_distance_2d<OUT>(a, b));
}

/// \brief Check the given point's position relative the infinite line passing
/// from p1 to p2. Logic based on http://geomalgorithms.com/a01-_area.html#isLeft()
/// \tparam T  T  point type. Must have point adapters defined or have float members x and y
/// \param p1 point 1 laying on the infinite line
/// \param p2 point 2 laying on the infinite line
/// \param q point to be checked against the line
/// \return > 0 for point q left of the line through p1 to p2
///         = 0 for point q on the line through p1 to p2
///         < 0 for point q right of the line through p1 to p2
template <typename T>
inline auto check_point_position_to_line_2d(const T & p1, const T & p2, const T & q)
{
  return cross_2d(minus_2d(p2, p1), minus_2d(q, p1));
}

/// Check if all points are ordered in x-y plane (in either clockwise or counter-clockwise
/// direction): This function does not check for convexity
/// \tparam IT Iterator type pointing to a point containing float x and float y
/// \param[in] begin Beginning of point sequence
/// \param[in] end One past the last of the point sequence
/// \return Whether or not all point triples p_i, p_{i+1}, p_{i+2} are in a particular order.
///         Returns true for collinear points as well
template <typename IT>
bool all_ordered(const IT begin, const IT end) noexcept
{
  // Short circuit: a line or point is always CCW or otherwise ill-defined
  if (std::distance(begin, end) <= 2U) {
    return true;
  }
  bool is_first_point_cw = false;
  // Can't use std::all_of because that works directly on the values
  for (auto line_start = begin; line_start != end; ++line_start) {
    const auto line_end = details::circular_next(begin, end, line_start);
    const auto query_point = details::circular_next(begin, end, line_end);
    // Check if 3 points starting from current point are in clockwise direction
    const bool is_cw = comparison::abs_lte(
      check_point_position_to_line_2d(*line_start, *line_end, *query_point), 0.0F, 1e-3F);
    if (is_cw) {
      if (line_start == begin) {
        is_first_point_cw = true;
      } else {
        if (!is_first_point_cw) {
          return false;
        }
      }
    } else if (is_first_point_cw) {
      return false;
    }
  }
  return true;
}

/// Compute the area of a convex hull, points are assumed to be ordered (in either CW or CCW)
/// \tparam IT Iterator type pointing to a point containing float x and float y
/// \param[in] begin Iterator pointing to the beginning of the polygon points
/// \param[in] end Iterator pointing to one past the last of the polygon points
/// \return The area of the polygon, in squared of whatever units your points are in
template <typename IT>
auto area_2d(const IT begin, const IT end) noexcept
{
  using point_adapter::x_;
  using point_adapter::y_;
  using T = decltype(x_(*begin));
  auto area = T{};  // zero initialization
  // use the approach of https://www.mathwords.com/a/area_convex_polygon.htm
  for (auto it = begin; it != end; ++it) {
    const auto next = details::circular_next(begin, end, it);
    area += x_(*it) * y_(*next);
    area -= x_(*next) * y_(*it);
  }
  return std::abs(T{0.5} * area);
}

/// Compute area of convex hull, throw if points are not ordered (convexity check is not
/// implemented)
/// \throw std::domain_error if points are not ordered either CW or CCW
/// \tparam IT Iterator type pointing to a point containing float x and float y
/// \param[in] begin Iterator pointing to the beginning of the polygon points
/// \param[in] end Iterator pointing to one past the last of the polygon points
/// \return The area of the polygon, in squared of whatever units your points are in
template <typename IT>
auto area_checked_2d(const IT begin, const IT end)
{
  if (!all_ordered(begin, end)) {
    throw std::domain_error{"Cannot compute area: points are not ordered"};
  }
  return area_2d(begin, end);
}

/// \brief Check if the given point is inside or on the edge of the given polygon
/// \tparam IteratorType iterator type. The value pointed to by this must have point adapters
///                         defined or have float members x and y
/// \tparam PointType point type. Must have point adapters defined or have float members x and y
/// \param start_it iterator pointing to the first vertex of the polygon
/// \param end_it iterator pointing to the last vertex of the polygon. The vertices should be in
///               order.
/// \param p point to be searched
/// \return True if the point is inside or on the edge of the polygon. False otherwise
template <typename IteratorType, typename PointType>
bool is_point_inside_polygon_2d(
  const IteratorType & start_it, const IteratorType & end_it, const PointType & p)
{
  std::int32_t winding_num = 0;

  for (IteratorType it = start_it; it != end_it; ++it) {
    auto next_it = std::next(it);
    if (next_it == end_it) {
      next_it = start_it;
    }
    if (point_adapter::y_(*it) <= point_adapter::y_(p)) {
      // Upward crossing edge
      if (point_adapter::y_(*next_it) >= point_adapter::y_(p)) {
        if (comparison::abs_gt(check_point_position_to_line_2d(*it, *next_it, p), 0.0F, 1e-3F)) {
          ++winding_num;
        } else {
          if (comparison::abs_eq_zero(check_point_position_to_line_2d(*it, *next_it, p), 1e-3F)) {
            return true;
          }
        }
      }
    } else {
      // Downward crossing edge
      if (point_adapter::y_(*next_it) <= point_adapter::y_(p)) {
        if (comparison::abs_lt(check_point_position_to_line_2d(*it, *next_it, p), 0.0F, 1e-3F)) {
          --winding_num;
        } else {
          if (comparison::abs_eq_zero(check_point_position_to_line_2d(*it, *next_it, p), 1e-3F)) {
            return true;
          }
        }
      }
    }
  }
  return winding_num != 0;
}

}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // AUTOWARE_AUTO_GEOMETRY__COMMON_2D_HPP_
