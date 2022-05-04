// Copyright 2017-2019 the Autoware Foundation
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
/// \brief This file implements the rotating calipers algorithm for minimum oriented bounding boxes

#ifndef GEOMETRY__BOUNDING_BOX__ROTATING_CALIPERS_HPP_
#define GEOMETRY__BOUNDING_BOX__ROTATING_CALIPERS_HPP_
#include <geometry/bounding_box/bounding_box_common.hpp>
#include <geometry/common_2d.hpp>
#include <geometry/convex_hull.hpp>

#include <algorithm>
#include <cstring>
#include <limits>
#include <list>

namespace autoware
{
namespace common
{
namespace geometry
{
namespace bounding_box
{
namespace details
{
/// \brief Find which (next) edge has smallest angle delta to directions, rotate directions
/// \param[inout] edges Array of edges on polygon after each anchor point (in ccw order).
///                     E.g. if anchor point = p_i, edge = P[i+1] - P[i]
/// \param[inout] directions Array of directions of current bounding box (in ccw order)
/// \tparam PointT Point type of the lists, must have float members x and y
/// \return index of array to advance, e.g. the one with the smallest angle between edge and dir
template <typename PointT>
uint32_t update_angles(const Point4<PointT> & edges, Point4<PointT> & directions)
{
  // find smallest angle to next
  float32_t best_cos_th = -std::numeric_limits<float32_t>::max();
  float32_t best_edge_dir_mag = 0.0F;
  uint32_t advance_idx = 0U;
  for (uint32_t idx = 0U; idx < edges.size(); ++idx) {
    const float32_t edge_dir_mag = std::max(
      norm_2d(edges[idx]) * norm_2d(directions[idx]), std::numeric_limits<float32_t>::epsilon());
    const float32_t cos_th = dot_2d(edges[idx], directions[idx]) / edge_dir_mag;
    if (cos_th > best_cos_th) {
      best_cos_th = cos_th;
      best_edge_dir_mag = edge_dir_mag;
      advance_idx = idx;
    }
  }

  // update directions by smallest angle
  const float32_t sin_th =
    cross_2d(directions[advance_idx], edges[advance_idx]) / best_edge_dir_mag;
  for (uint32_t idx = 0U; idx < edges.size(); ++idx) {
    rotate_2d(directions[idx], best_cos_th, sin_th);
  }

  return advance_idx;
}

/// \brief Given support points i, find direction of edge: e = P[i+1] - P[i]
/// \tparam PointT Point type of the lists, must have float members x and y
/// \tparam IT The iterator type, should dereference to a point type with float members x and y
/// \param[in] begin An iterator pointing to the first point in a point list
/// \param[in] end An iterator pointing to one past the last point in the point list
/// \param[in] support Array of points that are most extreme in 4 directions for current OBB
/// \param[out] edges An array to be filled with the polygon edges next from support points
template <typename IT, typename PointT>
void init_edges(const IT begin, const IT end, const Point4<IT> & support, Point4<PointT> & edges)
{
  for (uint32_t idx = 0U; idx < support.size(); ++idx) {
    auto tmp_it = support[idx];
    ++tmp_it;
    const PointT & q = (tmp_it == end) ? *begin : *tmp_it;
    edges[idx] = minus_2d(q, *support[idx]);
  }
}

/// \brief Scan through list to find support points for bounding box oriented in direction of
///        u = P[1] - P[0]
/// \tparam IT The iterator type, should dereference to a point type with float members x and y
/// \param[in] begin An iterator pointing to the first point in a point list
/// \param[in] end An iterator pointing to one past the last point in the point list
/// \param[out] support Array that gets filled with pointers to points that are most extreme in
///                     each direction aligned with and orthogonal to the first polygon edge.
///                     Most extreme = max/min wrt u = P[1]-P[0] (in the dot/cross product sense)
template <typename IT>
void init_bbox(const IT begin, const IT end, Point4<IT> & support)
{
  // compute initial orientation using first two points
  auto pt_it = begin;
  ++pt_it;
  const auto u = minus_2d(*pt_it, *begin);
  support[0U] = begin;
  std::array<float32_t, 3U> metric{
    {-std::numeric_limits<float32_t>::max(), -std::numeric_limits<float32_t>::max(),
     std::numeric_limits<float32_t>::max()}};
  // track u * p, fabsf(u x p), and -u * p
  for (pt_it = begin; pt_it != end; ++pt_it) {
    // check points against orientation for run_ptr
    const auto & pt = *pt_it;
    // u * p
    const float32_t up = dot_2d(u, pt);
    if (up > metric[0U]) {
      metric[0U] = up;
      support[1U] = pt_it;
    }
    // -u * p
    if (up < metric[2U]) {
      metric[2U] = up;
      support[3U] = pt_it;
    }
    // u x p
    const float32_t uxp = cross_2d(u, pt);
    if (uxp > metric[1U]) {
      metric[1U] = uxp;
      support[2U] = pt_it;
    }
  }
}
/// \brief Compute the minimum bounding box for a convex hull using the rotating calipers method.
/// This function may possibly also be used for computing the width of a convex hull, as it uses the
/// external supports of a single convex hull.
/// \param[in] begin An iterator to the first point on a convex hull
/// \param[in] end An iterator to one past the last point on a convex hull
/// \param[in] metric_fn A functor determining what measure the bounding box is minimum with respect
///                      to
/// \tparam IT An iterator type dereferencable into a point type with float members x and y
/// \tparam MetricF A functor that computes a float measure from the x and y size (width and length)
///                 of a bounding box, assumed to be packed in a Point32 message.
/// \throw std::domain_error if the list of points is too small to reasonable generate a bounding
///                          box
template <typename IT, typename MetricF>
BoundingBox rotating_calipers_impl(const IT begin, const IT end, const MetricF metric_fn)
{
  using PointT = base_type<decltype(*begin)>;
  // sanity check TODO(c.ho) more checks (up to n = 2?)
  if (begin == end) {
    throw std::domain_error("Malformed list, size = " + std::to_string(std::distance(begin, end)));
  }
  // initial scan to get anchor points
  Point4<IT> support;
  init_bbox(begin, end, support);
  // initialize directions accordingly
  Point4<PointT> directions;
  {  // I just don't want the temporary variable floating around
    auto tmp = support[0U];
    ++tmp;
    directions[0U] = minus_2d(*tmp, *support[0U]);
    directions[1U] = get_normal(directions[0U]);
    directions[2U] = minus_2d(directions[0U]);
    directions[3U] = minus_2d(directions[1U]);
  }
  // initialize edges
  Point4<PointT> edges;
  init_edges(begin, end, support, edges);
  // size of initial guess
  BoundingBox bbox;
  decltype(BoundingBox::corners) best_corners;
  compute_corners(best_corners, support, directions);
  size_2d(best_corners, bbox.size);
  bbox.value = metric_fn(bbox.size);
  // rotating calipers step: incrementally advance, update angles, points, compute area
  for (auto it = begin; it != end; ++it) {
    // find smallest angle to next, update directions
    const uint32_t advance_idx = update_angles(edges, directions);
    // recompute area
    decltype(BoundingBox::corners) corners;
    compute_corners(corners, support, directions);
    decltype(BoundingBox::size) tmp_size;
    size_2d(corners, tmp_size);
    const float32_t tmp_value = metric_fn(tmp_size);
    // update best if necessary
    if (tmp_value < bbox.value) {
      // corners: memcpy is fine since I know corners and best_corners are distinct
      (void)std::memcpy(&best_corners[0U], &corners[0U], sizeof(corners));
      // size
      bbox.size = tmp_size;
      bbox.value = tmp_value;
    }
    // Step to next iteration of calipers
    {
      // update smallest support
      auto next_it = support[advance_idx];
      ++next_it;
      const auto run_it = (end == next_it) ? begin : next_it;
      support[advance_idx] = run_it;
      // update edges
      next_it = run_it;
      ++next_it;
      const PointT & next = (end == next_it) ? (*begin) : (*next_it);
      edges[advance_idx] = minus_2d(next, *run_it);
    }
  }

  finalize_box(best_corners, bbox);

  // TODO(christopher.ho) check if one of the sizes is too small, fuzz corner 1 and 2
  // This doesn't cause any issues now, it shouldn't happen in practice, and even if it did,
  // it would probably get smoothed out by prediction. But, this could cause issues with the
  // collision detection algorithms (i.e GJK), but probably not.

  return bbox;
}
}  // namespace details

/// \brief Compute the minimum area bounding box given a convex hull of points.
/// This function is exposed in case a user might already have a convex hull via other methods.
/// \param[in] begin An iterator to the first point on a convex hull
/// \param[in] end An iterator to one past the last point on a convex hull
/// \return A minimum area bounding box, value field is the area
/// \tparam IT An iterator type dereferencable into a point type with float members x and y
template <typename IT>
BoundingBox minimum_area_bounding_box(const IT begin, const IT end)
{
  const auto metric_fn = [](const decltype(BoundingBox::size) & pt) -> float32_t {
    return pt.x * pt.y;
  };
  return details::rotating_calipers_impl(begin, end, metric_fn);
}

/// \brief Compute the minimum perimeter bounding box given a convex hull of points
/// This function is exposed in case a user might already have a convex hull via other methods.
/// \param[in] begin An iterator to the first point on a convex hull
/// \param[in] end An iterator to one past the last point on a convex hull
/// \return A minimum perimeter bounding box, value field is half the perimeter
/// \tparam IT An iterator type dereferencable into a point type with float members x and y
template <typename IT>
BoundingBox minimum_perimeter_bounding_box(const IT begin, const IT end)
{
  const auto metric_fn = [](const decltype(BoundingBox::size) & pt) -> float32_t {
    return pt.x + pt.y;
  };
  return details::rotating_calipers_impl(begin, end, metric_fn);
}

/// \brief Compute the minimum area bounding box given an unstructured list of points.
/// Only a list is supported as it enables the convex hull to be formed in O(n log n) time and
/// without memory allocation.
/// \param[inout] list A list of points to form a hull around, gets reordered
/// \return A minimum area bounding box, value field is the area
/// \tparam PointT Point type of the lists, must have float members x and y
template <typename PointT>
BoundingBox minimum_area_bounding_box(std::list<PointT> & list)
{
  const auto last = convex_hull(list);
  return minimum_area_bounding_box(list.cbegin(), last);
}

/// \brief Compute the minimum perimeter bounding box given an unstructured list of points
/// Only a list is supported as it enables the convex hull to be formed in O(n log n) time and
/// without memory allocation.
/// \param[inout] list A list of points to form a hull around, gets reordered
/// \return A minimum perimeter bounding box, value field is half the perimeter
/// \tparam PointT Point type of the lists, must have float members x and y
template <typename PointT>
BoundingBox minimum_perimeter_bounding_box(std::list<PointT> & list)
{
  const auto last = convex_hull(list);
  return minimum_perimeter_bounding_box(list.cbegin(), last);
}
}  // namespace bounding_box
}  // namespace geometry
}  // namespace common
}  // namespace autoware
#endif  // GEOMETRY__BOUNDING_BOX__ROTATING_CALIPERS_HPP_
