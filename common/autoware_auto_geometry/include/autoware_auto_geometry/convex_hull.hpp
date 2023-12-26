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
/// \brief This file implements the monotone chain algorithm to compute 2D convex hulls on linked
///        lists of points

#ifndef AUTOWARE_AUTO_GEOMETRY__CONVEX_HULL_HPP_
#define AUTOWARE_AUTO_GEOMETRY__CONVEX_HULL_HPP_

#include "autoware_auto_geometry/common_2d.hpp"

#include <common/types.hpp>

// lint -e537 NOLINT pclint vs cpplint
#include <algorithm>
// lint -e537 NOLINT pclint vs cpplint
#include <limits>
#include <list>
#include <utility>

using autoware::common::types::float32_t;

namespace autoware
{
namespace common
{
namespace geometry
{
/// \brief Contains computation geometry functions not intended for the end user to directly use
namespace details
{

/// \brief Moves points comprising the lower convex hull from points to hull.
/// \param[inout] points A list of points, assumed to be sorted in lexical order
/// \param[inout] hull An empty list of points, assumed to have same allocator as points
///                    (for splice)
/// \tparam PointT The point type for the points list
/// \tparam HullT the point type for the hull list
template <typename PointT, typename HullT>
void form_lower_hull(std::list<PointT> & points, std::list<HullT> & hull)
{
  auto hull_it = hull.cbegin();
  auto point_it = points.cbegin();
  // This ensures that the points we splice to back to the end of list are not processed
  const auto iters = points.size();
  for (auto idx = decltype(iters){0}; idx < iters; ++idx) {
    // splice points from tail of hull to tail of list until point from head of list satisfies ccw
    bool8_t is_ccw = true;
    while ((hull.cbegin() != hull_it) && is_ccw) {
      const auto current_hull_it = hull_it;
      --hull_it;
      is_ccw = ccw(*hull_it, *current_hull_it, *point_it);
      if (!is_ccw) {
        hull_it = current_hull_it;
        break;
      }
      // return this node to list for consideration in upper hull
      points.splice(points.end(), hull, current_hull_it);
    }
    const auto last_point_it = point_it;
    ++point_it;
    // Splice head of list to tail of (lower) hull
    hull.splice(hull.end(), points, last_point_it);
    // point_it has been advanced, hull_it has been advanced (to where point_it was previously)
    hull_it = last_point_it;
  }
  // loop is guaranteed not to underflow because a node can be removed from points AT MOST once
  // per loop iteration. The loop is upper bounded to the prior size of the point list
}
/// \brief Moves points comprising the lower convex hull from points to hull.
/// \param[inout] points A list of points, assumed to be sorted in lexical order, and to contain
///                      the leftmost point
/// \param[inout] hull A list of points, assumed to have same allocator as points (for splice),
///                    and to contain the lower hull (minus the left-most point)
/// \tparam PointT The point type for the points list
/// \tparam HullT the point type for the hull list
template <typename PointT, typename HullT>
void form_upper_hull(std::list<PointT> & points, std::list<HullT> & hull)
{
  // TODO(c.ho) consider reverse iterators, not sure if they work with splice()
  auto hull_it = hull.cend();
  --hull_it;
  const auto lower_hull_end = hull_it;
  auto point_it = points.cend();
  --point_it;
  // This ensures that the points we splice to back to the head of list are not processed
  const auto iters = points.size();
  for (auto idx = decltype(iters){0}; idx < iters; ++idx) {
    // splice points from tail of hull to head of list until ccw is satisfied with tail of list
    bool8_t is_ccw = true;
    while ((lower_hull_end != hull_it) && is_ccw) {
      const auto current_hull_it = hull_it;
      --hull_it;
      is_ccw = ccw(*hull_it, *current_hull_it, *point_it);
      if (!is_ccw) {
        hull_it = current_hull_it;
        break;
      }
      points.splice(points.begin(), hull, current_hull_it);
    }
    const auto last_point_it = point_it;
    --point_it;
    // Splice points from tail of lexically ordered point list to tail of hull
    hull.splice(hull.end(), points, last_point_it);
    hull_it = last_point_it;
  }
  // loop is guaranteed not to underflow because a node can be removed from points AT MOST once
  // per loop iteration. The loop is upper bounded to the prior size of the point list
}

/// \brief A static memory implementation of convex hull computation. Shuffles points around the
///        deque such that the points of the convex hull of the deque of points are first in the
///        deque, with the internal points following in an unspecified order.
///        The head of the deque will be the point with the smallest x value, with the other
///        points following in a counter-clockwise manner (from a top down view/facing -z direction)
/// \param[inout] list A list of nodes that will be pruned down and reordered into a ccw convex hull
/// \return An iterator pointing to one after the last point contained in the hull
/// \tparam PointT Type of a point, must have x and y float members
template <typename PointT>
typename std::list<PointT>::const_iterator convex_hull_impl(std::list<PointT> & list)
{
  // Functor that return whether a <= b in the lexical sense (a.x < b.x), sort by y if tied
  const auto lexical_comparator = [](const PointT & a, const PointT & b) -> bool8_t {
    using point_adapter::x_;
    using point_adapter::y_;
    // cspell: ignore FEPS
    // FEPS means "Float EPSilon"
    constexpr auto FEPS = std::numeric_limits<float32_t>::epsilon();
    return (fabsf(x_(a) - x_(b)) > FEPS) ? (x_(a) < x_(b)) : (y_(a) < y_(b));
  };
  list.sort(lexical_comparator);

  // Temporary list to store points
  std::list<PointT> tmp_hull_list{list.get_allocator()};

  // Shuffle lower hull points over to tmp_hull_list
  form_lower_hull(list, tmp_hull_list);

  // Resort list since we can't guarantee the order TODO(c.ho) is this true?
  list.sort(lexical_comparator);
  // splice first hull point to beginning of list to ensure upper hull is properly closed
  // This is done after sorting because we know exactly where it should go, and keeping it out of
  // sorting reduces complexity somewhat
  list.splice(list.begin(), tmp_hull_list, tmp_hull_list.begin());

  // build upper hull
  form_upper_hull(list, tmp_hull_list);
  // Move hull to beginning of list, return iterator pointing to one after the convex hull
  const auto ret = list.begin();
  // first move left-most point to ensure it is first
  auto tmp_it = tmp_hull_list.end();
  --tmp_it;
  list.splice(list.begin(), tmp_hull_list, tmp_it);
  // Then move remainder of hull points
  list.splice(ret, tmp_hull_list);
  return ret;
}
}  // namespace details

/// \brief A static memory implementation of convex hull computation. Shuffles points around the
///        deque such that the points of the convex hull of the deque of points are first in the
///        deque, with the internal points following in an unspecified order.
///
///        The head of the deque will be the point with the smallest x value, with the other
///        points following in a counter-clockwise manner (from a top down view/facing -z
///        direction). If the point list is 3 or smaller, nothing is done (e.g. the ordering result
///        as previously stated does not hold).
/// \param[inout] list A list of nodes that will be reordered into a ccw convex hull
/// \return An iterator pointing to one after the last point contained in the hull
/// \tparam PointT Type of a point, must have x and y float members
template <typename PointT>
typename std::list<PointT>::const_iterator convex_hull(std::list<PointT> & list)
{
  return (list.size() <= 3U) ? list.end() : details::convex_hull_impl(list);
}

}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // AUTOWARE_AUTO_GEOMETRY__CONVEX_HULL_HPP_
