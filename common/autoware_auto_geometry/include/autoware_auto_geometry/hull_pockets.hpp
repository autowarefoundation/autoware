// Copyright 2020 Embotech AG, Zurich, Switzerland. All rights reserved.
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
/// \brief This file implements an algorithm for getting a list of "pockets" in the convex
///        hull of a non-convex simple polygon.

#ifndef AUTOWARE_AUTO_GEOMETRY__HULL_POCKETS_HPP_
#define AUTOWARE_AUTO_GEOMETRY__HULL_POCKETS_HPP_

#include "autoware_auto_geometry/common_2d.hpp"

#include <common/types.hpp>

#include <algorithm>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

using autoware::common::types::float32_t;

namespace autoware
{
namespace common
{
namespace geometry
{

/// \brief Compute a list of "pockets" of a simple polygon
///   (https://en.wikipedia.org/wiki/Simple_polygon), that is, the areas that make
///   up the difference between the polygon and its convex hull.
///   This currently has a bug:
//    * "Rollover" is not properly handled - if a pocket contains the segment from
//      the last point in the list to the first point (which is still part of the
//      polygon), that does not get added
///
/// \param[in] polygon_start Start iterator for the simple polygon (has to be on convex hull)
/// \param[in] polygon_end End iterator for the simple polygon
/// \param[in] convex_hull_start Start iterator for the convex hull of the simple polygon
/// \param[in] convex_hull_end End iterator for the convex hull of the simple polygon
/// \return A vector of vectors of the iterator value type. Each inner vector contains the points
///         for one pocket. We return by value instead of as iterator pairs, because it is possible
///         that the edge connecting the final point in the list and the first point in the list is
///         part of a pocket as well, and this is awkward to represent using iterators into the
///         original collection.
///
/// \tparam Iter1 Iterator to a point type
/// \tparam Iter2 Iterator to a point type (can be the same as Iter1, but does not need to be)
template <typename Iter1, typename Iter2>
typename std::vector<std::vector<typename std::iterator_traits<Iter1>::value_type>> hull_pockets(
  const Iter1 polygon_start, const Iter1 polygon_end, const Iter2 convex_hull_start,
  const Iter2 convex_hull_end)
{
  auto pockets = std::vector<std::vector<typename std::iterator_traits<Iter1>::value_type>>{};
  if (std::distance(polygon_start, polygon_end) <= 3) {
    return pockets;
  }

  // Function to check whether a point is in the convex hull.
  const auto in_convex_hull = [convex_hull_start, convex_hull_end](Iter1 p) {
    return std::any_of(convex_hull_start, convex_hull_end, [p](auto hull_entry) {
      return norm_2d(minus_2d(hull_entry, *p)) < std::numeric_limits<float32_t>::epsilon();
    });
  };

  // We go through the points of the polygon only once, adding pockets to the list of pockets
  // as we detect them.
  std::vector<typename std::iterator_traits<Iter1>::value_type> current_pocket{};
  for (auto it = polygon_start; it != polygon_end; it = std::next(it)) {
    if (in_convex_hull(it)) {
      if (current_pocket.size() > 1) {
        current_pocket.emplace_back(*it);
        pockets.push_back(current_pocket);
      }
      current_pocket.clear();
      current_pocket.emplace_back(*it);
    } else {
      current_pocket.emplace_back(*it);
    }
  }

  // At this point, we have reached the end of the polygon, but have not considered the connection
  // of the final point back to the first. In case the first point is part of a pocket as well, we
  // have some points left in current_pocket, and we add one final pocket that is made up by those
  // points plus the first point in the collection.
  if (current_pocket.size() > 1) {
    current_pocket.push_back(*polygon_start);
    pockets.push_back(current_pocket);
  }

  return pockets;
}
}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // AUTOWARE_AUTO_GEOMETRY__HULL_POCKETS_HPP_
