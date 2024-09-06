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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__RANDOM_CONCAVE_POLYGON_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__RANDOM_CONCAVE_POLYGON_HPP_

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <vector>
namespace autoware::universe_utils
{
/// @brief generate a random non-convex polygon
/// @param vertices number of vertices for the desired polygon
/// @param max points will be generated in the range [-max, max]
/// @details algorithm from
/// https://digitalscholarship.unlv.edu/cgi/viewcontent.cgi?article=3183&context=thesesdissertations
Polygon2d random_concave_polygon(const size_t vertices, const double max);

/// @brief checks for collisions between two vectors of convex polygons using a specified collision
/// detection algorithm
/// @param polygons1 A vector of convex polygons to check for collisions.
/// @param polygons2 A vector of convex polygons to check for collisions.
/// @param intersection_func A function that takes two polygons and returns true if they intersect,
/// otherwise false.
/// @return True if at least one pair of polygons intersects, otherwise false.
bool test_intersection(
  const std::vector<autoware::universe_utils::Polygon2d> & polygons1,
  const std::vector<autoware::universe_utils::Polygon2d> & polygons2,
  const std::function<bool(
    const autoware::universe_utils::Polygon2d &, const autoware::universe_utils::Polygon2d &)> &);

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__RANDOM_CONCAVE_POLYGON_HPP_
