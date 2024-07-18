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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__GJK_2D_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__GJK_2D_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

namespace autoware::universe_utils::gjk
{
/**
 * @brief Check if 2 convex polygons intersect using the GJK algorithm
 * @details much faster than boost::geometry::overlaps() but limited to convex polygons
 */
bool intersects(const Polygon2d & convex_polygon1, const Polygon2d & convex_polygon2);
}  // namespace autoware::universe_utils::gjk

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__GJK_2D_HPP_
