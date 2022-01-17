// Copyright 2021 Tier IV, Inc.
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

#ifndef SCENE_MODULE__OCCLUSION_SPOT__GEOMETRY_HPP_
#define SCENE_MODULE__OCCLUSION_SPOT__GEOMETRY_HPP_

#include <boost/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>

#include <vector>

namespace behavior_velocity_planner
{
namespace geometry
{
namespace bg = boost::geometry;

// @brief represent the range of a slice
// (either by index on the lanelet centerline or by arc coordinates)
struct SliceRange
{
  double min_length{};
  double max_length{};
  double min_distance{};
  double max_distance{};
};

// @brief representation of a slice along a path
struct Slice
{
  SliceRange range{};
  lanelet::BasicPolygon2d polygon{};
};

//!< @brief build slices all along the trajectory
// using the given range and desired slice length and width
void buildSlices(
  std::vector<Slice> & slices, const lanelet::ConstLanelet & path_lanelet, const SliceRange & range,
  const double slice_length, const double slice_width, const double resolution);
//!< @brief build sidewalk slice from path
void buildSidewalkSlices(
  std::vector<geometry::Slice> & slice, const lanelet::ConstLanelet & path_lanelet,
  const double longitudinal_offset, const double lateral_offset, const double min_size,
  const double lateral_max_dist);
//!< @brief calculate interpolation between a and b at distance ratio t
template <typename T>
T lerp(T a, T b, double t)
{
  return a + t * (b - a);
}

}  // namespace geometry
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OCCLUSION_SPOT__GEOMETRY_HPP_
