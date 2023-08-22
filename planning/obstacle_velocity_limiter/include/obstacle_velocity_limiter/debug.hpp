// Copyright 2022 TIER IV, Inc.
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

#ifndef OBSTACLE_VELOCITY_LIMITER__DEBUG_HPP_
#define OBSTACLE_VELOCITY_LIMITER__DEBUG_HPP_

#include "obstacle_velocity_limiter/obstacles.hpp"
#include "obstacle_velocity_limiter/types.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace obstacle_velocity_limiter
{
/// @brief make the visualization Marker of the given linestring
/// @param[in] ls linestring to turn into a marker
/// @param[in] z z-value to use in the marker
/// @return marker representing the linestring
visualization_msgs::msg::Marker makeLinestringMarker(const linestring_t & ls, const Float z);

/// @brief make debug marker array
/// @param[in] obstacles obstacles
/// @param[in] original_projections original forward projection linestrings
/// @param[in] adjusted_projections adjusted forward projection linestrings
/// @param[in] original_footprints original forward projection footprints
/// @param[in] adjusted_footprints adjusted forward projection footprints
/// @param[in] marker_z z-value to use for markers
/// @return marker array
visualization_msgs::msg::MarkerArray makeDebugMarkers(
  const Obstacles & obstacles, const std::vector<multi_linestring_t> & original_projections,
  const std::vector<multi_linestring_t> & adjusted_projections,
  const std::vector<polygon_t> & original_footprints,
  const std::vector<polygon_t> & adjusted_footprints, const ObstacleMasks & obstacle_masks,
  const Float marker_z);

}  // namespace obstacle_velocity_limiter
#endif  // OBSTACLE_VELOCITY_LIMITER__DEBUG_HPP_
