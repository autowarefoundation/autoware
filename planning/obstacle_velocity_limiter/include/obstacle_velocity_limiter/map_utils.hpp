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

#ifndef OBSTACLE_VELOCITY_LIMITER__MAP_UTILS_HPP_
#define OBSTACLE_VELOCITY_LIMITER__MAP_UTILS_HPP_

#include "obstacle_velocity_limiter/types.hpp"

#include <lanelet2_core/LaneletMap.h>

#include <string>
#include <vector>

namespace obstacle_velocity_limiter
{

/// @brief Extract static obstacles from the lanelet map
/// @param[in] lanelet_map lanelet map
/// @param[in] tags tags to identify obstacle linestrings
/// @return the extracted obstacles
multi_linestring_t extractStaticObstacles(
  const lanelet::LaneletMap & lanelet_map, const std::vector<std::string> & tags);

/// @brief Determine if the given linestring is an obstacle
/// @param[in] ls linestring to check
/// @param[in] tags obstacle tags
/// @return true if the linestring is an obstacle
bool isObstacle(const lanelet::ConstLineString3d & ls, const std::vector<std::string> & tags);
}  // namespace obstacle_velocity_limiter

#endif  // OBSTACLE_VELOCITY_LIMITER__MAP_UTILS_HPP_
