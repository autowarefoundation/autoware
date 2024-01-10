// Copyright 2023 TIER IV, Inc.
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

#ifndef COLLISION_HPP_
#define COLLISION_HPP_

#include "types.hpp"

#include <optional>
#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop
{

/// @brief find the earliest collision along the ego path and an obstacle footprint
/// @param [in] ego_data ego data including its path and footprint
/// @param [in] objects obstacles
/// @param [in] obstacle_forward_footprints obstacle footprints
/// @param [in] debug_data debug data
/// @return the point of earliest collision along the ego path
std::optional<geometry_msgs::msg::Point> find_earliest_collision(
  const EgoData & ego_data,
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
  const tier4_autoware_utils::MultiPolygon2d & obstacle_forward_footprints, DebugData & debug_data);

}  // namespace behavior_velocity_planner::dynamic_obstacle_stop

#endif  // COLLISION_HPP_
