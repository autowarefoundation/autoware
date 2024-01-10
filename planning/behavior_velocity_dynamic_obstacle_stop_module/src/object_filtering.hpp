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

#ifndef OBJECT_FILTERING_HPP_
#define OBJECT_FILTERING_HPP_

#include "types.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop
{

/// @brief filter the given predicted objects
/// @param objects predicted objects
/// @param ego_data ego data, including its path and pose
/// @param params parameters
/// @param hysteresis [m] extra distance threshold used for filtering
/// @return filtered predicted objects
std::vector<autoware_auto_perception_msgs::msg::PredictedObject> filter_predicted_objects(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects, const EgoData & ego_data,
  const PlannerParam & params, const double hysteresis);

}  // namespace behavior_velocity_planner::dynamic_obstacle_stop

#endif  // OBJECT_FILTERING_HPP_
