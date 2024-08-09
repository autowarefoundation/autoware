// Copyright 2023-2024 TIER IV, Inc.
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

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <vector>

namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
{
/// @brief check if the object is a vehicle
/// @details the object classification labels are used to determine if the object is a vehicle
/// (classification probabilities are ignored)
/// @param object a predicted object with a label to check
/// @return true if the object is a vehicle
bool is_vehicle(const autoware_perception_msgs::msg::PredictedObject & object);

/// @brief check if the object is within the lateral range of the ego trajectory
/// @details the lateral offset of the object position to the ego trajectory is compared to the ego
/// + object width + minimum_object_distance_from_ego_trajectory parameter + hysteresis
/// @param object a predicted object with its pose and the lateral offset of its shape
/// @param ego_trajectory trajectory of the ego vehicle
/// @param params parameters with the minimum lateral range and the ego lateral offset
/// @param hysteresis [m] currently used hysteresis distance
/// @return true if the object is in range of the ego trajectory
bool is_in_range(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const TrajectoryPoints & ego_trajectory, const PlannerParam & params, const double hysteresis);

/// @brief check if an object is not too close to the ego vehicle
/// @details the arc length difference of the ego and of the object position on the ego trajectory
/// are compared with the ego and object length
/// @param object a predicted object with its pose and shape
/// @param ego_data ego data with its trajectory, pose, and arc length of the pose on the trajectory
/// @param ego_earliest_stop_pose pose the ego vehicle would reach if it were to stop now
/// @param params parameters with the lateral ego offset distance and the hysteresis distance
/// @return true if the object is in range of the ego vehicle
bool is_not_too_close(
  const autoware_perception_msgs::msg::PredictedObject & object, const EgoData & ego_data,
  const double & ego_longitudinal_offset);

/// @brief check if a collision would occur with the object, even if ego were stopping now
/// @details we roughly check for a collision at the current ego pose or the earliest stop pose
/// @param object a predicted object with a label to check
/// @param ego_pose pose of the ego vehicle
/// @param ego_earliest_stop_pose pose the ego vehicle would reach if it were to stop now
/// @param params parameters with the lateral ego offset distance and the hysteresis distance
/// @return true if the object is in range of the ego vehicle
bool is_unavoidable(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const geometry_msgs::msg::Pose & ego_pose,
  const std::optional<geometry_msgs::msg::Pose> & ego_earliest_stop_pose,
  const PlannerParam & params);

/// @brief filter the given predicted objects
/// @param objects predicted objects
/// @param ego_data ego data, including its trajectory and pose
/// @param params parameters
/// @param hysteresis [m] extra distance threshold used for filtering
/// @return filtered predicted objects
std::vector<autoware_perception_msgs::msg::PredictedObject> filter_predicted_objects(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const EgoData & ego_data,
  const PlannerParam & params, const double hysteresis);

}  // namespace autoware::motion_velocity_planner::dynamic_obstacle_stop

#endif  // OBJECT_FILTERING_HPP_
