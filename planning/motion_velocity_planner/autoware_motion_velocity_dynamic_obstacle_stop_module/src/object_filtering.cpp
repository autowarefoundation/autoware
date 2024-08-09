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

#include "object_filtering.hpp"

#include "types.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <geometry_msgs/msg/detail/pose__struct.hpp>

#include <algorithm>
#include <vector>

namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
{

bool is_vehicle(const autoware_perception_msgs::msg::PredictedObject & object)
{
  constexpr auto is_vehicle_class = [](const auto & c) {
    return c.label == autoware_perception_msgs::msg::ObjectClassification::CAR ||
           c.label == autoware_perception_msgs::msg::ObjectClassification::BUS ||
           c.label == autoware_perception_msgs::msg::ObjectClassification::TRUCK ||
           c.label == autoware_perception_msgs::msg::ObjectClassification::TRAILER ||
           c.label == autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE ||
           c.label == autoware_perception_msgs::msg::ObjectClassification::BICYCLE;
  };
  return std::find_if(
           object.classification.begin(), object.classification.end(), is_vehicle_class) !=
         object.classification.end();
}

bool is_in_range(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const TrajectoryPoints & ego_trajectory, const PlannerParam & params, const double hysteresis)
{
  const auto distance = std::abs(motion_utils::calcLateralOffset(
    ego_trajectory, object.kinematics.initial_pose_with_covariance.pose.position));
  return distance <= params.minimum_object_distance_from_ego_trajectory +
                       params.ego_lateral_offset + object.shape.dimensions.y / 2.0 + hysteresis;
};

bool is_not_too_close(
  const autoware_perception_msgs::msg::PredictedObject & object, const EgoData & ego_data,
  const double & ego_longitudinal_offset)
{
  const auto obj_arc_length = motion_utils::calcSignedArcLength(
    ego_data.trajectory, ego_data.pose.position,
    object.kinematics.initial_pose_with_covariance.pose.position);
  return std::abs(obj_arc_length) > ego_data.longitudinal_offset_to_first_trajectory_idx +
                                      ego_longitudinal_offset + object.shape.dimensions.x / 2.0;
};

bool is_unavoidable(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const geometry_msgs::msg::Pose & ego_pose,
  const std::optional<geometry_msgs::msg::Pose> & ego_earliest_stop_pose,
  const PlannerParam & params)
{
  constexpr auto same_direction_diff_threshold = M_PI_2 + M_PI_4;
  const auto & o_pose = object.kinematics.initial_pose_with_covariance.pose;
  const auto o_yaw = tf2::getYaw(o_pose.orientation);
  const auto ego_yaw = tf2::getYaw(ego_pose.orientation);
  const auto yaw_diff = std::abs(universe_utils::normalizeRadian(o_yaw - ego_yaw));
  const auto opposite_heading = yaw_diff > same_direction_diff_threshold;
  const auto collision_distance_threshold =
    params.ego_lateral_offset + object.shape.dimensions.y / 2.0 + params.hysteresis;
  const auto lat_distance =
    std::abs(universe_utils::calcLateralDeviation(o_pose, ego_pose.position));
  auto has_collision = opposite_heading && lat_distance <= collision_distance_threshold;
  if (ego_earliest_stop_pose) {
    const auto direction_yaw = std::atan2(
      o_pose.position.y - ego_earliest_stop_pose->position.y,
      o_pose.position.x - ego_earliest_stop_pose->position.x);
    const auto yaw_diff_at_earliest_stop_pose =
      std::abs(universe_utils::normalizeRadian(o_yaw - direction_yaw));
    const auto lat_distance_at_earliest_stop_pose =
      std::abs(universe_utils::calcLateralDeviation(o_pose, ego_earliest_stop_pose->position));
    const auto collision_at_earliest_stop_pose =
      yaw_diff_at_earliest_stop_pose > same_direction_diff_threshold &&
      lat_distance_at_earliest_stop_pose <= collision_distance_threshold;
    has_collision |= collision_at_earliest_stop_pose;
  }
  return has_collision;
};

std::vector<autoware_perception_msgs::msg::PredictedObject> filter_predicted_objects(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const EgoData & ego_data,
  const PlannerParam & params, const double hysteresis)
{
  std::vector<autoware_perception_msgs::msg::PredictedObject> filtered_objects;
  for (const auto & object : objects.objects) {
    const auto is_not_too_slow = object.kinematics.initial_twist_with_covariance.twist.linear.x >=
                                 params.minimum_object_velocity;
    if (
      is_vehicle(object) && is_not_too_slow &&
      is_in_range(object, ego_data.trajectory, params, hysteresis) &&
      is_not_too_close(object, ego_data, params.ego_longitudinal_offset) &&
      (!params.ignore_unavoidable_collisions ||
       !is_unavoidable(object, ego_data.pose, ego_data.earliest_stop_pose, params)))
      filtered_objects.push_back(object);
  }
  return filtered_objects;
}
}  // namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
