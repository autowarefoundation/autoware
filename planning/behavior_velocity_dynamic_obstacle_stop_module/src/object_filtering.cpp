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

#include "object_filtering.hpp"

#include "types.hpp"

#include <motion_utils/trajectory/trajectory.hpp>

#include <algorithm>
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
  const PlannerParam & params, const double hysteresis)
{
  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> filtered_objects;
  const auto is_vehicle = [](const auto & o) {
    return std::find_if(o.classification.begin(), o.classification.end(), [](const auto & c) {
             return c.label == autoware_auto_perception_msgs::msg::ObjectClassification::CAR ||
                    c.label == autoware_auto_perception_msgs::msg::ObjectClassification::BUS ||
                    c.label == autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK ||
                    c.label == autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER ||
                    c.label ==
                      autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE ||
                    c.label == autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE;
           }) != o.classification.end();
  };
  const auto is_in_range = [&](const auto & o) {
    const auto distance = std::abs(motion_utils::calcLateralOffset(
      ego_data.path.points, o.kinematics.initial_pose_with_covariance.pose.position));
    return distance <= params.minimum_object_distance_from_ego_path + params.ego_lateral_offset +
                         o.shape.dimensions.y / 2.0 + hysteresis;
  };
  const auto is_not_too_close = [&](const auto & o) {
    const auto obj_arc_length = motion_utils::calcSignedArcLength(
      ego_data.path.points, ego_data.pose.position,
      o.kinematics.initial_pose_with_covariance.pose.position);
    return obj_arc_length > ego_data.longitudinal_offset_to_first_path_idx +
                              params.ego_longitudinal_offset + o.shape.dimensions.x / 2.0;
  };
  const auto is_unavoidable = [&](const auto & o) {
    const auto & o_pose = o.kinematics.initial_pose_with_covariance.pose;
    const auto o_yaw = tf2::getYaw(o_pose.orientation);
    const auto ego_yaw = tf2::getYaw(ego_data.pose.orientation);
    const auto yaw_diff = std::abs(tier4_autoware_utils::normalizeRadian(o_yaw - ego_yaw));
    const auto opposite_heading = yaw_diff > M_PI_2 + M_PI_4;
    const auto collision_distance_threshold =
      params.ego_lateral_offset + o.shape.dimensions.y / 2.0 + params.hysteresis;
    // https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_point_and_angle
    const auto lat_distance = std::abs(
      (o_pose.position.y - ego_data.pose.position.y) * std::cos(o_yaw) -
      (o_pose.position.x - ego_data.pose.position.x) * std::sin(o_yaw));
    auto has_collision = lat_distance <= collision_distance_threshold;
    if (ego_data.earliest_stop_pose) {
      const auto collision_at_earliest_stop_pose =
        std::abs(
          (o_pose.position.y - ego_data.earliest_stop_pose->position.y) * std::cos(o_yaw) -
          (o_pose.position.x - ego_data.earliest_stop_pose->position.x) * std::sin(o_yaw)) <=
        collision_distance_threshold;
      has_collision |= collision_at_earliest_stop_pose;
    }
    return opposite_heading && has_collision;
  };
  for (const auto & object : objects.objects) {
    const auto is_not_too_slow = object.kinematics.initial_twist_with_covariance.twist.linear.x >=
                                 params.minimum_object_velocity;
    if (
      is_vehicle(object) && is_not_too_slow && is_in_range(object) && is_not_too_close(object) &&
      (!params.ignore_unavoidable_collisions || !is_unavoidable(object)))
      filtered_objects.push_back(object);
  }
  return filtered_objects;
}
}  // namespace behavior_velocity_planner::dynamic_obstacle_stop
