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

#include "obstacle_cruise_planner/utils.hpp"

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

namespace obstacle_cruise_utils
{
bool isVehicle(const uint8_t label)
{
  return label == ObjectClassification::CAR || label == ObjectClassification::TRUCK ||
         label == ObjectClassification::BUS || label == ObjectClassification::MOTORCYCLE;
}

visualization_msgs::msg::Marker getObjectMarker(
  const geometry_msgs::msg::Pose & obstacle_pose, size_t idx, const std::string & ns,
  const double r, const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();

  auto marker = tier4_autoware_utils::createDefaultMarker(
    "map", current_time, ns, idx, visualization_msgs::msg::Marker::SPHERE,
    tier4_autoware_utils::createMarkerScale(2.0, 2.0, 2.0),
    tier4_autoware_utils::createMarkerColor(r, g, b, 0.8));

  marker.lifetime = rclcpp::Duration::from_seconds(0.8);
  marker.pose = obstacle_pose;

  return marker;
}

boost::optional<geometry_msgs::msg::Pose> calcForwardPose(
  const autoware_auto_planning_msgs::msg::Trajectory & traj, const size_t nearest_idx,
  const double target_length)
{
  if (traj.points.empty()) {
    return {};
  }

  size_t search_idx = nearest_idx;
  double length_to_search_idx = 0.0;
  for (; search_idx < traj.points.size(); ++search_idx) {
    length_to_search_idx =
      tier4_autoware_utils::calcSignedArcLength(traj.points, nearest_idx, search_idx);
    if (length_to_search_idx > target_length) {
      break;
    } else if (search_idx == traj.points.size() - 1) {
      return {};
    }
  }

  if (search_idx == 0 && !traj.points.empty()) {
    return traj.points.at(0).pose;
  }

  const auto & pre_pose = traj.points.at(search_idx - 1).pose;
  const auto & next_pose = traj.points.at(search_idx).pose;

  geometry_msgs::msg::Pose target_pose;

  // lerp position
  const double seg_length =
    tier4_autoware_utils::calcDistance2d(pre_pose.position, next_pose.position);
  const double lerp_ratio = (length_to_search_idx - target_length) / seg_length;
  target_pose.position.x =
    pre_pose.position.x + (next_pose.position.x - pre_pose.position.x) * lerp_ratio;
  target_pose.position.y =
    pre_pose.position.y + (next_pose.position.y - pre_pose.position.y) * lerp_ratio;
  target_pose.position.z =
    pre_pose.position.z + (next_pose.position.z - pre_pose.position.z) * lerp_ratio;

  // lerp orientation
  const double pre_yaw = tf2::getYaw(pre_pose.orientation);
  const double next_yaw = tf2::getYaw(next_pose.orientation);
  target_pose.orientation =
    tier4_autoware_utils::createQuaternionFromYaw(pre_yaw + (next_yaw - pre_yaw) * lerp_ratio);

  return target_pose;
}

boost::optional<geometry_msgs::msg::Pose> getCurrentObjectPoseFromPredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const rclcpp::Time & obj_base_time, const rclcpp::Time & current_time)
{
  for (size_t i = 0; i < predicted_path.path.size(); ++i) {
    const auto & obj_p = predicted_path.path.at(i);

    const double object_time =
      (obj_base_time + rclcpp::Duration(predicted_path.time_step) * static_cast<double>(i) -
       current_time)
        .seconds();
    if (object_time >= 0) {
      return obj_p;
    }
  }

  return boost::none;
}
}  // namespace obstacle_cruise_utils
