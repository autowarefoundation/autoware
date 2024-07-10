// Copyright 2024 TIER IV, Inc.
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

#include "autoware_radar_object_tracker/utils/radar_object_tracker_utils.hpp"

namespace autoware::radar_object_tracker::utils
{

boost::optional<geometry_msgs::msg::Transform> getTransformAnonymous(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
    std::string errstr;
    if (!tf_buffer.canTransform(
          target_frame_id, source_frame_id, tf2::TimePointZero, tf2::Duration::zero(), &errstr)) {
      return boost::none;
    }

    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer.lookupTransform(
      target_frame_id, source_frame_id, time, rclcpp::Duration::from_seconds(0.5));
    return self_transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("radar_object_tracker"), ex.what());
    return boost::none;
  }
}

bool isDuplicated(
  const std::pair<double, lanelet::ConstLanelet> & target_lanelet,
  const lanelet::ConstLanelets & lanelets)
{
  const double CLOSE_LANELET_THRESHOLD = 0.1;
  for (const auto & lanelet : lanelets) {
    const auto target_lanelet_end_p = target_lanelet.second.centerline2d().back();
    const auto lanelet_end_p = lanelet.centerline2d().back();
    const double dist = std::hypot(
      target_lanelet_end_p.x() - lanelet_end_p.x(), target_lanelet_end_p.y() - lanelet_end_p.y());
    if (dist < CLOSE_LANELET_THRESHOLD) {
      return true;
    }
  }

  return false;
}

bool checkCloseLaneletCondition(
  const std::pair<double, lanelet::Lanelet> & lanelet,
  const autoware_perception_msgs::msg::TrackedObject & object, const double max_distance_from_lane,
  const double max_angle_diff_from_lane)
{
  if (lanelet.second.centerline().size() <= 1) {
    return false;
  }

  lanelet::BasicPoint2d search_point(
    object.kinematics.pose_with_covariance.pose.position.x,
    object.kinematics.pose_with_covariance.pose.position.y);
  if (!lanelet::geometry::inside(lanelet.second, search_point)) {
    const auto distance = lanelet.first;
    if (distance > max_distance_from_lane) {
      return false;
    }
  }

  const double object_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  const double lane_yaw = lanelet::utils::getLaneletAngle(
    lanelet.second, object.kinematics.pose_with_covariance.pose.position);
  double object_motion_yaw = object_yaw;
  bool velocity_is_reverted = object.kinematics.twist_with_covariance.twist.linear.x < 0.0;
  if (velocity_is_reverted) {
    object_motion_yaw = autoware::universe_utils::normalizeRadian(object_yaw + M_PI);
  }
  const double delta_yaw = object_motion_yaw - lane_yaw;
  const double normalized_delta_yaw = autoware::universe_utils::normalizeRadian(delta_yaw);
  const double abs_norm_delta_yaw = std::fabs(normalized_delta_yaw);

  if (abs_norm_delta_yaw > max_angle_diff_from_lane) {
    return false;
  }

  return true;
}

lanelet::ConstLanelets getClosestValidLanelets(
  const autoware_perception_msgs::msg::TrackedObject & object,
  const lanelet::LaneletMapPtr & lanelet_map_ptr, const double max_distance_from_lane,
  const double max_angle_diff_from_lane)
{
  lanelet::BasicPoint2d search_point(
    object.kinematics.pose_with_covariance.pose.position.x,
    object.kinematics.pose_with_covariance.pose.position.y);

  std::vector<std::pair<double, lanelet::Lanelet>> surrounding_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr->laneletLayer, search_point, 10);

  if (surrounding_lanelets.empty()) {
    return {};
  }

  lanelet::ConstLanelets closest_lanelets;
  for (const auto & lanelet : surrounding_lanelets) {
    if (
      !checkCloseLaneletCondition(
        lanelet, object, max_distance_from_lane, max_angle_diff_from_lane) ||
      isDuplicated(lanelet, closest_lanelets)) {
      continue;
    }

    closest_lanelets.push_back(lanelet.second);
  }

  return closest_lanelets;
}

bool hasValidVelocityDirectionToLanelet(
  const autoware_perception_msgs::msg::TrackedObject & object,
  const lanelet::ConstLanelets & lanelets, const double max_lateral_velocity)
{
  const double object_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  const double object_vel_x = object.kinematics.twist_with_covariance.twist.linear.x;
  const double object_vel_y = object.kinematics.twist_with_covariance.twist.linear.y;
  const double object_vel_yaw = std::atan2(object_vel_y, object_vel_x);
  const double object_vel_yaw_global =
    autoware::universe_utils::normalizeRadian(object_yaw + object_vel_yaw);
  const double object_vel = std::hypot(object_vel_x, object_vel_y);

  for (const auto & lanelet : lanelets) {
    const double lane_yaw = lanelet::utils::getLaneletAngle(
      lanelet, object.kinematics.pose_with_covariance.pose.position);
    const double delta_yaw = object_vel_yaw_global - lane_yaw;
    const double normalized_delta_yaw = autoware::universe_utils::normalizeRadian(delta_yaw);

    const double lane_vel = object_vel * std::sin(normalized_delta_yaw);
    if (std::fabs(lane_vel) < max_lateral_velocity) {
      return true;
    }
  }
  return false;
}

}  // namespace autoware::radar_object_tracker::utils
