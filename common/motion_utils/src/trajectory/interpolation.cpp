// Copyright 2022 Tier IV, Inc.
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

#include "motion_utils/trajectory/interpolation.hpp"

#include "interpolation/linear_interpolation.hpp"
#include "motion_utils/trajectory/trajectory.hpp"

using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

namespace motion_utils
{
TrajectoryPoint calcInterpolatedPoint(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & target_pose,
  const bool use_zero_order_hold_for_twist, const double dist_threshold, const double yaw_threshold)
{
  if (trajectory.points.empty()) {
    TrajectoryPoint interpolated_point{};
    interpolated_point.pose = target_pose;
    return interpolated_point;
  }
  if (trajectory.points.size() == 1) {
    return trajectory.points.front();
  }

  const size_t segment_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    trajectory.points, target_pose, dist_threshold, yaw_threshold);

  // Calculate interpolation ratio
  const auto & curr_pt = trajectory.points.at(segment_idx);
  const auto & next_pt = trajectory.points.at(segment_idx + 1);
  const auto v1 = tier4_autoware_utils::point2tfVector(curr_pt, next_pt);
  const auto v2 = tier4_autoware_utils::point2tfVector(curr_pt, target_pose);
  if (v1.length2() < 1e-3) {
    return curr_pt;
  }

  const double ratio = v1.dot(v2) / v1.length2();
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);

  // Interpolate
  TrajectoryPoint interpolated_point{};

  // pose interpolation
  interpolated_point.pose =
    tier4_autoware_utils::calcInterpolatedPose(curr_pt, next_pt, clamped_ratio);

  // twist interpolation
  if (use_zero_order_hold_for_twist) {
    interpolated_point.longitudinal_velocity_mps = curr_pt.longitudinal_velocity_mps;
    interpolated_point.lateral_velocity_mps = curr_pt.lateral_velocity_mps;
    interpolated_point.acceleration_mps2 = curr_pt.acceleration_mps2;
  } else {
    interpolated_point.longitudinal_velocity_mps = interpolation::lerp(
      curr_pt.longitudinal_velocity_mps, next_pt.longitudinal_velocity_mps, clamped_ratio);
    interpolated_point.lateral_velocity_mps = interpolation::lerp(
      curr_pt.lateral_velocity_mps, next_pt.lateral_velocity_mps, clamped_ratio);
    interpolated_point.acceleration_mps2 =
      interpolation::lerp(curr_pt.acceleration_mps2, next_pt.acceleration_mps2, clamped_ratio);
  }

  // heading rate interpolation
  interpolated_point.heading_rate_rps =
    interpolation::lerp(curr_pt.heading_rate_rps, next_pt.heading_rate_rps, clamped_ratio);

  // wheel interpolation
  interpolated_point.front_wheel_angle_rad = interpolation::lerp(
    curr_pt.front_wheel_angle_rad, next_pt.front_wheel_angle_rad, clamped_ratio);
  interpolated_point.rear_wheel_angle_rad =
    interpolation::lerp(curr_pt.rear_wheel_angle_rad, next_pt.rear_wheel_angle_rad, clamped_ratio);

  // time interpolation
  const double interpolated_time = interpolation::lerp(
    rclcpp::Duration(curr_pt.time_from_start).seconds(),
    rclcpp::Duration(next_pt.time_from_start).seconds(), clamped_ratio);
  interpolated_point.time_from_start = rclcpp::Duration::from_seconds(interpolated_time);

  return interpolated_point;
}

PathPointWithLaneId calcInterpolatedPoint(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & target_pose,
  const bool use_zero_order_hold_for_twist, const double dist_threshold, const double yaw_threshold)
{
  if (path.points.empty()) {
    PathPointWithLaneId interpolated_point{};
    interpolated_point.point.pose = target_pose;
    return interpolated_point;
  }
  if (path.points.size() == 1) {
    return path.points.front();
  }

  const size_t segment_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, target_pose, dist_threshold, yaw_threshold);

  // Calculate interpolation ratio
  const auto & curr_pt = path.points.at(segment_idx);
  const auto & next_pt = path.points.at(segment_idx + 1);
  const auto v1 = tier4_autoware_utils::point2tfVector(curr_pt.point, next_pt.point);
  const auto v2 = tier4_autoware_utils::point2tfVector(curr_pt.point, target_pose);
  if (v1.length2() < 1e-3) {
    return curr_pt;
  }

  const double ratio = v1.dot(v2) / v1.length2();
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);

  // Interpolate
  PathPointWithLaneId interpolated_point{};

  // pose interpolation
  interpolated_point.point.pose =
    tier4_autoware_utils::calcInterpolatedPose(curr_pt.point, next_pt.point, clamped_ratio);

  // twist interpolation
  if (use_zero_order_hold_for_twist) {
    interpolated_point.point.longitudinal_velocity_mps = curr_pt.point.longitudinal_velocity_mps;
    interpolated_point.point.lateral_velocity_mps = curr_pt.point.lateral_velocity_mps;
  } else {
    interpolated_point.point.longitudinal_velocity_mps = interpolation::lerp(
      curr_pt.point.longitudinal_velocity_mps, next_pt.point.longitudinal_velocity_mps,
      clamped_ratio);
    interpolated_point.point.lateral_velocity_mps = interpolation::lerp(
      curr_pt.point.lateral_velocity_mps, next_pt.point.lateral_velocity_mps, clamped_ratio);
  }

  // heading rate interpolation
  interpolated_point.point.heading_rate_rps = interpolation::lerp(
    curr_pt.point.heading_rate_rps, next_pt.point.heading_rate_rps, clamped_ratio);

  return interpolated_point;
}
}  // namespace motion_utils
