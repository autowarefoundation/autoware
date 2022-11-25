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

#include "motion_utils/resample/resample.hpp"

#include "motion_utils/resample/resample_utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

namespace motion_utils
{
std::vector<geometry_msgs::msg::Pose> resamplePoseVector(
  const std::vector<geometry_msgs::msg::Pose> & points,
  const std::vector<double> & resampled_arclength, const bool use_lerp_for_xy,
  const bool use_lerp_for_z)
{
  // validate arguments
  if (!resample_utils::validate_arguments(points, resampled_arclength)) {
    return points;
  }

  // Input Path Information
  std::vector<double> input_arclength;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  input_arclength.reserve(points.size());
  x.reserve(points.size());
  y.reserve(points.size());
  z.reserve(points.size());

  input_arclength.push_back(0.0);
  x.push_back(points.front().position.x);
  y.push_back(points.front().position.y);
  z.push_back(points.front().position.z);
  for (size_t i = 1; i < points.size(); ++i) {
    const auto & prev_pt = points.at(i - 1);
    const auto & curr_pt = points.at(i);
    const double ds = tier4_autoware_utils::calcDistance2d(prev_pt.position, curr_pt.position);
    input_arclength.push_back(ds + input_arclength.back());
    x.push_back(curr_pt.position.x);
    y.push_back(curr_pt.position.y);
    z.push_back(curr_pt.position.z);
  }

  // Interpolate
  const auto lerp = [&](const auto & input) {
    return interpolation::lerp(input_arclength, input, resampled_arclength);
  };
  const auto spline = [&](const auto & input) {
    return interpolation::spline(input_arclength, input, resampled_arclength);
  };

  const auto interpolated_x = use_lerp_for_xy ? lerp(x) : spline(x);
  const auto interpolated_y = use_lerp_for_xy ? lerp(y) : spline(y);
  const auto interpolated_z = use_lerp_for_z ? lerp(z) : spline(z);

  std::vector<geometry_msgs::msg::Pose> resampled_points;
  resampled_points.resize(interpolated_x.size());

  // Insert Position, Velocity and Heading Rate
  for (size_t i = 0; i < resampled_points.size(); ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = interpolated_x.at(i);
    pose.position.y = interpolated_y.at(i);
    pose.position.z = interpolated_z.at(i);
    resampled_points.at(i) = pose;
  }

  const bool is_driving_forward =
    tier4_autoware_utils::isDrivingForward(points.at(0), points.at(1));
  motion_utils::insertOrientation(resampled_points, is_driving_forward);

  // Initial orientation is depend on the initial value of the resampled_arclength
  // when backward driving
  if (!is_driving_forward && resampled_arclength.front() < 1e-3) {
    resampled_points.at(0).orientation = points.at(0).orientation;
  }

  return resampled_points;
}

std::vector<geometry_msgs::msg::Pose> resamplePoseVector(
  const std::vector<geometry_msgs::msg::Pose> & points, const double resample_interval,
  const bool use_lerp_for_xy, const bool use_lerp_for_z)
{
  const double input_length = motion_utils::calcArcLength(points);

  std::vector<double> resampling_arclength;
  for (double s = 0.0; s < input_length; s += resample_interval) {
    resampling_arclength.push_back(s);
  }
  if (resampling_arclength.empty()) {
    std::cerr << "[motion_utils]: resampling arclength is empty" << std::endl;
    return points;
  }

  // Insert terminal point
  if (input_length - resampling_arclength.back() < motion_utils::overlap_threshold) {
    resampling_arclength.back() = input_length;
  } else {
    resampling_arclength.push_back(input_length);
  }

  return resamplePoseVector(points, resampling_arclength, use_lerp_for_xy, use_lerp_for_z);
}

autoware_auto_planning_msgs::msg::PathWithLaneId resamplePath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path,
  const std::vector<double> & resampled_arclength, const bool use_lerp_for_xy,
  const bool use_lerp_for_z, const bool use_zero_order_hold_for_v)
{
  // validate arguments
  if (!resample_utils::validate_arguments(input_path.points, resampled_arclength)) {
    return input_path;
  }

  // For LaneIds, is_final
  //
  // ------|----|----|----|----|----|----|-------> resampled
  //      [0]  [1]  [2]  [3]  [4]  [5]  [6]
  //
  // ------|----------------|----------|---------> base
  //      [0]             [1]        [2]
  //
  // resampled[0~3] = base[0]
  // resampled[4~5] = base[1]
  // resampled[6] = base[2]

  // Input Path Information
  std::vector<double> input_arclength;
  std::vector<geometry_msgs::msg::Pose> input_pose;
  std::vector<double> v_lon;
  std::vector<double> v_lat;
  std::vector<double> heading_rate;
  std::vector<bool> is_final;
  std::vector<std::vector<int64_t>> lane_ids;
  input_arclength.reserve(input_path.points.size());
  input_pose.reserve(input_path.points.size());
  v_lon.reserve(input_path.points.size());
  v_lat.reserve(input_path.points.size());
  heading_rate.reserve(input_path.points.size());
  is_final.reserve(input_path.points.size());
  lane_ids.reserve(input_path.points.size());

  input_arclength.push_back(0.0);
  input_pose.push_back(input_path.points.front().point.pose);
  v_lon.push_back(input_path.points.front().point.longitudinal_velocity_mps);
  v_lat.push_back(input_path.points.front().point.lateral_velocity_mps);
  heading_rate.push_back(input_path.points.front().point.heading_rate_rps);
  is_final.push_back(input_path.points.front().point.is_final);
  lane_ids.push_back(input_path.points.front().lane_ids);
  for (size_t i = 1; i < input_path.points.size(); ++i) {
    const auto & prev_pt = input_path.points.at(i - 1).point;
    const auto & curr_pt = input_path.points.at(i).point;
    const double ds =
      tier4_autoware_utils::calcDistance2d(prev_pt.pose.position, curr_pt.pose.position);
    input_arclength.push_back(ds + input_arclength.back());
    input_pose.push_back(curr_pt.pose);
    v_lon.push_back(curr_pt.longitudinal_velocity_mps);
    v_lat.push_back(curr_pt.lateral_velocity_mps);
    heading_rate.push_back(curr_pt.heading_rate_rps);
    is_final.push_back(curr_pt.is_final);
    lane_ids.push_back(input_path.points.at(i).lane_ids);
  }

  if (input_arclength.back() < resampled_arclength.back()) {
    std::cerr << "[motion_utils]: resampled path length is longer than input path length"
              << std::endl;
    return input_path;
  }

  // Interpolate
  const auto lerp = [&](const auto & input) {
    return interpolation::lerp(input_arclength, input, resampled_arclength);
  };
  const auto zoh = [&](const auto & input) {
    return interpolation::zero_order_hold(input_arclength, input, resampled_arclength);
  };

  const auto interpolated_pose =
    resamplePoseVector(input_pose, resampled_arclength, use_lerp_for_xy, use_lerp_for_z);
  const auto interpolated_v_lon = use_zero_order_hold_for_v ? zoh(v_lon) : lerp(v_lon);
  const auto interpolated_v_lat = use_zero_order_hold_for_v ? zoh(v_lat) : lerp(v_lat);
  const auto interpolated_heading_rate = lerp(heading_rate);
  const auto interpolated_is_final = zoh(is_final);
  const auto interpolated_lane_ids = zoh(lane_ids);

  if (interpolated_pose.size() != resampled_arclength.size()) {
    std::cerr << "[motion_utils]: Resampled pose size is different from resampled arclength"
              << std::endl;
    return input_path;
  }

  autoware_auto_planning_msgs::msg::PathWithLaneId resampled_path;
  resampled_path.header = input_path.header;
  resampled_path.drivable_area = input_path.drivable_area;
  resampled_path.points.resize(interpolated_pose.size());
  for (size_t i = 0; i < resampled_path.points.size(); ++i) {
    autoware_auto_planning_msgs::msg::PathPoint path_point;
    path_point.pose = interpolated_pose.at(i);
    path_point.longitudinal_velocity_mps = interpolated_v_lon.at(i);
    path_point.lateral_velocity_mps = interpolated_v_lat.at(i);
    path_point.heading_rate_rps = interpolated_heading_rate.at(i);
    path_point.is_final = interpolated_is_final.at(i);
    resampled_path.points.at(i).point = path_point;
    resampled_path.points.at(i).lane_ids = interpolated_lane_ids.at(i);
  }

  return resampled_path;
}

autoware_auto_planning_msgs::msg::PathWithLaneId resamplePath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path,
  const double resample_interval, const bool use_lerp_for_xy, const bool use_lerp_for_z,
  const bool use_zero_order_hold_for_v, const bool resample_input_path_stop_point)
{
  // validate arguments
  if (!resample_utils::validate_arguments(input_path.points, resample_interval)) {
    return input_path;
  }

  // transform input_path
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> transformed_input_path(
    input_path.points.size());
  for (size_t i = 0; i < input_path.points.size(); ++i) {
    transformed_input_path.at(i) = input_path.points.at(i).point;
  }
  // compute path length
  const double input_path_len = motion_utils::calcArcLength(transformed_input_path);

  std::vector<double> resampling_arclength;
  for (double s = 0.0; s < input_path_len; s += resample_interval) {
    resampling_arclength.push_back(s);
  }
  if (resampling_arclength.empty()) {
    std::cerr << "[motion_utils]: resampling arclength is empty" << std::endl;
    return input_path;
  }

  // Insert terminal point
  if (input_path_len - resampling_arclength.back() < motion_utils::overlap_threshold) {
    resampling_arclength.back() = input_path_len;
  } else {
    resampling_arclength.push_back(input_path_len);
  }

  // Insert stop point
  if (resample_input_path_stop_point) {
    const auto distance_to_stop_point =
      motion_utils::calcDistanceToForwardStopPoint(transformed_input_path, 0);
    if (distance_to_stop_point && !resampling_arclength.empty()) {
      for (size_t i = 1; i < resampling_arclength.size(); ++i) {
        if (
          resampling_arclength.at(i - 1) <= *distance_to_stop_point &&
          *distance_to_stop_point < resampling_arclength.at(i)) {
          const double dist_to_prev_point =
            std::fabs(*distance_to_stop_point - resampling_arclength.at(i - 1));
          const double dist_to_following_point =
            std::fabs(resampling_arclength.at(i) - *distance_to_stop_point);
          if (dist_to_prev_point < motion_utils::overlap_threshold) {
            resampling_arclength.at(i - 1) = *distance_to_stop_point;
          } else if (dist_to_following_point < motion_utils::overlap_threshold) {
            resampling_arclength.at(i) = *distance_to_stop_point;
          } else {
            resampling_arclength.insert(resampling_arclength.begin() + i, *distance_to_stop_point);
          }
          break;
        }
      }
    }
  }

  return resamplePath(
    input_path, resampling_arclength, use_lerp_for_xy, use_lerp_for_z, use_zero_order_hold_for_v);
}

autoware_auto_planning_msgs::msg::Path resamplePath(
  const autoware_auto_planning_msgs::msg::Path & input_path,
  const std::vector<double> & resampled_arclength, const bool use_lerp_for_xy,
  const bool use_lerp_for_z, const bool use_zero_order_hold_for_v)
{
  // validate arguments
  if (!resample_utils::validate_arguments(input_path.points, resampled_arclength)) {
    return input_path;
  }

  // Input Path Information
  std::vector<double> input_arclength;
  std::vector<geometry_msgs::msg::Pose> input_pose;
  std::vector<double> v_lon;
  std::vector<double> v_lat;
  std::vector<double> heading_rate;
  input_arclength.reserve(input_path.points.size());
  input_pose.reserve(input_path.points.size());
  v_lon.reserve(input_path.points.size());
  v_lat.reserve(input_path.points.size());
  heading_rate.reserve(input_path.points.size());

  input_arclength.push_back(0.0);
  input_pose.push_back(input_path.points.front().pose);
  v_lon.push_back(input_path.points.front().longitudinal_velocity_mps);
  v_lat.push_back(input_path.points.front().lateral_velocity_mps);
  heading_rate.push_back(input_path.points.front().heading_rate_rps);
  for (size_t i = 1; i < input_path.points.size(); ++i) {
    const auto & prev_pt = input_path.points.at(i - 1);
    const auto & curr_pt = input_path.points.at(i);
    const double ds =
      tier4_autoware_utils::calcDistance2d(prev_pt.pose.position, curr_pt.pose.position);
    input_arclength.push_back(ds + input_arclength.back());
    input_pose.push_back(curr_pt.pose);
    v_lon.push_back(curr_pt.longitudinal_velocity_mps);
    v_lat.push_back(curr_pt.lateral_velocity_mps);
    heading_rate.push_back(curr_pt.heading_rate_rps);
  }

  // Interpolate
  const auto lerp = [&](const auto & input) {
    return interpolation::lerp(input_arclength, input, resampled_arclength);
  };
  const auto zoh = [&](const auto & input) {
    return interpolation::zero_order_hold(input_arclength, input, resampled_arclength);
  };

  const auto interpolated_pose =
    resamplePoseVector(input_pose, resampled_arclength, use_lerp_for_xy, use_lerp_for_z);
  const auto interpolated_v_lon = use_zero_order_hold_for_v ? zoh(v_lon) : lerp(v_lon);
  const auto interpolated_v_lat = use_zero_order_hold_for_v ? zoh(v_lat) : lerp(v_lat);
  const auto interpolated_heading_rate = lerp(heading_rate);

  if (interpolated_pose.size() != resampled_arclength.size()) {
    std::cerr << "[motion_utils]: Resampled pose size is different from resampled arclength"
              << std::endl;
    return input_path;
  }

  autoware_auto_planning_msgs::msg::Path resampled_path;
  resampled_path.header = input_path.header;
  resampled_path.drivable_area = input_path.drivable_area;
  resampled_path.points.resize(interpolated_pose.size());
  for (size_t i = 0; i < resampled_path.points.size(); ++i) {
    autoware_auto_planning_msgs::msg::PathPoint path_point;
    path_point.pose = interpolated_pose.at(i);
    path_point.longitudinal_velocity_mps = interpolated_v_lon.at(i);
    path_point.lateral_velocity_mps = interpolated_v_lat.at(i);
    path_point.heading_rate_rps = interpolated_heading_rate.at(i);
    resampled_path.points.at(i) = path_point;
  }

  return resampled_path;
}

autoware_auto_planning_msgs::msg::Path resamplePath(
  const autoware_auto_planning_msgs::msg::Path & input_path, const double resample_interval,
  const bool use_lerp_for_xy, const bool use_lerp_for_z, const bool use_zero_order_hold_for_twist,
  const bool resample_input_path_stop_point)
{
  // validate arguments
  if (!resample_utils::validate_arguments(input_path.points, resample_interval)) {
    return input_path;
  }

  const double input_path_len = motion_utils::calcArcLength(input_path.points);

  std::vector<double> resampling_arclength;
  for (double s = 0.0; s < input_path_len; s += resample_interval) {
    resampling_arclength.push_back(s);
  }
  if (resampling_arclength.empty()) {
    std::cerr << "[motion_utils]: resampling arclength is empty" << std::endl;
    return input_path;
  }

  // Insert terminal point
  if (input_path_len - resampling_arclength.back() < motion_utils::overlap_threshold) {
    resampling_arclength.back() = input_path_len;
  } else {
    resampling_arclength.push_back(input_path_len);
  }

  // Insert stop point
  if (resample_input_path_stop_point) {
    const auto distance_to_stop_point =
      motion_utils::calcDistanceToForwardStopPoint(input_path.points, 0);
    if (distance_to_stop_point && !resampling_arclength.empty()) {
      for (size_t i = 1; i < resampling_arclength.size(); ++i) {
        if (
          resampling_arclength.at(i - 1) <= *distance_to_stop_point &&
          *distance_to_stop_point < resampling_arclength.at(i)) {
          const double dist_to_prev_point =
            std::fabs(*distance_to_stop_point - resampling_arclength.at(i - 1));
          const double dist_to_following_point =
            std::fabs(resampling_arclength.at(i) - *distance_to_stop_point);
          if (dist_to_prev_point < motion_utils::overlap_threshold) {
            resampling_arclength.at(i - 1) = *distance_to_stop_point;
          } else if (dist_to_following_point < motion_utils::overlap_threshold) {
            resampling_arclength.at(i) = *distance_to_stop_point;
          } else {
            resampling_arclength.insert(resampling_arclength.begin() + i, *distance_to_stop_point);
          }
          break;
        }
      }
    }
  }

  return resamplePath(
    input_path, resampling_arclength, use_lerp_for_xy, use_lerp_for_z,
    use_zero_order_hold_for_twist);
}

autoware_auto_planning_msgs::msg::Trajectory resampleTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & input_trajectory,
  const std::vector<double> & resampled_arclength, const bool use_lerp_for_xy,
  const bool use_lerp_for_z, const bool use_zero_order_hold_for_twist)
{
  // validate arguments
  if (!resample_utils::validate_arguments(input_trajectory.points, resampled_arclength)) {
    return input_trajectory;
  }

  // Input Trajectory Information
  std::vector<double> input_arclength;
  std::vector<geometry_msgs::msg::Pose> input_pose;
  std::vector<double> v_lon;
  std::vector<double> v_lat;
  std::vector<double> heading_rate;
  std::vector<double> acceleration;
  std::vector<double> front_wheel_angle;
  std::vector<double> rear_wheel_angle;
  std::vector<double> time_from_start;
  input_arclength.reserve(input_trajectory.points.size());
  input_pose.reserve(input_trajectory.points.size());
  v_lon.reserve(input_trajectory.points.size());
  v_lat.reserve(input_trajectory.points.size());
  heading_rate.reserve(input_trajectory.points.size());
  acceleration.reserve(input_trajectory.points.size());
  front_wheel_angle.reserve(input_trajectory.points.size());
  rear_wheel_angle.reserve(input_trajectory.points.size());
  time_from_start.reserve(input_trajectory.points.size());

  input_arclength.push_back(0.0);
  input_pose.push_back(input_trajectory.points.front().pose);
  v_lon.push_back(input_trajectory.points.front().longitudinal_velocity_mps);
  v_lat.push_back(input_trajectory.points.front().lateral_velocity_mps);
  heading_rate.push_back(input_trajectory.points.front().heading_rate_rps);
  acceleration.push_back(input_trajectory.points.front().acceleration_mps2);
  front_wheel_angle.push_back(input_trajectory.points.front().front_wheel_angle_rad);
  rear_wheel_angle.push_back(input_trajectory.points.front().rear_wheel_angle_rad);
  time_from_start.push_back(
    rclcpp::Duration(input_trajectory.points.front().time_from_start).seconds());
  for (size_t i = 1; i < input_trajectory.points.size(); ++i) {
    const auto & prev_pt = input_trajectory.points.at(i - 1);
    const auto & curr_pt = input_trajectory.points.at(i);
    const double ds =
      tier4_autoware_utils::calcDistance2d(prev_pt.pose.position, curr_pt.pose.position);
    input_arclength.push_back(ds + input_arclength.back());
    input_pose.push_back(curr_pt.pose);
    v_lon.push_back(curr_pt.longitudinal_velocity_mps);
    v_lat.push_back(curr_pt.lateral_velocity_mps);
    heading_rate.push_back(curr_pt.heading_rate_rps);
    acceleration.push_back(curr_pt.acceleration_mps2);
    front_wheel_angle.push_back(curr_pt.front_wheel_angle_rad);
    rear_wheel_angle.push_back(curr_pt.rear_wheel_angle_rad);
    time_from_start.push_back(rclcpp::Duration(curr_pt.time_from_start).seconds());
  }

  // Interpolate
  const auto lerp = [&](const auto & input) {
    return interpolation::lerp(input_arclength, input, resampled_arclength);
  };
  const auto zoh = [&](const auto & input) {
    return interpolation::zero_order_hold(input_arclength, input, resampled_arclength);
  };

  const auto interpolated_pose =
    resamplePoseVector(input_pose, resampled_arclength, use_lerp_for_xy, use_lerp_for_z);
  const auto interpolated_v_lon = use_zero_order_hold_for_twist ? zoh(v_lon) : lerp(v_lon);
  const auto interpolated_v_lat = use_zero_order_hold_for_twist ? zoh(v_lat) : lerp(v_lat);
  const auto interpolated_heading_rate = lerp(heading_rate);
  const auto interpolated_acceleration =
    use_zero_order_hold_for_twist ? zoh(acceleration) : lerp(acceleration);
  const auto interpolated_front_wheel_angle = lerp(front_wheel_angle);
  const auto interpolated_rear_wheel_angle = lerp(rear_wheel_angle);
  const auto interpolated_time_from_start = lerp(time_from_start);

  if (interpolated_pose.size() != resampled_arclength.size()) {
    std::cerr << "[motion_utils]: Resampled pose size is different from resampled arclength"
              << std::endl;
    return input_trajectory;
  }

  autoware_auto_planning_msgs::msg::Trajectory resampled_trajectory;
  resampled_trajectory.header = input_trajectory.header;
  resampled_trajectory.points.resize(interpolated_pose.size());
  for (size_t i = 0; i < resampled_trajectory.points.size(); ++i) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint traj_point;
    traj_point.pose = interpolated_pose.at(i);
    traj_point.longitudinal_velocity_mps = interpolated_v_lon.at(i);
    traj_point.lateral_velocity_mps = interpolated_v_lat.at(i);
    traj_point.heading_rate_rps = interpolated_heading_rate.at(i);
    traj_point.acceleration_mps2 = interpolated_acceleration.at(i);
    traj_point.front_wheel_angle_rad = interpolated_front_wheel_angle.at(i);
    traj_point.rear_wheel_angle_rad = interpolated_rear_wheel_angle.at(i);
    traj_point.time_from_start = rclcpp::Duration::from_seconds(interpolated_time_from_start.at(i));
    resampled_trajectory.points.at(i) = traj_point;
  }

  return resampled_trajectory;
}

autoware_auto_planning_msgs::msg::Trajectory resampleTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & input_trajectory,
  const double resample_interval, const bool use_lerp_for_xy, const bool use_lerp_for_z,
  const bool use_zero_order_hold_for_twist, const bool resample_input_trajectory_stop_point)
{
  // validate arguments
  if (!resample_utils::validate_arguments(input_trajectory.points, resample_interval)) {
    return input_trajectory;
  }

  const double input_trajectory_len = motion_utils::calcArcLength(input_trajectory.points);

  std::vector<double> resampling_arclength;
  for (double s = 0.0; s < input_trajectory_len; s += resample_interval) {
    resampling_arclength.push_back(s);
  }
  if (resampling_arclength.empty()) {
    std::cerr << "[motion_utils]: resampling arclength is empty" << std::endl;
    return input_trajectory;
  }

  // Insert terminal point
  if (input_trajectory_len - resampling_arclength.back() < motion_utils::overlap_threshold) {
    resampling_arclength.back() = input_trajectory_len;
  } else {
    resampling_arclength.push_back(input_trajectory_len);
  }

  // Insert stop point
  if (resample_input_trajectory_stop_point) {
    const auto distance_to_stop_point =
      motion_utils::calcDistanceToForwardStopPoint(input_trajectory.points, 0);
    if (distance_to_stop_point && !resampling_arclength.empty()) {
      for (size_t i = 1; i < resampling_arclength.size(); ++i) {
        if (
          resampling_arclength.at(i - 1) <= *distance_to_stop_point &&
          *distance_to_stop_point < resampling_arclength.at(i)) {
          const double dist_to_prev_point =
            std::fabs(*distance_to_stop_point - resampling_arclength.at(i - 1));
          const double dist_to_following_point =
            std::fabs(resampling_arclength.at(i) - *distance_to_stop_point);
          if (dist_to_prev_point < motion_utils::overlap_threshold) {
            resampling_arclength.at(i - 1) = *distance_to_stop_point;
          } else if (dist_to_following_point < motion_utils::overlap_threshold) {
            resampling_arclength.at(i) = *distance_to_stop_point;
          } else {
            resampling_arclength.insert(resampling_arclength.begin() + i, *distance_to_stop_point);
          }
          break;
        }
      }
    }
  }

  return resampleTrajectory(
    input_trajectory, resampling_arclength, use_lerp_for_xy, use_lerp_for_z,
    use_zero_order_hold_for_twist);
}

}  // namespace motion_utils
