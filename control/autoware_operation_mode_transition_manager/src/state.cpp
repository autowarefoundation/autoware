// Copyright 2022 Autoware Foundation
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

#include "state.hpp"

#include "util.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/geometry/pose_deviation.hpp>

#include <algorithm>
#include <cmath>

namespace autoware::operation_mode_transition_manager
{

using autoware::motion_utils::findNearestIndex;
using autoware::universe_utils::calcDistance2d;
using autoware::universe_utils::calcYawDeviation;

AutonomousMode::AutonomousMode(rclcpp::Node * node)
: logger_(node->get_logger()), clock_(node->get_clock())
{
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo();

  sub_control_cmd_ = node->create_subscription<Control>(
    "control_cmd", 1, [this](const Control::SharedPtr msg) { control_cmd_ = *msg; });
  sub_trajectory_follower_control_cmd_ = node->create_subscription<Control>(
    "trajectory_follower_control_cmd", 1,
    [this](const Control::SharedPtr msg) { trajectory_follower_control_cmd_ = *msg; });

  sub_kinematics_ = node->create_subscription<Odometry>(
    "kinematics", 1, [this](const Odometry::SharedPtr msg) { kinematics_ = *msg; });

  sub_trajectory_ = node->create_subscription<Trajectory>(
    "trajectory", 1, [this](const Trajectory::SharedPtr msg) { trajectory_ = *msg; });

  check_engage_condition_ = node->declare_parameter<bool>("check_engage_condition");
  enable_engage_on_driving_ = node->declare_parameter<bool>("enable_engage_on_driving");
  nearest_dist_deviation_threshold_ =
    node->declare_parameter<double>("nearest_dist_deviation_threshold");
  nearest_yaw_deviation_threshold_ =
    node->declare_parameter<double>("nearest_yaw_deviation_threshold");

  // params for mode change available
  {
    auto & p = engage_acceptable_param_;
    p.allow_autonomous_in_stopped =
      node->declare_parameter<bool>("engage_acceptable_limits.allow_autonomous_in_stopped");
    p.dist_threshold = node->declare_parameter<double>("engage_acceptable_limits.dist_threshold");
    p.speed_upper_threshold =
      node->declare_parameter<double>("engage_acceptable_limits.speed_upper_threshold");
    p.speed_lower_threshold =
      node->declare_parameter<double>("engage_acceptable_limits.speed_lower_threshold");
    p.yaw_threshold = node->declare_parameter<double>("engage_acceptable_limits.yaw_threshold");
    p.acc_threshold = node->declare_parameter<double>("engage_acceptable_limits.acc_threshold");
    p.lateral_acc_threshold =
      node->declare_parameter<double>("engage_acceptable_limits.lateral_acc_threshold");
    p.lateral_acc_diff_threshold =
      node->declare_parameter<double>("engage_acceptable_limits.lateral_acc_diff_threshold");
  }

  // params for mode change completed
  {
    auto & p = stable_check_param_;
    p.duration = node->get_parameter("stable_check.duration").as_double();
    p.dist_threshold = node->declare_parameter<double>("stable_check.dist_threshold");
    p.speed_upper_threshold = node->declare_parameter<double>("stable_check.speed_upper_threshold");
    p.speed_lower_threshold = node->declare_parameter<double>("stable_check.speed_lower_threshold");
    p.yaw_threshold = node->declare_parameter<double>("stable_check.yaw_threshold");
  }
}

void AutonomousMode::update(bool transition)
{
  if (!transition) {
    stable_start_time_.reset();
  }
}

bool AutonomousMode::isModeChangeCompleted()
{
  if (!check_engage_condition_) {
    return true;
  }

  const auto current_speed = kinematics_.twist.twist.linear.x;
  const auto & param = engage_acceptable_param_;

  // Engagement completes quickly if the vehicle is stopped.
  if (param.allow_autonomous_in_stopped && std::abs(current_speed) < 0.01) {
    return true;
  }

  const auto unstable = [this]() {
    stable_start_time_.reset();
    return false;
  };

  if (trajectory_.points.size() < 2) {
    RCLCPP_INFO(logger_, "Not stable yet: trajectory size must be > 2");
    return unstable();
  }

  const auto closest_idx = findNearestIndex(
    trajectory_.points, kinematics_.pose.pose, nearest_dist_deviation_threshold_,
    nearest_yaw_deviation_threshold_);
  if (!closest_idx) {
    RCLCPP_INFO(logger_, "Not stable yet: closest point not found");
    return unstable();
  }

  const auto closest_point = trajectory_.points.at(*closest_idx);

  // check for lateral deviation
  const auto dist_deviation =
    autoware::motion_utils::calcLateralOffset(trajectory_.points, kinematics_.pose.pose.position);
  if (std::isnan(dist_deviation)) {
    RCLCPP_INFO(logger_, "Not stable yet: lateral offset calculation failed.");
    return unstable();
  }
  if (dist_deviation > stable_check_param_.dist_threshold) {
    RCLCPP_INFO(logger_, "Not stable yet: distance deviation is too large: %f", dist_deviation);
    return unstable();
  }

  // check for yaw deviation
  const auto yaw_deviation =
    autoware::motion_utils::calcYawDeviation(trajectory_.points, kinematics_.pose.pose);
  if (std::isnan(yaw_deviation)) {
    RCLCPP_INFO(logger_, "Not stable yet: lateral offset calculation failed.");
    return unstable();
  }
  if (yaw_deviation > stable_check_param_.yaw_threshold) {
    RCLCPP_INFO(logger_, "Not stable yet: yaw deviation is too large: %f", yaw_deviation);
    return unstable();
  }

  // check for speed deviation
  const auto speed_deviation =
    kinematics_.twist.twist.linear.x - closest_point.longitudinal_velocity_mps;
  if (speed_deviation > stable_check_param_.speed_upper_threshold) {
    RCLCPP_INFO(logger_, "Not stable yet: ego speed is too high: %f", speed_deviation);
    return unstable();
  }
  if (speed_deviation < stable_check_param_.speed_lower_threshold) {
    RCLCPP_INFO(logger_, "Not stable yet: ego speed is too low: %f", speed_deviation);
    return unstable();
  }

  // count start.
  if (!stable_start_time_) {
    stable_start_time_ = std::make_unique<rclcpp::Time>(clock_->now());
  }

  // keep being stable for enough time.
  const double stable_time = (clock_->now() - *stable_start_time_).seconds();
  const bool is_system_stable = stable_time > stable_check_param_.duration;
  RCLCPP_INFO(logger_, "Now stable: now duration: %f", stable_time);
  return is_system_stable;
}

bool AutonomousMode::hasDangerAcceleration()
{
  debug_info_.target_control_acceleration = control_cmd_.longitudinal.acceleration;

  const bool is_stopping = std::abs(kinematics_.twist.twist.linear.x) < 0.01;
  if (is_stopping) {
    return false;  // any acceleration is ok when stopped
  }

  const bool has_large_acc =
    std::abs(control_cmd_.longitudinal.acceleration) > engage_acceptable_param_.acc_threshold;
  return has_large_acc;
}

std::pair<bool, bool> AutonomousMode::hasDangerLateralAcceleration()
{
  const auto wheelbase = vehicle_info_.wheel_base_m;
  const auto curr_vx = kinematics_.twist.twist.linear.x;
  const auto curr_wz = kinematics_.twist.twist.angular.z;

  // Calculate angular velocity from kinematics model.
  // Use current_vx to focus on the steering behavior.
  const auto target_wz = curr_vx * std::tan(control_cmd_.lateral.steering_tire_angle) / wheelbase;

  const auto curr_lat_acc = curr_vx * curr_wz;
  const auto target_lat_acc = curr_vx * target_wz;

  const bool has_large_lat_acc =
    std::abs(curr_lat_acc) > engage_acceptable_param_.lateral_acc_threshold;
  const bool has_large_lat_acc_diff =
    std::abs(curr_lat_acc - target_lat_acc) > engage_acceptable_param_.lateral_acc_diff_threshold;

  debug_info_.lateral_acceleration = curr_lat_acc;
  debug_info_.lateral_acceleration_deviation = curr_lat_acc - target_lat_acc;

  return {has_large_lat_acc, has_large_lat_acc_diff};
}

bool AutonomousMode::isModeChangeAvailable()
{
  if (!check_engage_condition_) {
    setAllOk(debug_info_);
    return true;
  }

  const auto current_speed = kinematics_.twist.twist.linear.x;
  const auto target_control_speed = control_cmd_.longitudinal.velocity;
  const auto & param = engage_acceptable_param_;

  if (!enable_engage_on_driving_ && std::fabs(current_speed) > 1.0e-2) {
    RCLCPP_INFO(
      logger_,
      "Engage unavailable: enable_engage_on_driving is false, and the vehicle is not "
      "stationary.");
    debug_info_ = DebugInfo{};  // all false
    return false;
  }

  if (trajectory_.points.size() < 2) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, *clock_, 5000, "Engage unavailable: trajectory size must be > 2");
    debug_info_ = DebugInfo{};  // all false
    return false;
  }

  const auto closest_idx = findNearestIndex(
    trajectory_.points, kinematics_.pose.pose, nearest_dist_deviation_threshold_,
    nearest_yaw_deviation_threshold_);
  if (!closest_idx) {
    RCLCPP_INFO(logger_, "Engage unavailable: closest point not found");
    debug_info_ = DebugInfo{};  // all false
    return false;               // closest trajectory point not found.
  }
  const auto closest_point = trajectory_.points.at(*closest_idx);
  const auto target_planning_speed = closest_point.longitudinal_velocity_mps;
  debug_info_.trajectory_available_ok = true;

  // No engagement is lateral control error is large
  const auto lateral_deviation = calcDistance2d(closest_point.pose, kinematics_.pose.pose);
  const bool lateral_deviation_ok = lateral_deviation < param.dist_threshold;

  // No engagement is yaw control error is large
  const auto yaw_deviation = calcYawDeviation(closest_point.pose, kinematics_.pose.pose);
  const bool yaw_deviation_ok = yaw_deviation < param.yaw_threshold;

  // No engagement if speed control error is large
  const auto speed_deviation = current_speed - target_planning_speed;
  const bool speed_upper_deviation_ok = speed_deviation <= param.speed_upper_threshold;
  const bool speed_lower_deviation_ok = speed_deviation >= param.speed_lower_threshold;

  // No engagement if the vehicle is moving but the target speed is zero.
  const bool is_stop_cmd_indicated =
    std::abs(target_control_speed) < 0.01 ||
    std::abs(trajectory_follower_control_cmd_.longitudinal.velocity) < 0.01;
  const bool stop_ok = !(std::abs(current_speed) > 0.1 && is_stop_cmd_indicated);

  // No engagement if the large acceleration is commanded.
  const bool large_acceleration_ok = !hasDangerAcceleration();

  // No engagement if the lateral acceleration is over threshold
  const auto [has_large_lat_acc, has_large_lat_acc_diff] = hasDangerLateralAcceleration();
  const auto large_lateral_acceleration_ok = !has_large_lat_acc;
  const auto large_lateral_acceleration_diff_ok = !has_large_lat_acc_diff;

  // No engagement if a stop is expected within a certain period of time
  // TODO(Horibe): write me
  // ...

  const bool is_all_ok = lateral_deviation_ok && yaw_deviation_ok && speed_upper_deviation_ok &&
                         speed_lower_deviation_ok && stop_ok && large_acceleration_ok &&
                         large_lateral_acceleration_ok && large_lateral_acceleration_diff_ok;

  // set for debug info
  {
    debug_info_.is_all_ok = is_all_ok;
    debug_info_.lateral_deviation_ok = lateral_deviation_ok;
    debug_info_.yaw_deviation_ok = yaw_deviation_ok;
    debug_info_.speed_upper_deviation_ok = speed_upper_deviation_ok;
    debug_info_.speed_lower_deviation_ok = speed_lower_deviation_ok;
    debug_info_.stop_ok = stop_ok;
    debug_info_.large_acceleration_ok = large_acceleration_ok;
    debug_info_.large_lateral_acceleration_ok = large_lateral_acceleration_ok;
    debug_info_.large_lateral_acceleration_diff_ok = large_lateral_acceleration_diff_ok;

    debug_info_.current_speed = current_speed;
    debug_info_.target_control_speed = target_control_speed;
    debug_info_.target_planning_speed = target_planning_speed;

    debug_info_.lateral_deviation = lateral_deviation;
    debug_info_.yaw_deviation = yaw_deviation;
    debug_info_.speed_deviation = speed_deviation;
  }

  // Engagement is ready if the vehicle is stopped.
  // (this is checked in the end to calculate some debug values.)
  if (param.allow_autonomous_in_stopped && std::abs(current_speed) < 0.01) {
    debug_info_.is_all_ok = true;
    debug_info_.engage_allowed_for_stopped_vehicle = true;
    return true;
  }

  return is_all_ok;
}

}  // namespace autoware::operation_mode_transition_manager
