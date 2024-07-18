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

#include "autoware/control_validator/control_validator.hpp"

#include "autoware/control_validator/utils.hpp"
#include "autoware/motion_utils/trajectory/interpolation.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <memory>
#include <string>
#include <utility>

namespace autoware::control_validator
{
using diagnostic_msgs::msg::DiagnosticStatus;

ControlValidator::ControlValidator(const rclcpp::NodeOptions & options)
: Node("control_validator", options)
{
  using std::placeholders::_1;
  sub_predicted_traj_ = create_subscription<Trajectory>(
    "~/input/predicted_trajectory", 1,
    std::bind(&ControlValidator::onPredictedTrajectory, this, _1));

  pub_status_ = create_publisher<ControlValidatorStatus>("~/output/validation_status", 1);
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", 1);

  debug_pose_publisher_ = std::make_shared<ControlValidatorDebugMarkerPublisher>(this);

  setupParameters();

  setupDiag();
}

void ControlValidator::setupParameters()
{
  diag_error_count_threshold_ = declare_parameter<int>("diag_error_count_threshold");
  display_on_terminal_ = declare_parameter<bool>("display_on_terminal");

  {
    auto & p = validation_params_;
    const std::string t = "thresholds.";
    p.max_distance_deviation_threshold = declare_parameter<double>(t + "max_distance_deviation");
    p.max_reverse_velocity_threshold = declare_parameter<double>(t + "reverse_velocity");
    p.max_over_velocity_ratio_threshold = declare_parameter<double>(t + "over_velocity_ratio");
  }

  try {
    vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "failed to get vehicle info. use default value.");
    vehicle_info_.front_overhang_m = 0.5;
    vehicle_info_.wheel_base_m = 4.0;
  }
}

void ControlValidator::setStatus(
  DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg)
{
  if (is_ok) {
    stat.summary(DiagnosticStatus::OK, "validated.");
  } else if (validation_status_.invalid_count < diag_error_count_threshold_) {
    const auto warn_msg = msg + " (invalid count is less than error threshold: " +
                          std::to_string(validation_status_.invalid_count) + " < " +
                          std::to_string(diag_error_count_threshold_) + ")";
    stat.summary(DiagnosticStatus::WARN, warn_msg);
  } else {
    stat.summary(DiagnosticStatus::ERROR, msg);
  }
}

void ControlValidator::setupDiag()
{
  auto & d = diag_updater_;
  d.setHardwareID("control_validator");

  std::string ns = "control_validation_";
  d.add(ns + "max_distance_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_max_distance_deviation,
      "control output is deviated from trajectory");
  });
  d.add(ns + "velocity_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_velocity_deviation,
      "current velocity is deviated from the desired velocity");
  });
}

bool ControlValidator::isDataReady()
{
  const auto waiting = [this](const auto s) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", s);
    return false;
  };

  if (!current_kinematics_) {
    return waiting("current_kinematics_");
  }
  if (!current_reference_trajectory_) {
    return waiting("current_reference_trajectory_");
  }
  if (!current_predicted_trajectory_) {
    return waiting("current_predicted_trajectory_");
  }
  return true;
}

void ControlValidator::onPredictedTrajectory(const Trajectory::ConstSharedPtr msg)
{
  current_predicted_trajectory_ = msg;
  current_reference_trajectory_ = sub_reference_traj_.takeData();
  current_kinematics_ = sub_kinematics_.takeData();

  if (!isDataReady()) return;

  debug_pose_publisher_->clearMarkers();

  validate(*current_predicted_trajectory_, *current_reference_trajectory_, *current_kinematics_);

  diag_updater_.force_update();

  // for debug
  publishDebugInfo();
  displayStatus();
}

void ControlValidator::publishDebugInfo()
{
  validation_status_.stamp = get_clock()->now();
  pub_status_->publish(validation_status_);

  if (!isAllValid(validation_status_)) {
    geometry_msgs::msg::Pose front_pose = current_kinematics_->pose.pose;
    shiftPose(front_pose, vehicle_info_.front_overhang_m + vehicle_info_.wheel_base_m);
    debug_pose_publisher_->pushVirtualWall(front_pose);
    debug_pose_publisher_->pushWarningMsg(front_pose, "INVALID CONTROL");
  }
  debug_pose_publisher_->publish();
}

void ControlValidator::validate(
  const Trajectory & predicted_trajectory, const Trajectory & reference_trajectory,
  const Odometry & kinematics)
{
  if (predicted_trajectory.points.size() < 2) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "predicted_trajectory size is less than 2. Cannot validate.");
    return;
  }

  auto & s = validation_status_;

  s.is_valid_max_distance_deviation = checkValidMaxDistanceDeviation(predicted_trajectory);
  s.is_valid_velocity_deviation = checkValidVelocityDeviation(reference_trajectory, kinematics);

  s.invalid_count = isAllValid(s) ? 0 : s.invalid_count + 1;
}

bool ControlValidator::checkValidMaxDistanceDeviation(const Trajectory & predicted_trajectory)
{
  validation_status_.max_distance_deviation =
    calcMaxLateralDistance(*current_reference_trajectory_, predicted_trajectory);
  if (
    validation_status_.max_distance_deviation >
    validation_params_.max_distance_deviation_threshold) {
    return false;
  }
  return true;
}

bool ControlValidator::checkValidVelocityDeviation(
  const Trajectory & reference_trajectory, const Odometry & kinematics)
{
  const double current_vel = kinematics.twist.twist.linear.x;
  if (reference_trajectory.points.size() < 2) return true;
  const double desired_vel =
    autoware::motion_utils::calcInterpolatedPoint(reference_trajectory, kinematics.pose.pose)
      .longitudinal_velocity_mps;

  validation_status_.current_velocity = current_vel;
  validation_status_.desired_velocity = desired_vel;

  const bool is_over_velocity =
    std::abs(current_vel) >
    std::abs(desired_vel) * (1.0 + validation_params_.max_over_velocity_ratio_threshold) +
      validation_params_.max_reverse_velocity_threshold;
  const bool is_reverse_velocity =
    std::signbit(current_vel * desired_vel) &&
    std::abs(current_vel) > validation_params_.max_reverse_velocity_threshold;

  return !(is_over_velocity || is_reverse_velocity);
}

bool ControlValidator::isAllValid(const ControlValidatorStatus & s)
{
  return s.is_valid_max_distance_deviation && s.is_valid_velocity_deviation;
}

void ControlValidator::displayStatus()
{
  if (!display_on_terminal_) return;
  rclcpp::Clock clock{RCL_ROS_TIME};

  const auto warn = [this, &clock](const bool status, const std::string & msg) {
    if (!status) {
      RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "%s", msg.c_str());
    }
  };

  const auto & s = validation_status_;

  warn(
    s.is_valid_max_distance_deviation,
    "predicted trajectory is too far from planning trajectory!!");
}

}  // namespace autoware::control_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control_validator::ControlValidator)
