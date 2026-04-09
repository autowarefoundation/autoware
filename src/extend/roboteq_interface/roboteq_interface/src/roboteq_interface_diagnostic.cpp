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

#include "roboteq_interface/roboteq_interface.hpp"

#include <string>

void RoboteqInterface::setupDiagnosticUpdater()
{
  diagnostic_updater_.setHardwareID("Roboteq SBL1360A");
  diagnostic_updater_.add("control_command_timeout", this, &RoboteqInterface::checkControlCommand);
  diagnostic_updater_.add(
    "left_roboteq_connection", this, &RoboteqInterface::checkLeftRoboteqConnection);
  diagnostic_updater_.add(
    "right_roboteq_connection", this, &RoboteqInterface::checkRightRoboteqConnection);
  diagnostic_updater_.add("left_roboteq_errors", this, &RoboteqInterface::checkLeftRoboteqErrors);
  diagnostic_updater_.add("right_roboteq_errors", this, &RoboteqInterface::checkRightRoboteqErrors);
}

void RoboteqInterface::checkControlCommand(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  const auto dt = (this->now() - prev_control_cmd_stamp_).seconds();
  is_control_command_timeout_ = dt > control_cmd_timeout_sec_;

  if (is_control_command_timeout_) {
    std::string error_msg = "control_cmd msg is timeout";
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "%s", error_msg.c_str());
    stat.summary(DiagnosticStatus::ERROR, error_msg);
    return;
  }

  stat.addf("control_cmd msg received delta time", "%lf", dt);
  stat.summary(DiagnosticStatus::OK, "OK");
}

void RoboteqInterface::checkLeftRoboteqConnection(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  int level = DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!left_status_ptr_) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000 /* ms */,
      "Left roboteq status msg not received");
    stat.summary(DiagnosticStatus::ERROR, "Left roboteq status msg not received");
    return;
  }

  // roboteq status msg timeout
  double dt = (this->now() - left_status_ptr_->stamp).seconds();
  if (dt > roboteq_status_timeout_sec_) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000 /* ms */, "Left roboteq status msg timed out");
    level = DiagnosticStatus::ERROR;
    msg = "Left roboteq status msg timeout";
  }

  stat.addf("Left roboteq status msg received delta time", "%lf", dt);
  stat.summary(level, msg);
}

void RoboteqInterface::checkRightRoboteqConnection(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  int level = DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!right_status_ptr_) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000 /* ms */,
      "Right roboteq status msg not received");
    stat.summary(DiagnosticStatus::ERROR, "Right roboteq status msg not received");
    return;
  }

  // roboteq status msg timeout
  double dt = (this->now() - right_status_ptr_->stamp).seconds();
  if (dt > roboteq_status_timeout_sec_) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000 /* ms */, "Right roboteq status msg timed out");
    level = DiagnosticStatus::ERROR;
    msg = "Right roboteq status msg timeout";
  }

  stat.addf("Right roboteq status msg received delta time", "%lf", dt);
  stat.summary(level, msg);
}

void RoboteqInterface::checkLeftRoboteqErrors(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using roboteq_msgs::msg::ControllerStatus;

  int level = DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!left_status_ptr_) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000 /* ms */,
      "Left roboteq status msg not received");
    stat.summary(DiagnosticStatus::ERROR, "Left roboteq status msg not received");
    return;
  }

  // fault status
  for (uint16_t i = 1; i <= 128; i <<= 1) {
    const std::string key = "fault_status_l-" + std::to_string(i);
    const bool value = left_status_ptr_->status.fault.status & i;

    stat.add(key, value);

    if (value) {
      level = DiagnosticStatus::ERROR;
      msg = "Left roboteq fault status error";
    }
  }

  stat.summary(level, msg);
}

void RoboteqInterface::checkRightRoboteqErrors(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using roboteq_msgs::msg::ControllerStatus;

  int level = DiagnosticStatus::OK;
  std::string msg = "OK";

  if (!right_status_ptr_) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000 /* ms */,
      "Right roboteq status msg not received");
    stat.summary(DiagnosticStatus::ERROR, "Right roboteq status msg not received");
    return;
  }

  // fault status
  for (uint16_t i = 1; i <= 128; i <<= 1) {
    const std::string key = "fault_status_r-" + std::to_string(i);
    const bool value = right_status_ptr_->status.fault.status & i;

    stat.add(key, value);

    if (value) {
      level = DiagnosticStatus::ERROR;
      msg = "Right roboteq fault status error";
    }
  }

  stat.summary(level, msg);
}
