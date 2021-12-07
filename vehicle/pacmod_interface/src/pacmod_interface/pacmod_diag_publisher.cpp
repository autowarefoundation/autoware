// Copyright 2021 Tier IV, Inc.
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

#include <pacmod_interface/pacmod_diag_publisher.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>

PacmodDiagPublisher::PacmodDiagPublisher()
: Node("pacmod_diag_publisher"),
  last_can_received_time_(this->now()),
  last_pacmod3_msgs_received_time_(this->now())
{
  /* ros parameters */
  can_timeout_sec_ = declare_parameter("can_timeout_sec", 10.0);
  pacmod3_msgs_timeout_sec_ = declare_parameter("pacmod_msg_timeout_sec", 10.0);
  const double update_rate = declare_parameter("update_rate", 10.0);

  /* Diagnostic Updater */
  updater_ptr_ = std::make_shared<diagnostic_updater::Updater>(this, 1.0 / update_rate);
  updater_ptr_->setHardwareID("pacmod_checker");
  updater_ptr_->add("pacmod_checker", this, &PacmodDiagPublisher::checkPacmodMsgs);

  /* register subscribers */
  can_sub_ = create_subscription<can_msgs::msg::Frame>(
    "/pacmod/can_tx", 1, std::bind(&PacmodDiagPublisher::callbackCan, this, std::placeholders::_1));

  steer_wheel_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
      this, "/pacmod/steering_rpt");
  wheel_speed_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::WheelSpeedRpt>>(
      this, "/pacmod/wheel_speed_rpt");
  accel_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
    this, "/pacmod/accel_rpt");
  brake_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
    this, "/pacmod/brake_rpt");
  shift_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>>(
    this, "/pacmod/shift_rpt");
  turn_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>>(
    this, "/pacmod/turn_rpt");
  global_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::GlobalRpt>>(
    this, "/pacmod/global_rpt");

  pacmod_feedbacks_sync_ =
    std::make_unique<message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>>(
      PacmodFeedbacksSyncPolicy(10), *steer_wheel_rpt_sub_, *wheel_speed_rpt_sub_, *accel_rpt_sub_,
      *brake_rpt_sub_, *shift_rpt_sub_, *turn_rpt_sub_, *global_rpt_sub_);

  pacmod_feedbacks_sync_->registerCallback(std::bind(
    &PacmodDiagPublisher::callbackPacmodRpt, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
    std::placeholders::_7));
}

void PacmodDiagPublisher::callbackCan(
  [[maybe_unused]] const can_msgs::msg::Frame::ConstSharedPtr can)
{
  last_can_received_time_ = this->now();
}

void PacmodDiagPublisher::callbackPacmodRpt(
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
  const pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt,
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt,
  const pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt)
{
  last_pacmod3_msgs_received_time_ = this->now();
  steer_wheel_rpt_ptr_ = steer_wheel_rpt;
  wheel_speed_rpt_ptr_ = wheel_speed_rpt;
  accel_rpt_ptr_ = accel_rpt;
  brake_rpt_ptr_ = brake_rpt;
  shift_rpt_ptr_ = shift_rpt;
  global_rpt_ptr_ = global_rpt;
  turn_rpt_ptr_ = turn_rpt;
  is_pacmod_rpt_received_ = true;
}

void PacmodDiagPublisher::checkPacmodMsgs(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  int8_t level = DiagStatus::OK;
  std::string msg = "";

  if (isTimeoutCanMsgs()) {
    level = DiagStatus::ERROR;
    msg = addMsg(msg, "CAN reception timeout.");
  }

  if (isTimeoutPacmodMsgs()) {
    level = DiagStatus::ERROR;
    msg = addMsg(msg, "Pacmod msgs reception timeout.");
  }

  if (!receivedPacmodMsgs()) {
    if (level == DiagStatus::OK) {
      msg = "OK";
    }
    stat.summary(level, msg);
    // do not receive pacmod msgs yet.
    return;
  }

  if (isBrakeActuatorAccident()) {
    level = DiagStatus::ERROR;
    msg = addMsg(msg, "Brake actuator failure.");
  }

  if (isBrakeWireAccident()) {
    level = DiagStatus::ERROR;
    msg = addMsg(msg, "Brake wire failure.");
  }

  if (isAccelAccident()) {
    level = DiagStatus::ERROR;
    msg = addMsg(msg, "Accel module failure.");
  }

  if (level == DiagStatus::OK && isOtherAccident()) {
    level = DiagStatus::ERROR;
    msg = addMsg(msg, "Unknown Pacmod Error.");
  }

  if (level == DiagStatus::OK) {
    msg = "OK";
  }
  stat.summary(level, msg);
}

std::string PacmodDiagPublisher::addMsg(
  const std::string & original_msg, const std::string & additional_msg)
{
  if (original_msg == "") {
    return additional_msg;
  }

  return original_msg + " ; " + additional_msg;
}

bool PacmodDiagPublisher::isTimeoutCanMsgs()
{
  const double dt = (this->now() - last_can_received_time_).seconds();
  return dt > can_timeout_sec_;
}

bool PacmodDiagPublisher::isTimeoutPacmodMsgs()
{
  const double dt = (this->now() - last_pacmod3_msgs_received_time_).seconds();
  return dt > pacmod3_msgs_timeout_sec_;
}

bool PacmodDiagPublisher::receivedPacmodMsgs() { return is_pacmod_rpt_received_; }

bool PacmodDiagPublisher::isBrakeActuatorAccident()
{
  return global_rpt_ptr_->pacmod_sys_fault_active && brake_rpt_ptr_->pacmod_fault;
}

bool PacmodDiagPublisher::isBrakeWireAccident()
{
  return global_rpt_ptr_->pacmod_sys_fault_active && brake_rpt_ptr_->command_output_fault;
}

bool PacmodDiagPublisher::isAccelAccident()
{
  return global_rpt_ptr_->pacmod_sys_fault_active && accel_rpt_ptr_->input_output_fault;
}

bool PacmodDiagPublisher::isOtherAccident() { return global_rpt_ptr_->pacmod_sys_fault_active; }
