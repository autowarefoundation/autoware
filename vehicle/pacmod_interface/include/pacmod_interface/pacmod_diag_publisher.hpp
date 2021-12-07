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

#ifndef PACMOD_INTERFACE__PACMOD_DIAG_PUBLISHER_HPP_
#define PACMOD_INTERFACE__PACMOD_DIAG_PUBLISHER_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/frame.hpp>
#include <pacmod3_msgs/msg/global_rpt.hpp>
#include <pacmod3_msgs/msg/steering_cmd.hpp>
#include <pacmod3_msgs/msg/system_cmd_float.hpp>
#include <pacmod3_msgs/msg/system_cmd_int.hpp>
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <pacmod3_msgs/msg/system_rpt_int.hpp>
#include <pacmod3_msgs/msg/wheel_speed_rpt.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>

class PacmodDiagPublisher : public rclcpp::Node
{
public:
  PacmodDiagPublisher();

private:
  using PacmodFeedbacksSyncPolicy = message_filters::sync_policies::ApproximateTime<
    pacmod3_msgs::msg::SystemRptFloat, pacmod3_msgs::msg::WheelSpeedRpt,
    pacmod3_msgs::msg::SystemRptFloat, pacmod3_msgs::msg::SystemRptFloat,
    pacmod3_msgs::msg::SystemRptInt, pacmod3_msgs::msg::SystemRptInt, pacmod3_msgs::msg::GlobalRpt>;

  /* subscribers */

  // From Pacmod
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>
    steer_wheel_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::WheelSpeedRpt>>
    wheel_speed_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>> accel_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>> brake_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>> shift_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>> turn_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::GlobalRpt>> global_rpt_sub_;
  std::unique_ptr<message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>> pacmod_feedbacks_sync_;

  // From CAN
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  /* ros parameters */
  double can_timeout_sec_;
  double pacmod3_msgs_timeout_sec_;

  /* variables */
  rclcpp::Time last_can_received_time_;
  rclcpp::Time last_pacmod3_msgs_received_time_;
  bool is_pacmod_rpt_received_ = false;
  pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt_ptr_;  // [rad]
  pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt_ptr_;   // [m/s]
  pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt_ptr_;
  pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt_ptr_;
  pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt_ptr_;
  pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt_ptr_;
  pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt_ptr_;

  // Diagnostic Updater
  std::shared_ptr<diagnostic_updater::Updater> updater_ptr_;

  /* callbacks */
  void callbackPacmodRpt(
    const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
    const pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
    const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
    const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
    const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt,
    const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt,
    const pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt);

  void callbackCan(const can_msgs::msg::Frame::ConstSharedPtr can);

  /* functions */
  void checkPacmodMsgs(diagnostic_updater::DiagnosticStatusWrapper & stat);
  std::string addMsg(const std::string & original_msg, const std::string & additional_msg);

  bool isTimeoutCanMsgs();
  bool isTimeoutPacmodMsgs();
  bool receivedPacmodMsgs();
  bool isBrakeActuatorAccident();
  bool isBrakeWireAccident();
  bool isAccelAccident();
  bool isOtherAccident();
};

#endif  // PACMOD_INTERFACE__PACMOD_DIAG_PUBLISHER_HPP_
