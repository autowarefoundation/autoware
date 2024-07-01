// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__JOY_CONTROLLER__JOY_CONTROLLER_HPP_
#define AUTOWARE__JOY_CONTROLLER__JOY_CONTROLLER_HPP_

#include "autoware/joy_controller/joy_converter/joy_converter_base.hpp"

#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/msg/control_command_stamped.hpp>
#include <tier4_external_api_msgs/msg/gear_shift_stamped.hpp>
#include <tier4_external_api_msgs/msg/heartbeat.hpp>
#include <tier4_external_api_msgs/msg/turn_signal_stamped.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <tier4_external_api_msgs/srv/set_emergency.hpp>

#include <memory>
#include <string>

namespace autoware::joy_controller
{
using GearShiftType = tier4_external_api_msgs::msg::GearShift::_data_type;
using TurnSignalType = tier4_external_api_msgs::msg::TurnSignal::_data_type;
using GateModeType = tier4_control_msgs::msg::GateMode::_data_type;

class AutowareJoyControllerNode : public rclcpp::Node
{
public:
  explicit AutowareJoyControllerNode(const rclcpp::NodeOptions & node_options);

private:
  // Parameter
  std::string joy_type_;
  double update_rate_;
  double accel_ratio_;
  double brake_ratio_;
  double steer_ratio_;
  double steering_angle_velocity_;
  double accel_sensitivity_;
  double brake_sensitivity_;

  // ControlCommand Parameter
  bool raw_control_;
  double velocity_gain_;
  double max_forward_velocity_;
  double max_backward_velocity_;
  double backward_accel_ratio_;

  // CallbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_services_;

  // Subscriber
  autoware::universe_utils::InterProcessPollingSubscriber<sensor_msgs::msg::Joy> sub_joy_{
    this, "input/joy"};
  autoware::universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> sub_odom_{
    this, "input/odometry"};

  rclcpp::Time last_joy_received_time_;
  std::shared_ptr<const JoyConverterBase> joy_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;

  void onJoy();
  void onOdometry();

  // Publisher
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr pub_control_command_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::ControlCommandStamped>::SharedPtr
    pub_external_control_command_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::GearShiftStamped>::SharedPtr pub_shift_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::TurnSignalStamped>::SharedPtr pub_turn_signal_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::Heartbeat>::SharedPtr pub_heartbeat_;
  rclcpp::Publisher<tier4_control_msgs::msg::GateMode>::SharedPtr pub_gate_mode_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Engage>::SharedPtr pub_vehicle_engage_;

  void publishControlCommand();
  void publishExternalControlCommand();
  void publishShift();
  void publishTurnSignal();
  void publishGateMode();
  void publishHeartbeat();
  void publishAutowareEngage();
  void publishVehicleEngage();
  void sendEmergencyRequest(bool emergency);

  // Service Client
  rclcpp::Client<tier4_external_api_msgs::srv::SetEmergency>::SharedPtr client_emergency_stop_;
  rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr client_autoware_engage_;

  // Previous State
  autoware_control_msgs::msg::Control prev_control_command_;
  tier4_external_api_msgs::msg::ControlCommand prev_external_control_command_;
  GearShiftType prev_shift_ = tier4_external_api_msgs::msg::GearShift::NONE;
  GateModeType prev_gate_mode_ = tier4_control_msgs::msg::GateMode::AUTO;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void initTimer(double period_s);

  bool isDataReady();
  void onTimer();
};
}  // namespace autoware::joy_controller

#endif  // AUTOWARE__JOY_CONTROLLER__JOY_CONTROLLER_HPP_
