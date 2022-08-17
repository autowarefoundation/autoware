//
//  Copyright 2022 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef TOOLS__MANUAL_CONTROLLER_HPP_
#define TOOLS__MANUAL_CONTROLLER_HPP_

#include <QDial>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>

#include <memory>

namespace rviz_plugins
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::Twist;
using tier4_control_msgs::msg::GateMode;
using EngageSrv = tier4_external_api_msgs::srv::Engage;
using autoware_auto_vehicle_msgs::msg::Engage;
using autoware_auto_vehicle_msgs::msg::GearReport;

class ManualController : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ManualController(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:  // NOLINT for Qt
  void onClickCruiseVelocity();
  void onClickEnableButton();
  void onManualSteering();
  void update();

protected:
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();
  void onPublishControlCommand();
  void onGateMode(const GateMode::ConstSharedPtr msg);
  void onVelocity(const VelocityReport::ConstSharedPtr msg);
  void onEngageStatus(const Engage::ConstSharedPtr msg);
  void onGear(const GearReport::ConstSharedPtr msg);
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<GateMode>::SharedPtr sub_gate_mode_;
  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_;
  rclcpp::Subscription<Engage>::SharedPtr sub_engage_;
  rclcpp::Publisher<tier4_control_msgs::msg::GateMode>::SharedPtr pub_gate_mode_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_control_command_;
  rclcpp::Publisher<GearCommand>::SharedPtr pub_gear_cmd_;
  rclcpp::Client<EngageSrv>::SharedPtr client_engage_;
  rclcpp::Subscription<GearReport>::SharedPtr sub_gear_;

  double cruise_velocity_{0.0};
  double steering_angle_{0.0};
  double current_velocity_{0.0};
  rclcpp::Time prev_stamp_;
  std::unique_ptr<double> previous_velocity_;
  std::unique_ptr<double> current_acceleration_;

  QLabel * gate_mode_label_ptr_;
  QLabel * gear_label_ptr_;
  QLabel * engage_status_label_ptr_;
  QPushButton * enable_button_ptr_;
  QPushButton * cruise_velocity_button_ptr_;
  QSpinBox * cruise_velocity_input_;
  QDial * steering_slider_ptr_;
  QLabel * steering_angle_ptr_;

  bool current_engage_{false};
};

}  // namespace rviz_plugins

#endif  // TOOLS__MANUAL_CONTROLLER_HPP_
