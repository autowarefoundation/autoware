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

#ifndef ROBOTEQ_SERIAL_INTERFACE__ROBOTEQ_SERIAL_INTERFACE_HPP_
#define ROBOTEQ_SERIAL_INTERFACE__ROBOTEQ_SERIAL_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "roboteq_msgs/msg/roboteq_command_stamped.hpp"
#include "roboteq_msgs/msg/roboteq_status_stamped.hpp"

#include <boost/asio.hpp>

#include <memory>
#include <string>
#include <vector>

class RoboteqSerialInterface : public rclcpp::Node
{
public:
  explicit RoboteqSerialInterface(const rclcpp::NodeOptions & node_options);

private:
  using BatteryStatus = roboteq_msgs::msg::BatteryStatus;
  using FaultStatus = roboteq_msgs::msg::FaultStatus;
  using RoboteqStatus = roboteq_msgs::msg::RoboteqStatus;

  // serial
  boost::asio::io_service io_;
  boost::asio::serial_port serial_;
  std::string devname_;

  // variables
  std::unique_ptr<BatteryStatus::_battery_volts_type> battery_volts_ptr_{nullptr};
  std::unique_ptr<FaultStatus::_status_type> fault_flag_ptr_{nullptr};

  std::unique_ptr<RoboteqStatus::_motor_speed_rpm_type> rpm_ptr_{nullptr};

  std::unique_ptr<RoboteqStatus::_analog_input_1_type> analog_input_1_ptr_{nullptr};
  std::unique_ptr<RoboteqStatus::_digital_input_type> digital_input_ptr_{nullptr};
  std::unique_ptr<RoboteqStatus::_digital_output_type> digital_output_ptr_{nullptr};

  void openSerial();
  void closeSerial();
  void readSerial();

  std::vector<std::string> split(const std::string & input, char delimiter);
  void receiveCallback(const std::string & msg);
  void onCB(const std::string & msg);
  void onBS(const std::string & msg);
  void onFF(const std::string & msg);
  void onV(const std::string & msg);
  void onBA(const std::string & msg);
  void onD(const std::string & msg);
  void onDO(const std::string & msg);
  void onAI(const std::string & msg);

  // Callback
  void onRoboteqCommand(const roboteq_msgs::msg::RoboteqCommandStamped::ConstSharedPtr msg);
  void onDOutCommand(const roboteq_msgs::msg::DigitalOutCommand & msg);

  // Publish
  void publishRoboteqStatus();

  // Subscriber
  rclcpp::Subscription<roboteq_msgs::msg::RoboteqCommandStamped>::SharedPtr command_sub_;

  // Publisher
  rclcpp::Publisher<roboteq_msgs::msg::RoboteqStatusStamped>::SharedPtr status_pub_;
};

#endif  // ROBOTEQ_SERIAL_INTERFACE__ROBOTEQ_SERIAL_INTERFACE_HPP_
