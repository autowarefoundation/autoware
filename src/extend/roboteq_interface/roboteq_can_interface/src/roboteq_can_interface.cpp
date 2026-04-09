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

#include "roboteq_can_interface/roboteq_can_interface.hpp"

#include "roboteq_can_interface/can_command_data.hpp"

RoboteqCanInterface::RoboteqCanInterface(const rclcpp::NodeOptions & node_options)
: Node("roboteq_can_interface", node_options), status_()
{
  using std::placeholders::_1;

  node_id_ = declare_parameter<int>("node_id", 1);

  // Subscriber
  can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
    "/from_can_bus", 10, std::bind(&RoboteqCanInterface::onCanFrame, this, _1));
  command_sub_ = this->create_subscription<roboteq_msgs::msg::RoboteqCommandStamped>(
    "~/input/command", 10, std::bind(&RoboteqCanInterface::onRoboteqCommand, this, _1));

  // Publisher
  can_pub_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);
  status_pub_ =
    this->create_publisher<roboteq_msgs::msg::RoboteqStatusStamped>("~/output/status", 10);
}

void RoboteqCanInterface::onCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  const uint16_t tpdo1_id = 0x180 + node_id_;
  const uint16_t tpdo2_id = 0x280 + node_id_;
  const uint16_t tpdo3_id = 0x380 + node_id_;
  const uint16_t tpdo4_id = 0x480 + node_id_;

  if (msg->id == tpdo1_id) {  // TPDO1: Motor RPM, Counter value
    status_.stamp = msg->header.stamp;
    status_.status.motor_speed_rpm = (static_cast<int32_t>(
      (msg->data.at(3) << 24) + (msg->data.at(2) << 16) + (msg->data.at(1) << 8) +
      msg->data.at(0)));
    status_.status.bl_counter_abs = (static_cast<int32_t>(
      (msg->data.at(7) << 24) + (msg->data.at(6) << 16) + (msg->data.at(5) << 8) +
      msg->data.at(4)));
    status_pub_->publish(status_);
  } else if (msg->id == tpdo2_id) {  // TPDO2: Fault Flags, Runtime Status Flags
    status_.stamp = msg->header.stamp;
    status_.status.fault.status = (static_cast<int32_t>(
      (msg->data.at(3) << 24) + (msg->data.at(2) << 16) + (msg->data.at(1) << 8) +
      msg->data.at(0)));
    status_.status.runtime.status = (static_cast<int32_t>(
      (msg->data.at(7) << 24) + (msg->data.at(6) << 16) + (msg->data.at(5) << 8) +
      msg->data.at(4)));
    status_pub_->publish(status_);
  } else if (msg->id == tpdo3_id) {  // TPDO3: Status Flags, Temperature
    status_.stamp = msg->header.stamp;
    status_.status.controller.status = (static_cast<int32_t>(
      (msg->data.at(3) << 24) + (msg->data.at(2) << 16) + (msg->data.at(1) << 8) +
      msg->data.at(0)));
    status_.status.ic_temperature = (static_cast<int32_t>(
      (msg->data.at(7) << 24) + (msg->data.at(6) << 16) + (msg->data.at(5) << 8) +
      msg->data.at(4)));
    status_pub_->publish(status_);
  } else if (msg->id == tpdo4_id) {  // TPDO4: Battery Voltage, Amperage
    status_.stamp = msg->header.stamp;
    status_.status.battery.battery_volts = (static_cast<int32_t>(
                                             (msg->data.at(3) << 24) + (msg->data.at(2) << 16) +
                                             (msg->data.at(1) << 8) + msg->data.at(0))) *
                                           0.1;
    status_.status.battery.amperage = (static_cast<int32_t>(
                                        (msg->data.at(7) << 24) + (msg->data.at(6) << 16) +
                                        (msg->data.at(5) << 8) + msg->data.at(4))) *
                                      0.1;
    status_pub_->publish(status_);
  }
}

void RoboteqCanInterface::onRoboteqCommand(
  const roboteq_msgs::msg::RoboteqCommandStamped::ConstSharedPtr msg)
{
  can_msgs::msg::Frame can_msg_cg;
  can_msg_cg.header.stamp = msg->stamp;
  can_msg_cg.is_rtr = false;
  can_msg_cg.is_extended = false;
  can_msg_cg.is_error = false;
  can_msg_cg.dlc = 8;

  // tba
  // send set accel command
  // send set decel command

  // send Set Motor Speed (S) command
  can_msg_cg.id = 0x600 + node_id_;
  can_msg_cg.data.at(0) = 0x23;
  can_msg_cg.data.at(1) = 0x02;
  can_msg_cg.data.at(2) = 0x20;
  can_msg_cg.data.at(3) = 0x01;
  can_msg_cg.data.at(4) = msg->command.speed & 0xFF;
  can_msg_cg.data.at(5) = msg->command.speed >> 8;
  can_msg_cg.data.at(6) = msg->command.speed >> 16;
  can_msg_cg.data.at(7) = msg->command.speed >> 24;

  can_pub_->publish(can_msg_cg);
}

void RoboteqCanInterface::publishEmergencyRelease()
{
  can_msgs::msg::Frame can_msg;
  // can_msg.header = msg->header;
  can_msg.is_rtr = false;
  can_msg.is_extended = false;
  can_msg.is_error = false;
  can_msg.dlc = 8;
  can_msg.id = 0x600 + node_id_;
  can_msg.data = can_cmd_.get_mg();
  can_pub_->publish(can_msg);
}

void RoboteqCanInterface::publishEmergencyStop()
{
  can_msgs::msg::Frame can_msg;
  // can_msg.header = msg->header;
  can_msg.is_rtr = false;
  can_msg.is_extended = false;
  can_msg.is_error = false;
  can_msg.dlc = 8;
  can_msg.id = 0x600 + node_id_;
  can_msg.data = can_cmd_.get_ex();
  can_pub_->publish(can_msg);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RoboteqCanInterface)
