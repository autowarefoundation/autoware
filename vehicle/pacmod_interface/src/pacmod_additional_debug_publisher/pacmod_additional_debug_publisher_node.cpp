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

#include <pacmod_additional_debug_publisher/pacmod_additional_debug_publisher_node.hpp>

namespace
{
bool isTargetId(uint32_t id)
{
  return id == 0x32C || id == 0x451 || id == 0x452 || id == 0x453 || id == 0x454 || id == 0x455 ||
         id == 0x456 || id == 0x457 || id == 0x790 || id == 0x791 || id == 0x792;
}
}  // namespace

PacmodAdditionalDebugPublisherNode::PacmodAdditionalDebugPublisherNode()
: Node("pacmod_additional_debug_publisher")
{
  using std::placeholders::_1;

  debug_pub_ = create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>(
    "output/debug", rclcpp::QoS{1});
  sub_ = create_subscription<can_msgs::msg::Frame>(
    "input/can_tx", rclcpp::QoS{1},
    std::bind(&PacmodAdditionalDebugPublisherNode::canTxCallback, this, _1));
  debug_value_.data.resize(17);
  calibration_active_ = this->declare_parameter("calibration_active", false);
  if (calibration_active_) {
    accel_cal_rpt_pub_ =
      create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>("output/accel_cal_rpt", 1);
    brake_cal_rpt_pub_ =
      create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>("output/brake_cal_rpt", 1);
    steer_cal_rpt_pub_ =
      create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>("output/steer_cal_rpt", 1);
    accel_cal_rpt_.data.resize(3);
    brake_cal_rpt_.data.resize(5);
    steer_cal_rpt_.data.resize(3);
  }
}

void PacmodAdditionalDebugPublisherNode::canTxCallback(
  const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  if (isTargetId(msg->id)) {
    float debug1 = 0.0;
    float debug2 = 0.0;
    float debug3 = 0.0;
    float debug4 = 0.0;
    if (msg->id == 0x790) {
      int16_t temp = 0;
      temp = (static_cast<int16_t>(msg->data[0]) << 8) | msg->data[1];
      accel_cal_rpt_.data.at(0) = static_cast<double>(temp / 1000.0);  // accel_a_volt
      temp = (static_cast<int16_t>(msg->data[2]) << 8) | msg->data[3];
      accel_cal_rpt_.data.at(1) = static_cast<double>(temp / 1000.0);  // accel_b_volt
      temp = (static_cast<int16_t>(msg->data[4]) << 8) | msg->data[5];
      accel_cal_rpt_.data.at(2) = static_cast<double>(temp / 1000.0);  // output
    } else if (msg->id == 0x791) {
      int16_t temp = 0;
      int8_t temp1 = 0;
      temp = (static_cast<int16_t>(msg->data[0]) << 8) | msg->data[1];
      brake_cal_rpt_.data.at(0) = static_cast<double>(temp / 1000.0);  // sks1_volt
      temp = (static_cast<int16_t>(msg->data[2]) << 8) | msg->data[3];
      brake_cal_rpt_.data.at(1) = static_cast<double>(temp / 1000.0);  // sks2_volt
      temp1 = static_cast<int8_t>(msg->data[4]);
      brake_cal_rpt_.data.at(2) = static_cast<double>(temp1 / 100.0);  // pedal_position
      temp1 = static_cast<int8_t>(msg->data[5]);
      brake_cal_rpt_.data.at(3) = static_cast<double>(temp1 / 100.0);  // brake_cmd
      temp = (static_cast<int16_t>(msg->data[6]) << 8) | msg->data[7];
      brake_cal_rpt_.data.at(4) = static_cast<double>(temp / 1000.0);  // globe_position
    } else if (msg->id == 0x792) {
      int16_t temp = 0;
      temp = (static_cast<int16_t>(msg->data[0]) << 8) | msg->data[1];
      steer_cal_rpt_.data.at(0) = static_cast<double>(temp / 1000.0);  // trq1_volt
      temp = (static_cast<int16_t>(msg->data[2]) << 8) | msg->data[3];
      steer_cal_rpt_.data.at(1) = static_cast<double>(temp / 1000.0);  // trq2_volt
      temp = (static_cast<int16_t>(msg->data[4]) << 8) | msg->data[5];
      steer_cal_rpt_.data.at(2) = static_cast<double>(temp / 1000.0);  // position
    } else if (msg->id == 0x32C) {
      int16_t temp = 0;
      temp = (static_cast<int16_t>(msg->data[0]) << 8) | msg->data[1];
      debug1 = temp / 1000.0;
      temp = (static_cast<int16_t>(msg->data[2]) << 8) | msg->data[3];
      debug2 = temp / 100.0;
      temp = (static_cast<int16_t>(msg->data[4]) << 8) | msg->data[5];
      debug3 = temp / 1000.0;
      temp = (static_cast<int16_t>(msg->data[6]) << 8) | msg->data[7];
      debug4 = temp / 100.0;
    } else {
      union Data {
        uint32_t uint32_value;
        float float_value;
      } temp;
      temp.uint32_value = (static_cast<int32_t>(msg->data[3]) << 24) |
                          (static_cast<int32_t>(msg->data[2]) << 16) |
                          (static_cast<int32_t>(msg->data[1]) << 8) | msg->data[0];
      debug1 = temp.float_value;
      temp.uint32_value = (static_cast<int32_t>(msg->data[7]) << 24) |
                          (static_cast<int32_t>(msg->data[6]) << 16) |
                          (static_cast<int32_t>(msg->data[5]) << 8) | msg->data[4];
      debug2 = temp.float_value;
    }
    switch (msg->id) {
      case 0x32C:
        debug_value_.data.at(0) = debug1;  // steering pos
        debug_value_.data.at(1) = debug2;  // steering_eps_assist
        debug_value_.data.at(2) = debug3;  // steering rate
        debug_value_.data.at(3) = debug4;  // steering eps input
        break;
      case 0x451:
        debug_value_.data.at(4) = debug1;  // pid command
        debug_value_.data.at(5) = debug2;  // pid output
        break;
      case 0x452:
        debug_value_.data.at(6) = debug1;  // pid_error
        debug_value_.data.at(7) = debug2;  // pid_output
        break;
      case 0x453:
        debug_value_.data.at(8) = debug1;  // pid_p_term
        debug_value_.data.at(9) = debug2;  // pid_i_term
        break;
      case 0x454:
        debug_value_.data.at(10) = debug1;  // pid_d_term
        debug_value_.data.at(11) = debug2;  // pid_filtered_rate
        break;
      case 0x455:
        debug_value_.data.at(12) = debug1;  // lugre
        debug_value_.data.at(13) = debug2;  // rtz
        break;
      case 0x456:
        debug_value_.data.at(14) = debug1;  // lugre_rtz_filtered_rate
        debug_value_.data.at(15) = debug2;  // ctrl_dt
        break;
      case 0x457:
        debug_value_.data.at(16) = debug1;  // rpt_dt
        break;
      default:
        break;
    }
    debug_value_.stamp = this->now();
    debug_pub_->publish(debug_value_);
    if (calibration_active_) {
      accel_cal_rpt_.stamp = this->now();
      brake_cal_rpt_.stamp = this->now();
      steer_cal_rpt_.stamp = this->now();
      accel_cal_rpt_pub_->publish(accel_cal_rpt_);
      brake_cal_rpt_pub_->publish(brake_cal_rpt_);
      steer_cal_rpt_pub_->publish(steer_cal_rpt_);
    }
  }
}
