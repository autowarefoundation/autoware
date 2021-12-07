// Copyright 2021 The Autoware Foundation
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

#include <functional>
#include <memory>

#include "trajectory_follower_nodes/latlon_muxer_node.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower_nodes
{

LatLonMuxer::LatLonMuxer(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("latlon_muxer", node_options)
{
  m_control_cmd_pub =
    create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "~/output/control_cmd",
    rclcpp::QoS{1}.transient_local());
  m_lat_control_cmd_sub =
    create_subscription<autoware_auto_control_msgs::msg::AckermannLateralCommand>(
    "~/input/lateral/control_cmd", rclcpp::QoS{1},
    std::bind(&LatLonMuxer::latCtrlCmdCallback, this, std::placeholders::_1));
  m_lon_control_cmd_sub =
    create_subscription<autoware_auto_control_msgs::msg::LongitudinalCommand>(
    "~/input/longitudinal/control_cmd", rclcpp::QoS{1},
    std::bind(&LatLonMuxer::lonCtrlCmdCallback, this, std::placeholders::_1));
  m_timeout_thr_sec = declare_parameter<double>("timeout_thr_sec");
}

bool LatLonMuxer::checkTimeout()
{
  const auto now = this->now();
  if ((now - m_lat_cmd->stamp).seconds() > m_timeout_thr_sec) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(), 1000 /*ms*/,
      "Lateral control command too old, muxed command will not be published.");
    return false;
  }
  if ((now - m_lon_cmd->stamp).seconds() > m_timeout_thr_sec) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(), 1000 /*ms*/,
      "Longitudinal control command too old, muxed command will not be published.");
    return false;
  }
  return true;
}

void LatLonMuxer::publishCmd()
{
  if (!m_lat_cmd || !m_lon_cmd) {
    return;
  }
  if (!checkTimeout()) {
    return;
  }

  autoware_auto_control_msgs::msg::AckermannControlCommand out;
  out.stamp = this->now();
  out.lateral = *m_lat_cmd;
  out.longitudinal = *m_lon_cmd;

  m_control_cmd_pub->publish(out);
}

void LatLonMuxer::latCtrlCmdCallback(
  const autoware_auto_control_msgs::msg::AckermannLateralCommand::SharedPtr input_msg)
{
  m_lat_cmd = input_msg;
  publishCmd();
}

void LatLonMuxer::lonCtrlCmdCallback(
  const autoware_auto_control_msgs::msg::LongitudinalCommand::SharedPtr input_msg)
{
  m_lon_cmd = std::make_shared<autoware_auto_control_msgs::msg::LongitudinalCommand>(*input_msg);
  publishCmd();
}
}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::trajectory_follower_nodes::LatLonMuxer)
