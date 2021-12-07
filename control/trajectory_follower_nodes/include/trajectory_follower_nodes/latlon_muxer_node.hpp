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

#ifndef TRAJECTORY_FOLLOWER_NODES__LATLON_MUXER_NODE_HPP_
#define TRAJECTORY_FOLLOWER_NODES__LATLON_MUXER_NODE_HPP_

#include <memory>
#include <string>

#include "trajectory_follower_nodes/visibility_control.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_control_msgs/msg/longitudinal_command.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
/// \brief Resources relating to the trajectory_follower_nodes package
namespace trajectory_follower_nodes
{
/// \class LatLonMuxer
/// \brief The node class used for muxing lateral and longitudinal messages
class LatLonMuxer : public rclcpp::Node
{
public:
  explicit TRAJECTORY_FOLLOWER_PUBLIC LatLonMuxer(const rclcpp::NodeOptions & node_options);

private:
  // \brief Callback for the lateral control command
  void latCtrlCmdCallback(const autoware_auto_control_msgs::msg::AckermannLateralCommand::SharedPtr msg);
  // \brief Callback for the longitudinal control command
  void lonCtrlCmdCallback(const autoware_auto_control_msgs::msg::LongitudinalCommand::SharedPtr msg);
  // \brief Publish the combined control command message
  void publishCmd();
  // \brief Check that the received messages are not too old
  // \return bool True if the stored messages timed out
  bool checkTimeout();

  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr m_control_cmd_pub;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannLateralCommand>::SharedPtr
    m_lat_control_cmd_sub;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::LongitudinalCommand>::SharedPtr
    m_lon_control_cmd_sub;

  std::shared_ptr<autoware_auto_control_msgs::msg::AckermannLateralCommand> m_lat_cmd;
  std::shared_ptr<autoware_auto_control_msgs::msg::LongitudinalCommand> m_lon_cmd;
  // \brief Timeout duration in seconds
  double m_timeout_thr_sec;
};  // class LatLonMuxer
}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER_NODES__LATLON_MUXER_NODE_HPP_
