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

#ifndef MRM_EMERGENCY_STOP_OPERATOR__MRM_EMERGENCY_STOP_OPERATOR_CORE_HPP_
#define MRM_EMERGENCY_STOP_OPERATOR__MRM_EMERGENCY_STOP_OPERATOR_CORE_HPP_

// Core
#include <functional>
#include <memory>

// Autoware
#include <autoware_control_msgs/msg/control.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>

// ROS 2 core
#include <rclcpp/rclcpp.hpp>

#include <vector>
namespace mrm_emergency_stop_operator
{
using autoware_control_msgs::msg::Control;
using tier4_system_msgs::msg::MrmBehaviorStatus;
using tier4_system_msgs::srv::OperateMrm;

struct Parameters
{
  int update_rate;             // [Hz]
  double target_acceleration;  // [m/s^2]
  double target_jerk;          // [m/s^3]
};

class MrmEmergencyStopOperator : public rclcpp::Node
{
public:
  explicit MrmEmergencyStopOperator(const rclcpp::NodeOptions & node_options);

private:
  // Parameters
  Parameters params_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  // Subscriber
  rclcpp::Subscription<Control>::SharedPtr sub_control_cmd_;

  void onControlCommand(Control::ConstSharedPtr msg);

  // Server
  rclcpp::Service<OperateMrm>::SharedPtr service_operation_;

  void operateEmergencyStop(
    const OperateMrm::Request::SharedPtr request, const OperateMrm::Response::SharedPtr response);

  // Publisher
  rclcpp::Publisher<MrmBehaviorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<Control>::SharedPtr pub_control_cmd_;

  void publishStatus() const;
  void publishControlCommand(const Control & command) const;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();

  // States
  MrmBehaviorStatus status_;
  Control prev_control_cmd_;
  bool is_prev_control_cmd_subscribed_;

  // Algorithm
  Control calcTargetAcceleration(const Control & prev_control_cmd) const;
};

}  // namespace mrm_emergency_stop_operator

#endif  // MRM_EMERGENCY_STOP_OPERATOR__MRM_EMERGENCY_STOP_OPERATOR_CORE_HPP_
