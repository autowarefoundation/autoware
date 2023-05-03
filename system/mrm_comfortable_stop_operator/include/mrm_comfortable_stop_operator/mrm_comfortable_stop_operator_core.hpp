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

#ifndef MRM_COMFORTABLE_STOP_OPERATOR__MRM_COMFORTABLE_STOP_OPERATOR_CORE_HPP_
#define MRM_COMFORTABLE_STOP_OPERATOR__MRM_COMFORTABLE_STOP_OPERATOR_CORE_HPP_

// Core
#include <memory>

// Autoware
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <tier4_planning_msgs/msg/velocity_limit_constraints.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>

// ROS 2 core
#include <rclcpp/rclcpp.hpp>

namespace mrm_comfortable_stop_operator
{

struct Parameters
{
  int update_rate;          // [Hz]
  double min_acceleration;  // [m/s^2]
  double max_jerk;          // [m/s^3]
  double min_jerk;          // [m/s^3]
};

class MrmComfortableStopOperator : public rclcpp::Node
{
public:
  explicit MrmComfortableStopOperator(const rclcpp::NodeOptions & node_options);

private:
  // Parameters
  Parameters params_;

  // Server
  rclcpp::Service<tier4_system_msgs::srv::OperateMrm>::SharedPtr service_operation_;

  void operateComfortableStop(
    const tier4_system_msgs::srv::OperateMrm::Request::SharedPtr request,
    const tier4_system_msgs::srv::OperateMrm::Response::SharedPtr response);

  // Publisher
  rclcpp::Publisher<tier4_system_msgs::msg::MrmBehaviorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimitClearCommand>::SharedPtr
    pub_velocity_limit_clear_command_;

  void publishStatus() const;
  void publishVelocityLimit() const;
  void publishVelocityLimitClearCommand() const;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer() const;

  // States
  tier4_system_msgs::msg::MrmBehaviorStatus status_;
};

}  // namespace mrm_comfortable_stop_operator

#endif  // MRM_COMFORTABLE_STOP_OPERATOR__MRM_COMFORTABLE_STOP_OPERATOR_CORE_HPP_
