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

#include "mrm_comfortable_stop_operator/mrm_comfortable_stop_operator_core.hpp"

namespace mrm_comfortable_stop_operator
{

MrmComfortableStopOperator::MrmComfortableStopOperator(const rclcpp::NodeOptions & node_options)
: Node("mrm_comfortable_stop_operator", node_options)
{
  // Parameter
  params_.update_rate = static_cast<int>(declare_parameter<int>("update_rate", 1));
  params_.min_acceleration = declare_parameter<double>("min_acceleration", -1.0);
  params_.max_jerk = declare_parameter<double>("max_jerk", 0.3);
  params_.min_jerk = declare_parameter<double>("min_jerk", 0.3);

  // Server
  service_operation_ = create_service<tier4_system_msgs::srv::OperateMrm>(
    "~/input/mrm/comfortable_stop/operate", std::bind(
                                              &MrmComfortableStopOperator::operateComfortableStop,
                                              this, std::placeholders::_1, std::placeholders::_2));

  // Publisher
  pub_status_ = create_publisher<tier4_system_msgs::msg::MrmBehaviorStatus>(
    "~/output/mrm/comfortable_stop/status", 1);
  pub_velocity_limit_ = create_publisher<tier4_planning_msgs::msg::VelocityLimit>(
    "~/output/velocity_limit", rclcpp::QoS{1}.transient_local());
  pub_velocity_limit_clear_command_ =
    create_publisher<tier4_planning_msgs::msg::VelocityLimitClearCommand>(
      "~/output/velocity_limit/clear", rclcpp::QoS{1}.transient_local());

  // Timer
  const auto update_period_ns = rclcpp::Rate(params_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&MrmComfortableStopOperator::onTimer, this));

  // Initialize
  status_.state = tier4_system_msgs::msg::MrmBehaviorStatus::AVAILABLE;
}

void MrmComfortableStopOperator::operateComfortableStop(
  const tier4_system_msgs::srv::OperateMrm::Request::SharedPtr request,
  const tier4_system_msgs::srv::OperateMrm::Response::SharedPtr response)
{
  if (request->operate == true) {
    publishVelocityLimit();
    status_.state = tier4_system_msgs::msg::MrmBehaviorStatus::OPERATING;
    response->response.success = true;
  } else {
    publishVelocityLimitClearCommand();
    status_.state = tier4_system_msgs::msg::MrmBehaviorStatus::AVAILABLE;
    response->response.success = true;
  }
}

void MrmComfortableStopOperator::publishStatus() const
{
  auto status = status_;
  status.stamp = this->now();
  pub_status_->publish(status);
}

void MrmComfortableStopOperator::publishVelocityLimit() const
{
  auto velocity_limit = tier4_planning_msgs::msg::VelocityLimit();
  velocity_limit.stamp = this->now();
  velocity_limit.max_velocity = 0;
  velocity_limit.use_constraints = true;
  velocity_limit.constraints.min_acceleration = static_cast<float>(params_.min_acceleration);
  velocity_limit.constraints.max_jerk = static_cast<float>(params_.max_jerk);
  velocity_limit.constraints.min_jerk = static_cast<float>(params_.min_jerk);
  velocity_limit.sender = "mrm_comfortable_stop_operator";

  pub_velocity_limit_->publish(velocity_limit);
}

void MrmComfortableStopOperator::publishVelocityLimitClearCommand() const
{
  auto velocity_limit_clear_command = tier4_planning_msgs::msg::VelocityLimitClearCommand();
  velocity_limit_clear_command.stamp = this->now();
  velocity_limit_clear_command.command = true;
  velocity_limit_clear_command.sender = "mrm_comfortable_stop_operator";

  pub_velocity_limit_clear_command_->publish(velocity_limit_clear_command);
}

void MrmComfortableStopOperator::onTimer() const { publishStatus(); }

}  // namespace mrm_comfortable_stop_operator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrm_comfortable_stop_operator::MrmComfortableStopOperator)
