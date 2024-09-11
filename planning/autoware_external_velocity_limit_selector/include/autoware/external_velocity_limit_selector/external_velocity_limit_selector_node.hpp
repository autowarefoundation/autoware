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

#ifndef AUTOWARE__EXTERNAL_VELOCITY_LIMIT_SELECTOR__EXTERNAL_VELOCITY_LIMIT_SELECTOR_NODE_HPP_
#define AUTOWARE__EXTERNAL_VELOCITY_LIMIT_SELECTOR__EXTERNAL_VELOCITY_LIMIT_SELECTOR_NODE_HPP_

#include <external_velocity_limit_selector_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_debug_msgs/msg/string_stamped.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_planning_msgs/msg/velocity_limit_clear_command.hpp>

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::external_velocity_limit_selector
{

using tier4_debug_msgs::msg::StringStamped;
using tier4_planning_msgs::msg::VelocityLimit;
using tier4_planning_msgs::msg::VelocityLimitClearCommand;
using tier4_planning_msgs::msg::VelocityLimitConstraints;

using VelocityLimitTable = std::unordered_map<std::string, VelocityLimit>;

class ExternalVelocityLimitSelectorNode : public rclcpp::Node
{
public:
  explicit ExternalVelocityLimitSelectorNode(const rclcpp::NodeOptions & node_options);

  void onVelocityLimitFromAPI(const VelocityLimit::ConstSharedPtr msg);
  void onVelocityLimitFromInternal(const VelocityLimit::ConstSharedPtr msg);
  void onVelocityLimitClearCommand(const VelocityLimitClearCommand::ConstSharedPtr msg);

private:
  rclcpp::Subscription<VelocityLimit>::SharedPtr sub_external_velocity_limit_from_api_;
  rclcpp::Subscription<VelocityLimit>::SharedPtr sub_external_velocity_limit_from_internal_;
  rclcpp::Subscription<VelocityLimitClearCommand>::SharedPtr sub_velocity_limit_clear_command_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_external_velocity_limit_;
  rclcpp::Publisher<StringStamped>::SharedPtr pub_debug_string_;

  void publishVelocityLimit(const VelocityLimit & velocity_limit);
  void setVelocityLimitFromAPI(const VelocityLimit & velocity_limit);
  void setVelocityLimitFromInternal(const VelocityLimit & velocity_limit);
  void clearVelocityLimit(const std::string & sender);
  void updateVelocityLimit();
  void publishDebugString();
  VelocityLimit getCurrentVelocityLimit() { return hardest_limit_; }

  // Parameters
  std::shared_ptr<::external_velocity_limit_selector::ParamListener> param_listener_;
  VelocityLimit hardest_limit_{};
  VelocityLimitTable velocity_limit_table_;
};
}  // namespace autoware::external_velocity_limit_selector

#endif  // AUTOWARE__EXTERNAL_VELOCITY_LIMIT_SELECTOR__EXTERNAL_VELOCITY_LIMIT_SELECTOR_NODE_HPP_
