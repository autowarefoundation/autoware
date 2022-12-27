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

#ifndef EXTERNAL_VELOCITY_LIMIT_SELECTOR__EXTERNAL_VELOCITY_LIMIT_SELECTOR_NODE_HPP_
#define EXTERNAL_VELOCITY_LIMIT_SELECTOR__EXTERNAL_VELOCITY_LIMIT_SELECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_debug_msgs/msg/string_stamped.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_planning_msgs/msg/velocity_limit_clear_command.hpp>

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>

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

  struct NodeParam
  {
    double max_velocity;
    // constraints for normal driving
    double normal_min_acc;
    double normal_max_acc;
    double normal_min_jerk;
    double normal_max_jerk;
    // constraints to be observed
    double limit_min_acc;
    double limit_max_acc;
    double limit_min_jerk;
    double limit_max_jerk;
  };

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
  NodeParam node_param_{};
  VelocityLimit hardest_limit_{};
  VelocityLimitTable velocity_limit_table_;
};

#endif  // EXTERNAL_VELOCITY_LIMIT_SELECTOR__EXTERNAL_VELOCITY_LIMIT_SELECTOR_NODE_HPP_
