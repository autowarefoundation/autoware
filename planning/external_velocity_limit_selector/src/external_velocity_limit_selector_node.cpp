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

#include "external_velocity_limit_selector/external_velocity_limit_selector_node.hpp"

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
VelocityLimit getHardestLimit(
  const std::unordered_map<std::string, VelocityLimit> & velocity_limits,
  const ExternalVelocityLimitSelectorNode::NodeParam & node_param)
{
  VelocityLimit hardest_limit{};
  hardest_limit.max_velocity = node_param.max_velocity;

  VelocityLimitConstraints normal_constraints{};
  normal_constraints.min_acceleration = node_param.normal_min_acc;
  normal_constraints.min_jerk = node_param.normal_min_jerk;
  normal_constraints.max_jerk = node_param.normal_max_jerk;

  double hardest_max_velocity = node_param.max_velocity;
  double hardest_max_jerk = 0.0;

  for (const auto & limit : velocity_limits) {
    // guard nan, inf
    const auto max_velocity = std::isfinite(limit.second.max_velocity) ? limit.second.max_velocity
                                                                       : node_param.max_velocity;

    // find hardest max velocity
    if (max_velocity < hardest_max_velocity) {
      hardest_limit.stamp = limit.second.stamp;
      hardest_limit.max_velocity = max_velocity;
      hardest_max_velocity = max_velocity;
    }

    const auto constraints =
      limit.second.use_constraints && std::isfinite(limit.second.constraints.max_jerk)
        ? limit.second.constraints
        : normal_constraints;

    // find hardest jerk
    if (hardest_max_jerk < constraints.max_jerk) {
      hardest_limit.constraints = constraints;
      hardest_limit.use_constraints = true;
      hardest_max_jerk = constraints.max_jerk;
    }
  }

  return hardest_limit;
}
}  // namespace

ExternalVelocityLimitSelectorNode::ExternalVelocityLimitSelectorNode(
  const rclcpp::NodeOptions & node_options)
: Node("external_velocity_limit_selector", node_options)
{
  using std::placeholders::_1;
  // Input
  sub_external_velocity_limit_from_api_ = this->create_subscription<VelocityLimit>(
    "input/velocity_limit_from_api", rclcpp::QoS{1}.transient_local(),
    std::bind(&ExternalVelocityLimitSelectorNode::onVelocityLimitFromAPI, this, _1));

  sub_external_velocity_limit_from_internal_ = this->create_subscription<VelocityLimit>(
    "input/velocity_limit_from_internal", rclcpp::QoS{1}.transient_local(),
    std::bind(&ExternalVelocityLimitSelectorNode::onVelocityLimitFromInternal, this, _1));

  sub_velocity_limit_clear_command_ = this->create_subscription<VelocityLimitClearCommand>(
    "input/velocity_limit_clear_command_from_internal", rclcpp::QoS{1}.transient_local(),
    std::bind(&ExternalVelocityLimitSelectorNode::onVelocityLimitClearCommand, this, _1));

  // Output
  pub_external_velocity_limit_ =
    this->create_publisher<VelocityLimit>("output/external_velocity_limit", 1);

  // Params
  {
    auto & p = node_param_;
    p.max_velocity = this->declare_parameter<double>("max_velocity", 20.0);
    p.normal_min_acc = this->declare_parameter<double>("normal.min_acc", -1.0);
    p.normal_max_acc = this->declare_parameter<double>("normal.max_acc", 1.0);
    p.normal_min_jerk = this->declare_parameter<double>("normal.min_jerk", -0.1);
    p.normal_max_jerk = this->declare_parameter<double>("normal.max_jerk", 0.1);
    p.limit_min_acc = this->declare_parameter<double>("limit.min_acc", -2.5);
    p.limit_max_acc = this->declare_parameter<double>("limit.max_acc", 2.5);
    p.limit_min_jerk = this->declare_parameter<double>("limit.min_jerk", -1.5);
    p.limit_max_jerk = this->declare_parameter<double>("limit.max_jerk", 1.5);
  }
}

void ExternalVelocityLimitSelectorNode::onVelocityLimitFromAPI(
  const VelocityLimit::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "set velocity limit. sender:%s", msg->sender.c_str());
  setVelocityLimitFromAPI(*msg);

  const auto velocity_limit = getCurrentVelocityLimit();
  publishVelocityLimit(velocity_limit);
}

void ExternalVelocityLimitSelectorNode::onVelocityLimitFromInternal(
  const VelocityLimit::ConstSharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "set velocity limit. sender:%s", msg->sender.c_str());
  setVelocityLimitFromInternal(*msg);

  const auto velocity_limit = getCurrentVelocityLimit();
  publishVelocityLimit(velocity_limit);
}

void ExternalVelocityLimitSelectorNode::onVelocityLimitClearCommand(
  const VelocityLimitClearCommand::ConstSharedPtr msg)
{
  if (!msg->command) {
    return;
  }

  clearVelocityLimit(msg->sender);

  const auto velocity_limit = getCurrentVelocityLimit();
  publishVelocityLimit(velocity_limit);
}

void ExternalVelocityLimitSelectorNode::publishVelocityLimit(const VelocityLimit & velocity_limit)
{
  pub_external_velocity_limit_->publish(velocity_limit);
}

void ExternalVelocityLimitSelectorNode::setVelocityLimitFromAPI(
  const VelocityLimit & velocity_limit)
{
  const std::string sender = "api";

  if (velocity_limit_table_.count(sender) == 0) {
    velocity_limit_table_.emplace(sender, velocity_limit);
  } else {
    velocity_limit_table_.at(sender) = velocity_limit;
    RCLCPP_DEBUG(get_logger(), "overwrite velocity limit. sender:%s", sender.c_str());
  }

  updateVelocityLimit();
}

void ExternalVelocityLimitSelectorNode::setVelocityLimitFromInternal(
  const VelocityLimit & velocity_limit)
{
  const auto sender = velocity_limit.sender;

  if (velocity_limit_table_.count(sender) == 0) {
    velocity_limit_table_.emplace(sender, velocity_limit);
  } else {
    velocity_limit_table_.at(sender) = velocity_limit;
    RCLCPP_DEBUG(get_logger(), "overwrite velocity limit. sender:%s", sender.c_str());
  }

  updateVelocityLimit();
}

void ExternalVelocityLimitSelectorNode::clearVelocityLimit(const std::string & sender)
{
  if (velocity_limit_table_.empty()) {
    RCLCPP_WARN(get_logger(), "no velocity limit has been set from internal.");
    return;
  }

  velocity_limit_table_.erase(sender);

  updateVelocityLimit();
}

void ExternalVelocityLimitSelectorNode::updateVelocityLimit()
{
  if (velocity_limit_table_.empty()) {
    VelocityLimit default_velocity_limit{};
    default_velocity_limit.stamp = this->now();
    default_velocity_limit.max_velocity = node_param_.max_velocity;

    hardest_limit_ = default_velocity_limit;

    RCLCPP_DEBUG(
      get_logger(),
      "no velocity limit has been set or latest velocity limit has been already cleared.");

    return;
  }

  hardest_limit_ = getHardestLimit(velocity_limit_table_, node_param_);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ExternalVelocityLimitSelectorNode)
