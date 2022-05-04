// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include "goal_distance_calculator/goal_distance_calculator_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <tier4_debug_msgs/msg/float64_stamped.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace goal_distance_calculator
{
GoalDistanceCalculatorNode::GoalDistanceCalculatorNode(const rclcpp::NodeOptions & options)
: Node("goal_distance_calculator", options),
  self_pose_listener_(this),
  debug_publisher_(this, "goal_distance_calculator")
{
  using std::placeholders::_1;

  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);
  durable_qos.transient_local();

  // Node Parameter
  node_param_.update_rate = declare_parameter("update_rate", 10.0);
  node_param_.oneshot = declare_parameter("oneshot", true);

  // Core
  goal_distance_calculator_ = std::make_unique<GoalDistanceCalculator>();
  goal_distance_calculator_->setParam(param_);

  // Subscriber
  sub_route_ = create_subscription<autoware_auto_planning_msgs::msg::Route>(
    "/planning/mission_planning/route", queue_size,
    [&](const autoware_auto_planning_msgs::msg::Route::SharedPtr msg_ptr) { route_ = msg_ptr; });

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();

  // Timer
  const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&GoalDistanceCalculatorNode::onTimer, this));
  goal_distance_calculator_ = std::make_unique<GoalDistanceCalculator>();
}

bool GoalDistanceCalculatorNode::isDataReady()
{
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "waiting for current_pose...");
    return false;
  }

  if (!route_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "waiting for route msg...");
    return false;
  }

  return true;
}

bool GoalDistanceCalculatorNode::isDataTimeout()
{
  constexpr double th_pose_timeout = 1.0;
  const auto pose_time_diff = rclcpp::Time(current_pose_->header.stamp) - now();
  if (pose_time_diff.seconds() > th_pose_timeout) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "pose is timeout...");
    return true;
  }
  return false;
}

void GoalDistanceCalculatorNode::onTimer()
{
  current_pose_ = self_pose_listener_.getCurrentPose();

  if (!isDataReady()) {
    return;
  }

  if (isDataTimeout()) {
    return;
  }

  input_.current_pose = current_pose_;
  input_.route = route_;

  output_ = goal_distance_calculator_->update(input_);

  {
    using tier4_autoware_utils::rad2deg;
    const auto & deviation = output_.goal_deviation;

    debug_publisher_.publish<tier4_debug_msgs::msg::Float64Stamped>(
      "deviation/lateral", deviation.lateral);
    debug_publisher_.publish<tier4_debug_msgs::msg::Float64Stamped>(
      "deviation/longitudinal", deviation.longitudinal);
    debug_publisher_.publish<tier4_debug_msgs::msg::Float64Stamped>("deviation/yaw", deviation.yaw);
    debug_publisher_.publish<tier4_debug_msgs::msg::Float64Stamped>(
      "deviation/yaw_deg", rad2deg(deviation.yaw));
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "lateral: %f[mm], longitudinal: %f[mm], yaw: %f[deg]", 1000 * deviation.lateral,
      1000 * deviation.longitudinal, rad2deg(deviation.yaw));
  }

  if (node_param_.oneshot) {
    rclcpp::shutdown();
  }
}
}  // namespace goal_distance_calculator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(goal_distance_calculator::GoalDistanceCalculatorNode)
