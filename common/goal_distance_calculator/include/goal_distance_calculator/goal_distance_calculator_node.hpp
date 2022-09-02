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

#ifndef GOAL_DISTANCE_CALCULATOR__GOAL_DISTANCE_CALCULATOR_NODE_HPP_
#define GOAL_DISTANCE_CALCULATOR__GOAL_DISTANCE_CALCULATOR_NODE_HPP_

#include "goal_distance_calculator/goal_distance_calculator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <tier4_autoware_utils/ros/self_pose_listener.hpp>

#include <autoware_auto_planning_msgs/msg/route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>

#include <tf2_ros/transform_listener.h>

#include <memory>

namespace goal_distance_calculator
{
struct NodeParam
{
  double update_rate{0.0};
  bool oneshot{false};
};

class GoalDistanceCalculatorNode : public rclcpp::Node
{
public:
  explicit GoalDistanceCalculatorNode(const rclcpp::NodeOptions & options);

private:
  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_initial_pose_;
  tier4_autoware_utils::SelfPoseListener self_pose_listener_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Route>::SharedPtr sub_route_;

  // Data Buffer
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;
  autoware_auto_planning_msgs::msg::Route::SharedPtr route_;

  // Callback
  void onRoute(const autoware_auto_planning_msgs::msg::Route::ConstSharedPtr & msg);

  // Publisher
  tier4_autoware_utils::DebugPublisher debug_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  bool isDataReady();
  bool isDataTimeout();
  void onTimer();

  // Parameter
  NodeParam node_param_;
  Param param_;

  // Core
  Input input_;
  Output output_;
  std::unique_ptr<GoalDistanceCalculator> goal_distance_calculator_;
};
}  // namespace goal_distance_calculator
#endif  // GOAL_DISTANCE_CALCULATOR__GOAL_DISTANCE_CALCULATOR_NODE_HPP_
