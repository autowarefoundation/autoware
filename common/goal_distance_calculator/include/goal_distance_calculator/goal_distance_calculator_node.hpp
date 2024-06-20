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

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <autoware/universe_utils/ros/self_pose_listener.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
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
  autoware::universe_utils::SelfPoseListener self_pose_listener_;
  autoware::universe_utils::InterProcessPollingSubscriber<autoware_planning_msgs::msg::LaneletRoute>
    sub_route_{this, "/planning/mission_planning/route"};

  // Publisher
  autoware::universe_utils::DebugPublisher debug_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  bool tryGetCurrentPose(geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose);
  bool tryGetRoute(autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route);
  void onTimer();

  // Parameter
  NodeParam node_param_;
  Param param_;

  // Core
  std::unique_ptr<GoalDistanceCalculator> goal_distance_calculator_;
};
}  // namespace goal_distance_calculator
#endif  // GOAL_DISTANCE_CALCULATOR__GOAL_DISTANCE_CALCULATOR_NODE_HPP_
