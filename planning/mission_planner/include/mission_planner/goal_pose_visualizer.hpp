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

#ifndef MISSION_PLANNER__GOAL_POSE_VISUALIZER_HPP_
#define MISSION_PLANNER__GOAL_POSE_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace mission_planner
{
class GoalPoseVisualizer : public rclcpp::Node
{
public:
  explicit GoalPoseVisualizer(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::HADMapRoute>::SharedPtr sub_route_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;

  void echoBackRouteCallback(
    const autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr msg);
};

}  // namespace mission_planner
#endif  // MISSION_PLANNER__GOAL_POSE_VISUALIZER_HPP_
