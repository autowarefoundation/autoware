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

#ifndef INVALID_TRAJECTORY_PUBLISHER__INVALID_TRAJECTORY_PUBLISHER_HPP_
#define INVALID_TRAJECTORY_PUBLISHER__INVALID_TRAJECTORY_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <string>

namespace planning_validator
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

class InvalidTrajectoryPublisherNode : public rclcpp::Node
{
public:
  explicit InvalidTrajectoryPublisherNode(const rclcpp::NodeOptions & node_options);
  void onCurrentTrajectory(const Trajectory::ConstSharedPtr msg);
  void onTimer();

private:
  // ROS
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Trajectory::ConstSharedPtr current_trajectory_ = nullptr;
};
}  // namespace planning_validator

#endif  // INVALID_TRAJECTORY_PUBLISHER__INVALID_TRAJECTORY_PUBLISHER_HPP_
