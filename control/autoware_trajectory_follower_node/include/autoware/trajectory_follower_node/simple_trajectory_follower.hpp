//  Copyright 2022 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__SIMPLE_TRAJECTORY_FOLLOWER_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__SIMPLE_TRAJECTORY_FOLLOWER_HPP_

#include "autoware/universe_utils/ros/polling_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>

namespace simple_trajectory_follower
{
using autoware_control_msgs::msg::Control;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

class SimpleTrajectoryFollower : public rclcpp::Node
{
public:
  explicit SimpleTrajectoryFollower(const rclcpp::NodeOptions & options);
  ~SimpleTrajectoryFollower() = default;

private:
  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> sub_kinematics_{
    this, "~/input/kinematics"};
  autoware::universe_utils::InterProcessPollingSubscriber<Trajectory> sub_trajectory_{
    this, "~/input/trajectory"};
  rclcpp::Publisher<Control>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  Trajectory::ConstSharedPtr trajectory_;
  Odometry::ConstSharedPtr odometry_;
  TrajectoryPoint closest_traj_point_;
  bool use_external_target_vel_;
  double external_target_vel_;
  double lateral_deviation_;

  void onTimer();
  bool processData();
  void updateClosest();
  double calcSteerCmd();
  double calcAccCmd();
};

}  // namespace simple_trajectory_follower

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__SIMPLE_TRAJECTORY_FOLLOWER_HPP_
