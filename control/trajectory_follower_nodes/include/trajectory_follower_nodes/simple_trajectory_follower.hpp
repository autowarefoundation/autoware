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

#ifndef TRAJECTORY_FOLLOWER_NODES__SIMPLE_TRAJECTORY_FOLLOWER_HPP_
#define TRAJECTORY_FOLLOWER_NODES__SIMPLE_TRAJECTORY_FOLLOWER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>

namespace simple_trajectory_follower
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

class SimpleTrajectoryFollower : public rclcpp::Node
{
public:
  explicit SimpleTrajectoryFollower(const rclcpp::NodeOptions & options);
  ~SimpleTrajectoryFollower() = default;

private:
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  Trajectory::SharedPtr trajectory_;
  Odometry::SharedPtr odometry_;
  TrajectoryPoint closest_traj_point_;
  bool use_external_target_vel_;
  double external_target_vel_;
  double lateral_deviation_;

  void onTimer();
  bool checkData();
  void updateClosest();
  double calcSteerCmd();
  double calcAccCmd();
};

}  // namespace simple_trajectory_follower

#endif  // TRAJECTORY_FOLLOWER_NODES__SIMPLE_TRAJECTORY_FOLLOWER_HPP_
