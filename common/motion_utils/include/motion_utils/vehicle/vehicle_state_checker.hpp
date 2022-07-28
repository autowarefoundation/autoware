// Copyright 2022 TIER IV, Inc.
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

#ifndef MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_
#define MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <memory>

namespace motion_utils
{

using autoware_auto_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::Odometry;

class VehicleStopCheckerBase
{
public:
  VehicleStopCheckerBase(rclcpp::Node * node, double buffer_duration);
  rclcpp::Logger getLogger() { return logger_; }
  void addTwist(const TwistStamped & twist);
  bool isVehicleStopped(const double stop_duration = 0.0) const;

protected:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

private:
  double buffer_duration_;
  std::deque<TwistStamped> twist_buffer_;
};

class VehicleStopChecker : public VehicleStopCheckerBase
{
public:
  explicit VehicleStopChecker(rclcpp::Node * node);

protected:
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  Odometry::SharedPtr odometry_ptr_;

private:
  static constexpr double velocity_buffer_time_sec = 10.0;
  void onOdom(const Odometry::SharedPtr msg);
};

class VehicleArrivalChecker : public VehicleStopChecker
{
public:
  explicit VehicleArrivalChecker(rclcpp::Node * node);

  bool isVehicleStoppedAtStopPoint(const double stop_duration = 0.0) const;

private:
  static constexpr double th_arrived_distance_m = 1.0;

  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;

  Trajectory::SharedPtr trajectory_ptr_;

  void onTrajectory(const Trajectory::SharedPtr msg);
};
}  // namespace motion_utils

#endif  // MOTION_UTILS__VEHICLE__VEHICLE_STATE_CHECKER_HPP_
