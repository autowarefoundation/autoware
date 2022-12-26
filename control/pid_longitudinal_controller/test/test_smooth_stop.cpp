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

#include "gtest/gtest.h"
#include "pid_longitudinal_controller/smooth_stop.hpp"
#include "rclcpp/rclcpp.hpp"

#include <utility>
#include <vector>

TEST(TestSmoothStop, calculate_stopping_acceleration)
{
  using ::autoware::motion::control::pid_longitudinal_controller::SmoothStop;
  using rclcpp::Duration;
  using rclcpp::Time;

  const double max_strong_acc = -0.5;
  const double min_strong_acc = -1.0;
  const double weak_acc = -0.3;
  const double weak_stop_acc = -0.8;
  const double strong_stop_acc = -3.4;
  const double max_fast_vel = 0.5;
  const double min_running_vel = 0.01;
  const double min_running_acc = 0.01;
  const double weak_stop_time = 0.8;
  const double weak_stop_dist = -0.3;
  const double strong_stop_dist = -0.5;

  const double delay_time = 0.17;

  SmoothStop ss;

  // Cannot calculate before setting parameters
  EXPECT_THROW(ss.calculate(0.0, 0.0, 0.0, {}, delay_time), std::runtime_error);

  ss.setParams(
    max_strong_acc, min_strong_acc, weak_acc, weak_stop_acc, strong_stop_acc, max_fast_vel,
    min_running_vel, min_running_acc, weak_stop_time, weak_stop_dist, strong_stop_dist);

  double vel_in_target;
  double stop_dist;
  double current_vel;
  double current_acc = 0.0;
  const Time now = rclcpp::Clock{RCL_ROS_TIME}.now();
  const std::vector<std::pair<Time, double>> velocity_history = {
    {now - Duration(3, 0), 3.0}, {now - Duration(2, 0), 2.0}, {now - Duration(1, 0), 1.0}};
  double accel;

  // strong stop when the stop distance is below the threshold
  vel_in_target = 5.0;
  stop_dist = strong_stop_dist - 0.1;
  current_vel = 2.0;
  ss.init(vel_in_target, stop_dist);
  accel = ss.calculate(stop_dist, current_vel, current_acc, velocity_history, delay_time);
  EXPECT_EQ(accel, strong_stop_acc);

  // weak stop when the stop distance is below the threshold (but not bellow the strong_stop_dist)
  stop_dist = weak_stop_dist - 0.1;
  current_vel = 2.0;
  ss.init(vel_in_target, stop_dist);
  accel = ss.calculate(stop_dist, current_vel, current_acc, velocity_history, delay_time);
  EXPECT_EQ(accel, weak_stop_acc);

  // if not running, weak accel for 0.5 seconds after the previous init or previous weak_acc
  rclcpp::Rate rate_quart(1.0 / 0.25);
  rclcpp::Rate rate_half(1.0 / 0.5);
  stop_dist = 0.0;
  current_vel = 0.0;
  EXPECT_EQ(
    ss.calculate(stop_dist, current_vel, current_acc, velocity_history, delay_time), weak_acc);
  rate_quart.sleep();
  EXPECT_EQ(
    ss.calculate(stop_dist, current_vel, current_acc, velocity_history, delay_time), weak_acc);
  rate_half.sleep();
  EXPECT_NE(
    ss.calculate(stop_dist, current_vel, current_acc, velocity_history, delay_time), weak_acc);

  // strong stop when the car is not running (and is at least 0.5seconds after initialization)
  EXPECT_EQ(
    ss.calculate(stop_dist, current_vel, current_acc, velocity_history, delay_time),
    strong_stop_acc);

  // accel between min/max_strong_acc when the car is running:
  // not predicted to exceed the stop line and is predicted to stop after weak_stop_time + delay
  stop_dist = 1.0;
  current_vel = 1.0;
  vel_in_target = 1.0;
  ss.init(vel_in_target, stop_dist);
  EXPECT_EQ(
    ss.calculate(stop_dist, current_vel, current_acc, velocity_history, delay_time),
    max_strong_acc);

  vel_in_target = std::sqrt(2.0);
  ss.init(vel_in_target, stop_dist);
  EXPECT_EQ(
    ss.calculate(stop_dist, current_vel, current_acc, velocity_history, delay_time),
    min_strong_acc);

  for (double vel_in_target = 1.1; vel_in_target < std::sqrt(2.0); vel_in_target += 0.1) {
    ss.init(vel_in_target, stop_dist);
    accel = ss.calculate(stop_dist, current_vel, current_acc, velocity_history, delay_time);
    EXPECT_GT(accel, min_strong_acc);
    EXPECT_LT(accel, max_strong_acc);
  }
}
