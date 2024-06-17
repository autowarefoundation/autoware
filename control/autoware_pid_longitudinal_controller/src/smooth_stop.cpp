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

#include "autoware/pid_longitudinal_controller/smooth_stop.hpp"

#include <experimental/optional>  // NOLINT

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{
void SmoothStop::init(const double pred_vel_in_target, const double pred_stop_dist)
{
  m_weak_acc_time = rclcpp::Clock{RCL_ROS_TIME}.now();

  // when distance to stopline is near the car
  if (pred_stop_dist < std::numeric_limits<double>::epsilon()) {
    m_strong_acc = m_params.min_strong_acc;
    return;
  }

  m_strong_acc = -std::pow(pred_vel_in_target, 2) / (2 * pred_stop_dist);
  m_strong_acc = std::max(std::min(m_strong_acc, m_params.max_strong_acc), m_params.min_strong_acc);
}

void SmoothStop::setParams(
  double max_strong_acc, double min_strong_acc, double weak_acc, double weak_stop_acc,
  double strong_stop_acc, double min_fast_vel, double min_running_vel, double min_running_acc,
  double weak_stop_time, double weak_stop_dist, double strong_stop_dist)
{
  m_params.max_strong_acc = max_strong_acc;
  m_params.min_strong_acc = min_strong_acc;
  m_params.weak_acc = weak_acc;
  m_params.weak_stop_acc = weak_stop_acc;
  m_params.strong_stop_acc = strong_stop_acc;

  m_params.min_fast_vel = min_fast_vel;
  m_params.min_running_vel = min_running_vel;
  m_params.min_running_acc = min_running_acc;
  m_params.weak_stop_time = weak_stop_time;

  m_params.weak_stop_dist = weak_stop_dist;
  m_params.strong_stop_dist = strong_stop_dist;

  m_is_set_params = true;
}

std::experimental::optional<double> SmoothStop::calcTimeToStop(
  const std::vector<std::pair<rclcpp::Time, double>> & vel_hist) const
{
  if (!m_is_set_params) {
    throw std::runtime_error("Trying to calculate uninitialized SmoothStop");
  }

  // return when vel_hist is empty
  const double vel_hist_size = static_cast<double>(vel_hist.size());
  if (vel_hist_size == 0.0) {
    return {};
  }

  // calculate some variables for fitting
  const rclcpp::Time current_ros_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  double mean_t = 0.0;
  double mean_v = 0.0;
  double sum_tv = 0.0;
  double sum_tt = 0.0;
  for (const auto & vel : vel_hist) {
    const double t = (vel.first - current_ros_time).seconds();
    const double v = vel.second;

    mean_t += t / vel_hist_size;
    mean_v += v / vel_hist_size;
    sum_tv += t * v;
    sum_tt += t * t;
  }

  // return when gradient a (of v = at + b) cannot be calculated.
  // See the following calculation of a
  if (std::abs(vel_hist_size * mean_t * mean_t - sum_tt) < std::numeric_limits<double>::epsilon()) {
    return {};
  }

  // calculate coefficients of linear function (v = at + b)
  const double a =
    (vel_hist_size * mean_t * mean_v - sum_tv) / (vel_hist_size * mean_t * mean_t - sum_tt);
  const double b = mean_v - a * mean_t;

  // return when v is independent of time (v = b)
  if (std::abs(a) < std::numeric_limits<double>::epsilon()) {
    return {};
  }

  // calculate time to stop by substituting v = 0 for v = at + b
  const double time_to_stop = -b / a;
  if (time_to_stop > 0) {
    return time_to_stop;
  }

  return {};
}

double SmoothStop::calculate(
  const double stop_dist, const double current_vel, const double current_acc,
  const std::vector<std::pair<rclcpp::Time, double>> & vel_hist, const double delay_time)
{
  if (!m_is_set_params) {
    throw std::runtime_error("Trying to calculate uninitialized SmoothStop");
  }

  // predict time to stop
  const auto time_to_stop = calcTimeToStop(vel_hist);

  // calculate some flags
  const bool is_fast_vel = std::abs(current_vel) > m_params.min_fast_vel;
  const bool is_running = std::abs(current_vel) > m_params.min_running_vel ||
                          std::abs(current_acc) > m_params.min_running_acc;

  // when exceeding the stopline (stop_dist is negative in these cases.)
  if (stop_dist < m_params.strong_stop_dist) {  // when exceeding the stopline much
    return m_params.strong_stop_acc;
  } else if (stop_dist < m_params.weak_stop_dist) {  // when exceeding the stopline a bit
    return m_params.weak_stop_acc;
  }

  // when the car is running
  if (is_running) {
    // when the car will not stop in a certain time
    if (time_to_stop && *time_to_stop > m_params.weak_stop_time + delay_time) {
      return m_strong_acc;
    } else if (!time_to_stop && is_fast_vel) {
      return m_strong_acc;
    }

    m_weak_acc_time = rclcpp::Clock{RCL_ROS_TIME}.now();
    return m_params.weak_acc;
  }

  // for 0.5 seconds after the car stopped
  if ((rclcpp::Clock{RCL_ROS_TIME}.now() - m_weak_acc_time).seconds() < 0.5) {
    return m_params.weak_acc;
  }

  // when the car is not running
  return m_params.strong_stop_acc;
}
}  // namespace autoware::motion::control::pid_longitudinal_controller
