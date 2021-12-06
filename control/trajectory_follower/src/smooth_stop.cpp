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

#include <algorithm>
#include <cmath>
#include <experimental/optional>  // NOLINT
#include <limits>
#include <utility>
#include <vector>

#include "trajectory_follower/smooth_stop.hpp"


namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
void SmoothStop::init(const float64_t pred_vel_in_target, const float64_t pred_stop_dist)
{
  m_weak_acc_time = rclcpp::Clock{RCL_ROS_TIME}.now();

  // when distance to stopline is near the car
  if (pred_stop_dist < std::numeric_limits<float64_t>::epsilon()) {
    m_strong_acc = m_params.min_strong_acc;
    return;
  }

  m_strong_acc = -std::pow(pred_vel_in_target, 2) / (2 * pred_stop_dist);
  m_strong_acc =
    std::max(std::min(m_strong_acc, m_params.max_strong_acc), m_params.min_strong_acc);
}

void SmoothStop::setParams(
  float64_t max_strong_acc, float64_t min_strong_acc, float64_t weak_acc, float64_t weak_stop_acc,
  float64_t strong_stop_acc, float64_t min_fast_vel, float64_t min_running_vel,
  float64_t min_running_acc,
  float64_t weak_stop_time, float64_t weak_stop_dist, float64_t strong_stop_dist)
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

std::experimental::optional<float64_t> SmoothStop::calcTimeToStop(
  const std::vector<std::pair<rclcpp::Time, float64_t>> & vel_hist) const
{
  if (!m_is_set_params) {
    throw std::runtime_error("Trying to calculate uninitialized SmoothStop");
  }

  // return when vel_hist is empty
  const float64_t vel_hist_size = static_cast<float64_t>(vel_hist.size());
  if (vel_hist_size == 0.0) {
    return {};
  }

  // calculate some variables for fitting
  const rclcpp::Time current_ros_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  float64_t mean_t = 0.0;
  float64_t mean_v = 0.0;
  float64_t sum_tv = 0.0;
  float64_t sum_tt = 0.0;
  for (const auto & vel : vel_hist) {
    const float64_t t = (vel.first - current_ros_time).seconds();
    const float64_t v = vel.second;

    mean_t += t / vel_hist_size;
    mean_v += v / vel_hist_size;
    sum_tv += t * v;
    sum_tt += t * t;
  }

  // return when gradient a (of v = at + b) cannot be calculated.
  // See the following calculation of a
  if (std::abs(vel_hist_size * mean_t * mean_t - sum_tt) <
    std::numeric_limits<float64_t>::epsilon())
  {
    return {};
  }

  // calculate coefficients of linear function (v = at + b)
  const float64_t a =
    (vel_hist_size * mean_t * mean_v - sum_tv) / (vel_hist_size * mean_t * mean_t - sum_tt);
  const float64_t b = mean_v - a * mean_t;

  // return when v is independent of time (v = b)
  if (std::abs(a) < std::numeric_limits<float64_t>::epsilon()) {
    return {};
  }

  // calculate time to stop by substituting v = 0 for v = at + b
  const float64_t time_to_stop = -b / a;
  if (time_to_stop > 0) {
    return time_to_stop;
  }

  return {};
}

float64_t SmoothStop::calculate(
  const float64_t stop_dist, const float64_t current_vel, const float64_t current_acc,
  const std::vector<std::pair<rclcpp::Time, float64_t>> & vel_hist, const float64_t delay_time)
{
  if (!m_is_set_params) {
    throw std::runtime_error("Trying to calculate uninitialized SmoothStop");
  }

  // predict time to stop
  const auto time_to_stop = calcTimeToStop(vel_hist);

  // calculate some flags
  const bool8_t is_fast_vel = std::abs(current_vel) > m_params.min_fast_vel;
  const bool8_t is_running = std::abs(current_vel) > m_params.min_running_vel ||
    std::abs(current_acc) > m_params.min_running_acc;

  // when exceeding the stopline (stop_dist is negative in these cases.)
  if (stop_dist < m_params.strong_stop_dist) {    // when exceeding the stopline much
    return m_params.strong_stop_acc;
  } else if (stop_dist < m_params.weak_stop_dist) {    // when exceeding the stopline a bit
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
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
