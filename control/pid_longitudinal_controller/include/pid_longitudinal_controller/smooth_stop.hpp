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

#ifndef PID_LONGITUDINAL_CONTROLLER__SMOOTH_STOP_HPP_
#define PID_LONGITUDINAL_CONTROLLER__SMOOTH_STOP_HPP_

#include "rclcpp/rclcpp.hpp"

#include <experimental/optional>  // NOLINT

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{

/**
 * @brief Smooth stop class to implement vehicle specific deceleration profiles
 */
class SmoothStop
{
public:
  /**
   * @brief initialize the state of the smooth stop
   * @param [in] pred_vel_in_target predicted ego velocity when the stop command will be executed
   * @param [in] pred_stop_dist predicted stop distance when the stop command will be executed
   */
  void init(const double pred_vel_in_target, const double pred_stop_dist);

  /**
   * @brief set the parameters of this smooth stop
   * @param [in] max_strong_acc maximum strong acceleration value [m/s²]
   * @param [in] min_strong_acc minumum strong acceleration value [m/s²]
   * @param [in] weak_acc weak acceleration value [m/s²]
   * @param [in] weak_stop_acc weak stopping acceleration value [m/s²]
   * @param [in] strong_stop_acc strong stopping acceleration value [m/s²]
   * @param [in] min_fast_vel minumum velocity to consider ego to be running fast [m/s]
   * @param [in] min_running_vel minimum velocity to consider ego to be running [m/s]
   * @param [in] min_running_acc minimum acceleration to consider ego to be running [m/s]
   * @param [in] weak_stop_time time allowed for stopping with a weak acceleration [s]
   * @param [in] weak_stop_dist distance to the stop point bellow which a weak accel is applied [m]
   * @param [in] strong_stop_dist distance to the stop point bellow which a strong accel is applied
   * [m]
   */
  void setParams(
    double max_strong_acc, double min_strong_acc, double weak_acc, double weak_stop_acc,
    double strong_stop_acc, double min_fast_vel, double min_running_vel, double min_running_acc,
    double weak_stop_time, double weak_stop_dist, double strong_stop_dist);

  /**
   * @brief predict time when car stops by fitting some latest observed velocity history
   *        with linear function (v = at + b)
   * @param [in] vel_hist history of previous ego velocities as (rclcpp::Time, double[m/s]) pairs
   * @throw std::runtime_error if parameters have not been set
   */
  std::experimental::optional<double> calcTimeToStop(
    const std::vector<std::pair<rclcpp::Time, double>> & vel_hist) const;

  /**
   * @brief calculate accel command while stopping
   *        Decrease velocity with m_strong_acc,
   *        then loose brake pedal with m_params.weak_acc to stop smoothly
   *        If the car is still running, input m_params.weak_stop_acc
   *        and then m_params.strong_stop_acc in steps not to exceed stopline too much
   * @param [in] stop_dist distance left to travel before stopping [m]
   * @param [in] current_vel current velocity of ego [m/s]
   * @param [in] current_acc current acceleration of ego [m/s²]
   * @param [in] vel_hist history of previous ego velocities as (rclcpp::Time, double[m/s]) pairs
   * @param [in] delay_time assumed time delay when the stop command will actually be executed
   * @throw std::runtime_error if parameters have not been set
   */
  double calculate(
    const double stop_dist, const double current_vel, const double current_acc,
    const std::vector<std::pair<rclcpp::Time, double>> & vel_hist, const double delay_time);

private:
  struct Params
  {
    double max_strong_acc;
    double min_strong_acc;
    double weak_acc;
    double weak_stop_acc;
    double strong_stop_acc;

    double min_fast_vel;
    double min_running_vel;
    double min_running_acc;
    double weak_stop_time;

    double weak_stop_dist;
    double strong_stop_dist;
  };
  Params m_params;

  double m_strong_acc;
  rclcpp::Time m_weak_acc_time;
  bool m_is_set_params = false;
};
}  // namespace autoware::motion::control::pid_longitudinal_controller

#endif  // PID_LONGITUDINAL_CONTROLLER__SMOOTH_STOP_HPP_
