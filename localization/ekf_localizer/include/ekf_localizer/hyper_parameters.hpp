// Copyright 2022 Autoware Foundation
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

#ifndef EKF_LOCALIZER__HYPER_PARAMETERS_HPP_
#define EKF_LOCALIZER__HYPER_PARAMETERS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <string>

class HyperParameters
{
public:
  explicit HyperParameters(rclcpp::Node * node)
  : show_debug_info(node->declare_parameter("show_debug_info", false)),
    ekf_rate(node->declare_parameter("predict_frequency", 50.0)),
    ekf_dt(1.0 / std::max(ekf_rate, 0.1)),
    tf_rate_(node->declare_parameter("tf_rate", 10.0)),
    enable_yaw_bias_estimation(node->declare_parameter("enable_yaw_bias_estimation", true)),
    extend_state_step(node->declare_parameter("extend_state_step", 50)),
    pose_frame_id(node->declare_parameter("pose_frame_id", std::string("map"))),
    pose_additional_delay(node->declare_parameter("pose_additional_delay", 0.0)),
    pose_gate_dist(node->declare_parameter("pose_gate_dist", 10000.0)),
    pose_smoothing_steps(node->declare_parameter("pose_smoothing_steps", 5)),
    twist_additional_delay(node->declare_parameter("twist_additional_delay", 0.0)),
    twist_gate_dist(node->declare_parameter("twist_gate_dist", 10000.0)),
    twist_smoothing_steps(node->declare_parameter("twist_smoothing_steps", 2)),
    proc_stddev_vx_c(node->declare_parameter("proc_stddev_vx_c", 5.0)),
    proc_stddev_wz_c(node->declare_parameter("proc_stddev_wz_c", 1.0)),
    proc_stddev_yaw_c(node->declare_parameter("proc_stddev_yaw_c", 0.005)),
    pose_no_update_count_threshold_warn(
      node->declare_parameter("pose_no_update_count_threshold_warn", 50)),
    pose_no_update_count_threshold_error(
      node->declare_parameter("pose_no_update_count_threshold_error", 250)),
    twist_no_update_count_threshold_warn(
      node->declare_parameter("twist_no_update_count_threshold_warn", 50)),
    twist_no_update_count_threshold_error(
      node->declare_parameter("twist_no_update_count_threshold_error", 250)),
    threshold_observable_velocity_mps(
      node->declare_parameter("threshold_observable_velocity_mps", 0.5))
  {
  }

  const bool show_debug_info;
  const double ekf_rate;
  const double ekf_dt;
  const double tf_rate_;
  const bool enable_yaw_bias_estimation;
  const int extend_state_step;
  const std::string pose_frame_id;
  const double pose_additional_delay;
  const double pose_gate_dist;
  const int pose_smoothing_steps;
  const double twist_additional_delay;
  const double twist_gate_dist;
  const int twist_smoothing_steps;
  const double proc_stddev_vx_c;   //!< @brief  vx process noise
  const double proc_stddev_wz_c;   //!< @brief  wz process noise
  const double proc_stddev_yaw_c;  //!< @brief  yaw process noise
  const size_t pose_no_update_count_threshold_warn;
  const size_t pose_no_update_count_threshold_error;
  const size_t twist_no_update_count_threshold_warn;
  const size_t twist_no_update_count_threshold_error;
  const double threshold_observable_velocity_mps;
};

#endif  // EKF_LOCALIZER__HYPER_PARAMETERS_HPP_
