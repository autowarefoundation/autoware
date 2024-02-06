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
  : show_debug_info(node->declare_parameter<bool>("node.show_debug_info")),
    ekf_rate(node->declare_parameter<double>("node.predict_frequency")),
    ekf_dt(1.0 / std::max(ekf_rate, 0.1)),
    tf_rate_(node->declare_parameter<double>("node.tf_rate")),
    publish_tf_(node->declare_parameter<bool>("node.publish_tf")),
    enable_yaw_bias_estimation(node->declare_parameter<bool>("node.enable_yaw_bias_estimation")),
    extend_state_step(node->declare_parameter<int>("node.extend_state_step")),
    pose_frame_id(node->declare_parameter<std::string>("misc.pose_frame_id")),
    pose_additional_delay(
      node->declare_parameter<double>("pose_measurement.pose_additional_delay")),
    pose_gate_dist(node->declare_parameter<double>("pose_measurement.pose_gate_dist")),
    pose_smoothing_steps(node->declare_parameter<int>("pose_measurement.pose_smoothing_steps")),
    twist_additional_delay(
      node->declare_parameter<double>("twist_measurement.twist_additional_delay")),
    twist_gate_dist(node->declare_parameter<double>("twist_measurement.twist_gate_dist")),
    twist_smoothing_steps(node->declare_parameter<int>("twist_measurement.twist_smoothing_steps")),
    proc_stddev_vx_c(node->declare_parameter<double>("process_noise.proc_stddev_vx_c")),
    proc_stddev_wz_c(node->declare_parameter<double>("process_noise.proc_stddev_wz_c")),
    proc_stddev_yaw_c(node->declare_parameter<double>("process_noise.proc_stddev_yaw_c")),
    z_filter_proc_dev(
      node->declare_parameter<double>("simple_1d_filter_parameters.z_filter_proc_dev")),
    roll_filter_proc_dev(
      node->declare_parameter<double>("simple_1d_filter_parameters.roll_filter_proc_dev")),
    pitch_filter_proc_dev(
      node->declare_parameter<double>("simple_1d_filter_parameters.pitch_filter_proc_dev")),
    pose_no_update_count_threshold_warn(
      node->declare_parameter<int>("diagnostics.pose_no_update_count_threshold_warn")),
    pose_no_update_count_threshold_error(
      node->declare_parameter<int>("diagnostics.pose_no_update_count_threshold_error")),
    twist_no_update_count_threshold_warn(
      node->declare_parameter<int>("diagnostics.twist_no_update_count_threshold_warn")),
    twist_no_update_count_threshold_error(
      node->declare_parameter<int>("diagnostics.twist_no_update_count_threshold_error")),
    threshold_observable_velocity_mps(
      node->declare_parameter<double>("misc.threshold_observable_velocity_mps"))
  {
  }

  const bool show_debug_info;
  const double ekf_rate;
  const double ekf_dt;
  const double tf_rate_;
  const bool publish_tf_;
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
  const double z_filter_proc_dev;
  const double roll_filter_proc_dev;
  const double pitch_filter_proc_dev;
  const size_t pose_no_update_count_threshold_warn;
  const size_t pose_no_update_count_threshold_error;
  const size_t twist_no_update_count_threshold_warn;
  const size_t twist_no_update_count_threshold_error;
  const double threshold_observable_velocity_mps;
};

#endif  // EKF_LOCALIZER__HYPER_PARAMETERS_HPP_
