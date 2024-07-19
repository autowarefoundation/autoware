// Copyright 2024 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use node file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NDT_SCAN_MATCHER__HYPER_PARAMETERS_HPP_
#define NDT_SCAN_MATCHER__HYPER_PARAMETERS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <multigrid_pclomp/multigrid_ndt_omp.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

enum class ConvergedParamType {
  TRANSFORM_PROBABILITY = 0,
  NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD = 1
};

enum class CovarianceEstimationType {
  FIXED_VALUE = 0,
  LAPLACE_APPROXIMATION = 1,
  MULTI_NDT = 2,
  MULTI_NDT_SCORE = 3,
};

struct HyperParameters
{
  struct Frame
  {
    std::string base_frame{};
    std::string ndt_base_frame{};
    std::string map_frame{};
  } frame{};

  struct SensorPoints
  {
    double timeout_sec{};
    double required_distance{};
  } sensor_points{};

  pclomp::NdtParams ndt{};
  bool ndt_regularization_enable{};

  struct InitialPoseEstimation
  {
    int64_t particles_num{};
    int64_t n_startup_trials{};
  } initial_pose_estimation{};

  struct Validation
  {
    double initial_pose_timeout_sec{};
    double initial_pose_distance_tolerance_m{};
    double initial_to_result_distance_tolerance_m{};
    double critical_upper_bound_exe_time_ms{};
    int64_t skipping_publish_num{};
  } validation{};

  struct ScoreEstimation
  {
    ConvergedParamType converged_param_type{};
    double converged_param_transform_probability{};
    double converged_param_nearest_voxel_transformation_likelihood{};
    struct NoGroundPoints
    {
      bool enable{};
      double z_margin_for_ground_removal{};
    } no_ground_points{};
  } score_estimation{};

  struct Covariance
  {
    std::array<double, 36> output_pose_covariance{};

    struct CovarianceEstimation
    {
      CovarianceEstimationType covariance_estimation_type{};
      std::vector<double> initial_pose_offset_model_x{};
      std::vector<double> initial_pose_offset_model_y{};
      double temperature{};
      double scale_factor{};
    } covariance_estimation{};
  } covariance{};

  struct DynamicMapLoading
  {
    double update_distance{};
    double map_radius{};
    double lidar_radius{};
  } dynamic_map_loading{};

public:
  explicit HyperParameters(rclcpp::Node * node)
  {
    frame.base_frame = node->declare_parameter<std::string>("frame.base_frame");
    frame.ndt_base_frame = node->declare_parameter<std::string>("frame.ndt_base_frame");
    frame.map_frame = node->declare_parameter<std::string>("frame.map_frame");

    sensor_points.timeout_sec = node->declare_parameter<double>("sensor_points.timeout_sec");
    sensor_points.required_distance =
      node->declare_parameter<double>("sensor_points.required_distance");

    ndt.trans_epsilon = node->declare_parameter<double>("ndt.trans_epsilon");
    ndt.step_size = node->declare_parameter<double>("ndt.step_size");
    ndt.resolution = node->declare_parameter<double>("ndt.resolution");
    ndt.max_iterations = static_cast<int>(node->declare_parameter<int64_t>("ndt.max_iterations"));
    ndt.num_threads = static_cast<int>(node->declare_parameter<int64_t>("ndt.num_threads"));
    ndt.num_threads = std::max(ndt.num_threads, 1);
    ndt_regularization_enable = node->declare_parameter<bool>("ndt.regularization.enable");
    ndt.regularization_scale_factor =
      static_cast<float>(node->declare_parameter<float>("ndt.regularization.scale_factor"));

    initial_pose_estimation.particles_num =
      node->declare_parameter<int64_t>("initial_pose_estimation.particles_num");
    initial_pose_estimation.n_startup_trials =
      node->declare_parameter<int64_t>("initial_pose_estimation.n_startup_trials");

    validation.initial_pose_timeout_sec =
      node->declare_parameter<double>("validation.initial_pose_timeout_sec");
    validation.initial_pose_distance_tolerance_m =
      node->declare_parameter<double>("validation.initial_pose_distance_tolerance_m");
    validation.initial_to_result_distance_tolerance_m =
      node->declare_parameter<double>("validation.initial_to_result_distance_tolerance_m");
    validation.critical_upper_bound_exe_time_ms =
      node->declare_parameter<double>("validation.critical_upper_bound_exe_time_ms");
    validation.skipping_publish_num =
      node->declare_parameter<int64_t>("validation.skipping_publish_num");

    const int64_t converged_param_type_tmp =
      node->declare_parameter<int64_t>("score_estimation.converged_param_type");
    score_estimation.converged_param_type =
      static_cast<ConvergedParamType>(converged_param_type_tmp);
    score_estimation.converged_param_transform_probability =
      node->declare_parameter<double>("score_estimation.converged_param_transform_probability");
    score_estimation.converged_param_nearest_voxel_transformation_likelihood =
      node->declare_parameter<double>(
        "score_estimation.converged_param_nearest_voxel_transformation_likelihood");
    score_estimation.no_ground_points.enable =
      node->declare_parameter<bool>("score_estimation.no_ground_points.enable");
    score_estimation.no_ground_points.z_margin_for_ground_removal = node->declare_parameter<double>(
      "score_estimation.no_ground_points.z_margin_for_ground_removal");

    std::vector<double> output_pose_covariance =
      node->declare_parameter<std::vector<double>>("covariance.output_pose_covariance");
    for (std::size_t i = 0; i < output_pose_covariance.size(); ++i) {
      covariance.output_pose_covariance[i] = output_pose_covariance[i];
    }
    const int64_t covariance_estimation_type_tmp = node->declare_parameter<int64_t>(
      "covariance.covariance_estimation.covariance_estimation_type");
    covariance.covariance_estimation.covariance_estimation_type =
      static_cast<CovarianceEstimationType>(covariance_estimation_type_tmp);
    covariance.covariance_estimation.initial_pose_offset_model_x =
      node->declare_parameter<std::vector<double>>(
        "covariance.covariance_estimation.initial_pose_offset_model_x");
    covariance.covariance_estimation.initial_pose_offset_model_y =
      node->declare_parameter<std::vector<double>>(
        "covariance.covariance_estimation.initial_pose_offset_model_y");
    if (
      covariance.covariance_estimation.initial_pose_offset_model_x.size() !=
      covariance.covariance_estimation.initial_pose_offset_model_y.size()) {
      std::stringstream message;
      message << "Invalid initial pose offset model parameters."
              << "Please make sure that the number of elements in "
              << "initial_pose_offset_model_x and initial_pose_offset_model_y are the same.";
      throw std::runtime_error(message.str());
    }
    covariance.covariance_estimation.temperature =
      node->declare_parameter<double>("covariance.covariance_estimation.temperature");
    covariance.covariance_estimation.scale_factor =
      node->declare_parameter<double>("covariance.covariance_estimation.scale_factor");

    dynamic_map_loading.update_distance =
      node->declare_parameter<double>("dynamic_map_loading.update_distance");
    dynamic_map_loading.map_radius =
      node->declare_parameter<double>("dynamic_map_loading.map_radius");
    dynamic_map_loading.lidar_radius =
      node->declare_parameter<double>("dynamic_map_loading.lidar_radius");
  }
};

#endif  // NDT_SCAN_MATCHER__HYPER_PARAMETERS_HPP_
