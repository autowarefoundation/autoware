// Copyright 2015-2019 Autoware Foundation
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

#include "ndt_scan_matcher/ndt_scan_matcher_core.hpp"

#include "ndt_scan_matcher/matrix_type.hpp"
#include "ndt_scan_matcher/particle.hpp"
#include "ndt_scan_matcher/pose_array_interpolator.hpp"
#include "ndt_scan_matcher/util_func.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <thread>

tier4_debug_msgs::msg::Float32Stamped make_float32_stamped(
  const builtin_interfaces::msg::Time & stamp, const float data)
{
  using T = tier4_debug_msgs::msg::Float32Stamped;
  return tier4_debug_msgs::build<T>().stamp(stamp).data(data);
}

tier4_debug_msgs::msg::Int32Stamped make_int32_stamped(
  const builtin_interfaces::msg::Time & stamp, const int32_t data)
{
  using T = tier4_debug_msgs::msg::Int32Stamped;
  return tier4_debug_msgs::build<T>().stamp(stamp).data(data);
}

bool validate_local_optimal_solution_oscillation(
  const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array,
  const float oscillation_threshold, const float inversion_vector_threshold)
{
  bool prev_oscillation = false;
  int oscillation_cnt = 0;

  for (size_t i = 2; i < result_pose_msg_array.size(); ++i) {
    const Eigen::Vector3d current_pose = point_to_vector3d(result_pose_msg_array.at(i).position);
    const Eigen::Vector3d prev_pose = point_to_vector3d(result_pose_msg_array.at(i - 1).position);
    const Eigen::Vector3d prev_prev_pose =
      point_to_vector3d(result_pose_msg_array.at(i - 2).position);
    const auto current_vec = current_pose - prev_pose;
    const auto prev_vec = (prev_pose - prev_prev_pose).normalized();
    const bool oscillation = prev_vec.dot(current_vec) < inversion_vector_threshold;
    if (prev_oscillation && oscillation) {
      if (oscillation_cnt > oscillation_threshold) {
        return true;
      }
      ++oscillation_cnt;
    } else {
      oscillation_cnt = 0;
    }
    prev_oscillation = oscillation;
  }
  return false;
}

// cspell: ignore degrounded
NDTScanMatcher::NDTScanMatcher()
: Node("ndt_scan_matcher"),
  tf2_broadcaster_(*this),
  ndt_ptr_(new NormalDistributionsTransform),
  state_ptr_(new std::map<std::string, std::string>),
  base_frame_("base_link"),
  ndt_base_frame_("ndt_base_link"),
  map_frame_("map"),
  converged_param_type_(ConvergedParamType::TRANSFORM_PROBABILITY),
  converged_param_transform_probability_(4.5),
  converged_param_nearest_voxel_transformation_likelihood_(2.3),
  initial_estimate_particles_num_(100),
  initial_pose_timeout_sec_(1.0),
  initial_pose_distance_tolerance_m_(10.0),
  inversion_vector_threshold_(-0.9),
  oscillation_threshold_(10),
  regularization_enabled_(declare_parameter("regularization_enabled", false)),
  estimate_scores_for_degrounded_scan_(
    declare_parameter("estimate_scores_for_degrounded_scan", false)),
  z_margin_for_ground_removal_(declare_parameter("z_margin_for_ground_removal", 0.8))
{
  (*state_ptr_)["state"] = "Initializing";
  is_activated_ = false;

  int points_queue_size = this->declare_parameter("input_sensor_points_queue_size", 0);
  points_queue_size = std::max(points_queue_size, 0);
  RCLCPP_INFO(get_logger(), "points_queue_size: %d", points_queue_size);

  base_frame_ = this->declare_parameter("base_frame", base_frame_);
  RCLCPP_INFO(get_logger(), "base_frame_id: %s", base_frame_.c_str());

  ndt_base_frame_ = this->declare_parameter("ndt_base_frame", ndt_base_frame_);
  RCLCPP_INFO(get_logger(), "ndt_base_frame_id: %s", ndt_base_frame_.c_str());

  pclomp::NdtParams ndt_params;
  ndt_params.trans_epsilon = this->declare_parameter<double>("trans_epsilon");
  ndt_params.step_size = this->declare_parameter<double>("step_size");
  ndt_params.resolution = this->declare_parameter<double>("resolution");
  ndt_params.max_iterations = this->declare_parameter<int>("max_iterations");
  ndt_params.num_threads = this->declare_parameter<int>("num_threads");
  ndt_params.num_threads = std::max(ndt_params.num_threads, 1);
  ndt_params.regularization_scale_factor =
    this->declare_parameter<float>("regularization_scale_factor");
  ndt_ptr_->setParams(ndt_params);

  RCLCPP_INFO(
    get_logger(), "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d",
    ndt_params.trans_epsilon, ndt_params.step_size, ndt_params.resolution,
    ndt_params.max_iterations);

  int converged_param_type_tmp = this->declare_parameter("converged_param_type", 0);
  converged_param_type_ = static_cast<ConvergedParamType>(converged_param_type_tmp);

  converged_param_transform_probability_ = this->declare_parameter(
    "converged_param_transform_probability", converged_param_transform_probability_);
  converged_param_nearest_voxel_transformation_likelihood_ = this->declare_parameter(
    "converged_param_nearest_voxel_transformation_likelihood",
    converged_param_nearest_voxel_transformation_likelihood_);

  initial_pose_timeout_sec_ =
    this->declare_parameter("initial_pose_timeout_sec", initial_pose_timeout_sec_);

  initial_pose_distance_tolerance_m_ = this->declare_parameter(
    "initial_pose_distance_tolerance_m", initial_pose_distance_tolerance_m_);

  std::vector<double> output_pose_covariance =
    this->declare_parameter<std::vector<double>>("output_pose_covariance");
  for (std::size_t i = 0; i < output_pose_covariance.size(); ++i) {
    output_pose_covariance_[i] = output_pose_covariance[i];
  }

  rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group;
  initial_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::CallbackGroup::SharedPtr main_callback_group;
  main_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = initial_pose_callback_group;

  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", 100,
    std::bind(&NDTScanMatcher::callback_initial_pose, this, std::placeholders::_1),
    initial_pose_sub_opt);
  sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(points_queue_size),
    std::bind(&NDTScanMatcher::callback_sensor_points, this, std::placeholders::_1), main_sub_opt);
  regularization_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "regularization_pose_with_covariance", 100,
      std::bind(&NDTScanMatcher::callback_regularization_pose, this, std::placeholders::_1));

  sensor_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned", 10);
  no_ground_points_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned_no_ground", 10);
  ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "ndt_pose_with_covariance", 10);
  initial_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_pose_with_covariance", 10);
  exe_time_pub_ = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("exe_time_ms", 10);
  transform_probability_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("transform_probability", 10);
  nearest_voxel_transformation_likelihood_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "nearest_voxel_transformation_likelihood", 10);
  no_ground_transform_probability_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "no_ground_transform_probability", 10);
  no_ground_nearest_voxel_transformation_likelihood_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "no_ground_nearest_voxel_transformation_likelihood", 10);
  iteration_num_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Int32Stamped>("iteration_num", 10);
  initial_to_result_distance_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("initial_to_result_distance", 10);
  initial_to_result_distance_old_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_old", 10);
  initial_to_result_distance_new_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_new", 10);
  ndt_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ndt_marker", 10);
  diagnostics_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  service_trigger_node_ = this->create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &NDTScanMatcher::service_trigger_node, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), main_callback_group);

  diagnostic_thread_ = std::thread(&NDTScanMatcher::timer_diagnostic, this);
  diagnostic_thread_.detach();

  tf2_listener_module_ = std::make_shared<Tf2ListenerModule>(this);

  if (this->declare_parameter<bool>("use_dynamic_map_loading")) {
    map_update_module_ = std::make_unique<MapUpdateModule>(
      this, &ndt_ptr_mtx_, ndt_ptr_, tf2_listener_module_, map_frame_, main_callback_group,
      state_ptr_);
  } else {
    map_module_ = std::make_unique<MapModule>(this, &ndt_ptr_mtx_, ndt_ptr_, main_callback_group);
    pose_init_module_ = std::make_unique<PoseInitializationModule>(
      this, &ndt_ptr_mtx_, ndt_ptr_, tf2_listener_module_, map_frame_, main_callback_group,
      state_ptr_);
  }
}

void NDTScanMatcher::timer_diagnostic()
{
  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    diagnostic_msgs::msg::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "ndt_scan_matcher";
    diag_status_msg.hardware_id = "";

    for (const auto & key_value : (*state_ptr_)) {
      diagnostic_msgs::msg::KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (state_ptr_->count("state") && (*state_ptr_)["state"] == "Initializing") {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag_status_msg.message += "Initializing State. ";
    }
    if (
      state_ptr_->count("skipping_publish_num") &&
      std::stoi((*state_ptr_)["skipping_publish_num"]) > 1) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag_status_msg.message += "skipping_publish_num > 1. ";
    }
    if (
      state_ptr_->count("skipping_publish_num") &&
      std::stoi((*state_ptr_)["skipping_publish_num"]) >= 5) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
    }
    // Ignore local optimal solution
    if (
      state_ptr_->count("is_local_optimal_solution_oscillation") &&
      std::stoi((*state_ptr_)["is_local_optimal_solution_oscillation"])) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diag_status_msg.message = "local optimal solution oscillation occurred";
    }

    diagnostic_msgs::msg::DiagnosticArray diag_msg;
    diag_msg.header.stamp = this->now();
    diag_msg.status.push_back(diag_status_msg);

    diagnostics_pub_->publish(diag_msg);

    rate.sleep();
  }
}

void NDTScanMatcher::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  if (!is_activated_) return;

  // lock mutex for initial pose
  std::lock_guard<std::mutex> initial_pose_array_lock(initial_pose_array_mtx_);
  // if rosbag restart, clear buffer
  if (!initial_pose_msg_ptr_array_.empty()) {
    const builtin_interfaces::msg::Time & t_front =
      initial_pose_msg_ptr_array_.front()->header.stamp;
    const builtin_interfaces::msg::Time & t_msg = initial_pose_msg_ptr->header.stamp;
    if (t_front.sec > t_msg.sec || (t_front.sec == t_msg.sec && t_front.nanosec > t_msg.nanosec)) {
      initial_pose_msg_ptr_array_.clear();
    }
  }

  if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
    initial_pose_msg_ptr_array_.push_back(initial_pose_msg_ptr);
  } else {
    // get TF from pose_frame to map_frame
    auto TF_pose_to_map_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
    tf2_listener_module_->get_transform(
      this->now(), map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

    // transform pose_frame to map_frame
    auto mapTF_initial_pose_msg_ptr =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    // mapTF_initial_pose_msg_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
    *mapTF_initial_pose_msg_ptr = transform(*initial_pose_msg_ptr, *TF_pose_to_map_ptr);
    initial_pose_msg_ptr_array_.push_back(mapTF_initial_pose_msg_ptr);
  }
}

void NDTScanMatcher::callback_regularization_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr)
{
  regularization_pose_msg_ptr_array_.push_back(pose_conv_msg_ptr);
}

void NDTScanMatcher::callback_sensor_points(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_sensorTF_msg_ptr)
{
  // mutex ndt_ptr_
  std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);

  const auto exe_start_time = std::chrono::system_clock::now();
  const rclcpp::Time sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

  // preprocess input pointcloud
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_sensorTF_ptr(
    new pcl::PointCloud<PointSource>);
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_baselinkTF_ptr(
    new pcl::PointCloud<PointSource>);
  const std::string & sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;

  pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);
  transform_sensor_measurement(
    sensor_frame, base_frame_, sensor_points_sensorTF_ptr, sensor_points_baselinkTF_ptr);
  ndt_ptr_->setInputSource(sensor_points_baselinkTF_ptr);
  if (!is_activated_) return;

  // calculate initial pose
  std::unique_lock<std::mutex> initial_pose_array_lock(initial_pose_array_mtx_);
  if (initial_pose_msg_ptr_array_.size() <= 1) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "No Pose!");
    return;
  }
  PoseArrayInterpolator interpolator(
    this, sensor_ros_time, initial_pose_msg_ptr_array_, initial_pose_timeout_sec_,
    initial_pose_distance_tolerance_m_);
  if (!interpolator.is_success()) return;
  pop_old_pose(initial_pose_msg_ptr_array_, sensor_ros_time);
  initial_pose_array_lock.unlock();

  // if regularization is enabled and available, set pose to NDT for regularization
  if (regularization_enabled_) add_regularization_pose(sensor_ros_time);

  if (ndt_ptr_->getInputTarget() == nullptr) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "No MAP!");
    return;
  }

  // perform ndt scan matching
  (*state_ptr_)["state"] = "Aligning";
  const Eigen::Matrix4f initial_pose_matrix =
    pose_to_matrix4f(interpolator.get_current_pose().pose.pose);
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  ndt_ptr_->align(*output_cloud, initial_pose_matrix);
  const pclomp::NdtResult ndt_result = ndt_ptr_->getResult();
  (*state_ptr_)["state"] = "Sleeping";

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() /
    1000.0;

  const geometry_msgs::msg::Pose result_pose_msg = matrix4f_to_pose(ndt_result.pose);
  std::vector<geometry_msgs::msg::Pose> transformation_msg_array;
  for (const auto & pose_matrix : ndt_result.transformation_array) {
    geometry_msgs::msg::Pose pose_ros = matrix4f_to_pose(pose_matrix);
    transformation_msg_array.push_back(pose_ros);
  }

  // perform several validations
  /*****************************************************************************
  The reason the add 2 to the ndt_ptr_->getMaximumIterations() is that there are bugs in
  implementation of ndt.
  1. gradient descent method ends when the iteration is greater than max_iteration if it does not
  converge (be careful it's 'greater than' instead of 'greater equal than'.)
     https://github.com/tier4/autoware.iv/blob/2323e5baa0b680d43a9219f5fb3b7a11dd9edc82/localization/pose_estimator/ndt_scan_matcher/ndt_omp/include/ndt_omp/ndt_omp_impl.hpp#L212
  2. iterate iteration count when end of gradient descent function.
     https://github.com/tier4/autoware.iv/blob/2323e5baa0b680d43a9219f5fb3b7a11dd9edc82/localization/pose_estimator/ndt_scan_matcher/ndt_omp/include/ndt_omp/ndt_omp_impl.hpp#L217

  These bugs are now resolved in original pcl implementation.
  https://github.com/PointCloudLibrary/pcl/blob/424c1c6a0ca97d94ca63e5daff4b183a4db8aae4/registration/include/pcl/registration/impl/ndt.hpp#L73-L180
  *****************************************************************************/
  bool is_ok_iteration_num =
    validate_num_iteration(ndt_result.iteration_num, ndt_ptr_->getMaximumIterations() + 2);
  bool is_local_optimal_solution_oscillation = false;
  if (!is_ok_iteration_num) {
    is_local_optimal_solution_oscillation = validate_local_optimal_solution_oscillation(
      transformation_msg_array, oscillation_threshold_, inversion_vector_threshold_);
  }
  bool is_ok_converged_param = validate_converged_param(
    ndt_result.transform_probability, ndt_result.nearest_voxel_transformation_likelihood);
  bool is_converged = is_ok_iteration_num && is_ok_converged_param;
  static size_t skipping_publish_num = 0;
  if (is_converged) {
    skipping_publish_num = 0;
  } else {
    ++skipping_publish_num;
    RCLCPP_WARN(get_logger(), "Not Converged");
  }

  // publish
  initial_pose_with_covariance_pub_->publish(interpolator.get_current_pose());
  exe_time_pub_->publish(make_float32_stamped(sensor_ros_time, exe_time));
  transform_probability_pub_->publish(
    make_float32_stamped(sensor_ros_time, ndt_result.transform_probability));
  nearest_voxel_transformation_likelihood_pub_->publish(
    make_float32_stamped(sensor_ros_time, ndt_result.nearest_voxel_transformation_likelihood));
  iteration_num_pub_->publish(make_int32_stamped(sensor_ros_time, ndt_result.iteration_num));
  publish_tf(sensor_ros_time, result_pose_msg);
  publish_pose(sensor_ros_time, result_pose_msg, is_converged);
  publish_marker(sensor_ros_time, transformation_msg_array);
  publish_initial_to_result_distances(
    sensor_ros_time, result_pose_msg, interpolator.get_current_pose(), interpolator.get_old_pose(),
    interpolator.get_new_pose());

  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_mapTF_ptr(
    new pcl::PointCloud<PointSource>);
  pcl::transformPointCloud(
    *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, ndt_result.pose);
  publish_point_cloud(sensor_ros_time, map_frame_, sensor_points_mapTF_ptr);

  // whether use de-grounded points calculate score
  if (estimate_scores_for_degrounded_scan_) {
    // remove ground
    pcl::shared_ptr<pcl::PointCloud<PointSource>> no_ground_points_mapTF_ptr(
      new pcl::PointCloud<PointSource>);
    for (std::size_t i = 0; i < sensor_points_mapTF_ptr->size(); i++) {
      if (
        sensor_points_mapTF_ptr->points[i].z - matrix4f_to_pose(ndt_result.pose).position.z >
        z_margin_for_ground_removal_) {
        no_ground_points_mapTF_ptr->points.push_back(sensor_points_mapTF_ptr->points[i]);
      }
    }
    // pub remove-ground points
    sensor_msgs::msg::PointCloud2 no_ground_points_mapTF_msg;
    pcl::toROSMsg(*no_ground_points_mapTF_ptr, no_ground_points_mapTF_msg);
    no_ground_points_mapTF_msg.header.stamp = sensor_ros_time;
    no_ground_points_mapTF_msg.header.frame_id = map_frame_;
    no_ground_points_aligned_pose_pub_->publish(no_ground_points_mapTF_msg);
    // calculate score
    const float no_ground_transform_probability =
      ndt_ptr_->calculateTransformationProbability(*no_ground_points_mapTF_ptr);
    const float no_ground_nearest_voxel_transformation_likelihood =
      ndt_ptr_->calculateNearestVoxelTransformationLikelihood(*no_ground_points_mapTF_ptr);
    // pub score
    no_ground_transform_probability_pub_->publish(
      make_float32_stamped(sensor_ros_time, no_ground_transform_probability));
    no_ground_nearest_voxel_transformation_likelihood_pub_->publish(
      make_float32_stamped(sensor_ros_time, no_ground_nearest_voxel_transformation_likelihood));
  }

  (*state_ptr_)["transform_probability"] = std::to_string(ndt_result.transform_probability);
  (*state_ptr_)["nearest_voxel_transformation_likelihood"] =
    std::to_string(ndt_result.nearest_voxel_transformation_likelihood);
  (*state_ptr_)["iteration_num"] = std::to_string(ndt_result.iteration_num);
  (*state_ptr_)["skipping_publish_num"] = std::to_string(skipping_publish_num);
  if (is_local_optimal_solution_oscillation) {
    (*state_ptr_)["is_local_optimal_solution_oscillation"] = "1";
  } else {
    (*state_ptr_)["is_local_optimal_solution_oscillation"] = "0";
  }
}

void NDTScanMatcher::transform_sensor_measurement(
  const std::string source_frame, const std::string target_frame,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_input_ptr,
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_output_ptr)
{
  auto TF_target_to_source_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  tf2_listener_module_->get_transform(
    this->now(), target_frame, source_frame, TF_target_to_source_ptr);
  const geometry_msgs::msg::PoseStamped target_to_source_pose_stamped =
    tier4_autoware_utils::transform2pose(*TF_target_to_source_ptr);
  const Eigen::Matrix4f base_to_sensor_matrix =
    pose_to_matrix4f(target_to_source_pose_stamped.pose);
  pcl::transformPointCloud(
    *sensor_points_input_ptr, *sensor_points_output_ptr, base_to_sensor_matrix);
}

void NDTScanMatcher::publish_tf(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;
  tf2_broadcaster_.sendTransform(
    tier4_autoware_utils::pose2transform(result_pose_stamped_msg, ndt_base_frame_));
}

void NDTScanMatcher::publish_pose(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const bool is_converged)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;
  result_pose_with_cov_msg.pose.covariance = output_pose_covariance_;

  if (is_converged) {
    ndt_pose_pub_->publish(result_pose_stamped_msg);
    ndt_pose_with_covariance_pub_->publish(result_pose_with_cov_msg);
  }
}

void NDTScanMatcher::publish_point_cloud(
  const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_mapTF_ptr)
{
  sensor_msgs::msg::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = frame_id;
  sensor_aligned_pose_pub_->publish(sensor_points_mapTF_msg);
}

void NDTScanMatcher::publish_marker(
  const rclcpp::Time & sensor_ros_time, const std::vector<geometry_msgs::msg::Pose> & pose_array)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = sensor_ros_time;
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = tier4_autoware_utils::createMarkerScale(0.3, 0.1, 0.1);
  int i = 0;
  marker.ns = "result_pose_matrix_array";
  marker.action = visualization_msgs::msg::Marker::ADD;
  for (const auto & pose_msg : pose_array) {
    marker.id = i++;
    marker.pose = pose_msg;
    marker.color = exchange_color_crc((1.0 * i) / 15.0);
    marker_array.markers.push_back(marker);
  }

  // TODO(Tier IV): delete old marker
  for (; i < ndt_ptr_->getMaximumIterations() + 2;) {
    marker.id = i++;
    marker.pose = geometry_msgs::msg::Pose();
    marker.color = exchange_color_crc(0);
    marker_array.markers.push_back(marker);
  }
  ndt_marker_pub_->publish(marker_array);
}

void NDTScanMatcher::publish_initial_to_result_distances(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg)
{
  const float initial_to_result_distance =
    norm(initial_pose_cov_msg.pose.pose.position, result_pose_msg.position);
  initial_to_result_distance_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance));

  const float initial_to_result_distance_old =
    norm(initial_pose_old_msg.pose.pose.position, result_pose_msg.position);
  initial_to_result_distance_old_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance_old));

  const float initial_to_result_distance_new =
    norm(initial_pose_new_msg.pose.pose.position, result_pose_msg.position);
  initial_to_result_distance_new_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance_new));
}

bool NDTScanMatcher::validate_num_iteration(const int iter_num, const int max_iter_num)
{
  bool is_ok_iter_num = iter_num < max_iter_num;
  if (!is_ok_iter_num) {
    RCLCPP_WARN(
      get_logger(),
      "The number of iterations has reached its upper limit. The number of iterations: %d, Limit: "
      "%d",
      iter_num, max_iter_num);
  }
  return is_ok_iter_num;
}

bool NDTScanMatcher::validate_score(
  const double score, const double score_threshold, const std::string score_name)
{
  bool is_ok_score = score > score_threshold;
  if (!is_ok_score) {
    RCLCPP_WARN(
      get_logger(), "%s is below the threshold. Score: %lf, Threshold: %lf", score_name.c_str(),
      score, score_threshold);
  }
  return is_ok_score;
}

bool NDTScanMatcher::validate_converged_param(
  const double & transform_probability, const double & nearest_voxel_transformation_likelihood)
{
  bool is_ok_converged_param = false;
  if (converged_param_type_ == ConvergedParamType::TRANSFORM_PROBABILITY) {
    is_ok_converged_param = validate_score(
      transform_probability, converged_param_transform_probability_, "Transform Probability");
  } else if (converged_param_type_ == ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
    is_ok_converged_param = validate_score(
      nearest_voxel_transformation_likelihood,
      converged_param_nearest_voxel_transformation_likelihood_,
      "Nearest Voxel Transformation Likelihood");
  } else {
    is_ok_converged_param = false;
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1, "Unknown converged param type.");
  }
  return is_ok_converged_param;
}

std::optional<Eigen::Matrix4f> NDTScanMatcher::interpolate_regularization_pose(
  const rclcpp::Time & sensor_ros_time)
{
  if (regularization_pose_msg_ptr_array_.empty()) {
    return std::nullopt;
  }

  // synchronization
  PoseArrayInterpolator interpolator(this, sensor_ros_time, regularization_pose_msg_ptr_array_);

  pop_old_pose(regularization_pose_msg_ptr_array_, sensor_ros_time);

  // if the interpolate_pose fails, 0.0 is stored in the stamp
  if (rclcpp::Time(interpolator.get_current_pose().header.stamp).seconds() == 0.0) {
    return std::nullopt;
  }

  return pose_to_matrix4f(interpolator.get_current_pose().pose.pose);
}

void NDTScanMatcher::add_regularization_pose(const rclcpp::Time & sensor_ros_time)
{
  ndt_ptr_->unsetRegularizationPose();
  std::optional<Eigen::Matrix4f> pose_opt = interpolate_regularization_pose(sensor_ros_time);
  if (pose_opt.has_value()) {
    ndt_ptr_->setRegularizationPose(pose_opt.value());
    RCLCPP_DEBUG_STREAM(get_logger(), "Regularization pose is set to NDT");
  }
}

void NDTScanMatcher::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  is_activated_ = req->data;
  if (is_activated_) {
    std::lock_guard<std::mutex> initial_pose_array_lock(initial_pose_array_mtx_);
    initial_pose_msg_ptr_array_.clear();
  } else {
    (*state_ptr_)["state"] = "Initializing";
  }
  res->success = true;
  return;
}
