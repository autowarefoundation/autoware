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

#include "localization_util/matrix_type.hpp"
#include "localization_util/util_func.hpp"
#include "ndt_scan_matcher/particle.hpp"
#include "tree_structured_parzen_estimator/tree_structured_parzen_estimator.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/transform/transforms.hpp>

#include <boost/math/special_functions/erf.hpp>

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

Eigen::Matrix2d find_rotation_matrix_aligning_covariance_to_principal_axes(
  const Eigen::Matrix2d & matrix)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(matrix);
  if (eigensolver.info() == Eigen::Success) {
    const Eigen::Vector2d eigen_vec = eigensolver.eigenvectors().col(0);
    const double th = std::atan2(eigen_vec.y(), eigen_vec.x());
    return Eigen::Rotation2Dd(th).toRotationMatrix();
  }
  throw std::runtime_error("Eigen solver failed. Return output_pose_covariance value.");
}

// cspell: ignore degrounded
NDTScanMatcher::NDTScanMatcher()
: Node("ndt_scan_matcher"),
  tf2_broadcaster_(*this),
  ndt_ptr_(new NormalDistributionsTransform),
  state_ptr_(new std::map<std::string, std::string>),
  output_pose_covariance_({}),
  regularization_enabled_(declare_parameter<bool>("regularization_enabled")),
  is_activated_(false)
{
  (*state_ptr_)["state"] = "Initializing";

  int64_t points_queue_size = this->declare_parameter<int64_t>("input_sensor_points_queue_size");
  points_queue_size = std::max(points_queue_size, (int64_t)0);
  RCLCPP_INFO(get_logger(), "points_queue_size: %ld", points_queue_size);

  base_frame_ = this->declare_parameter<std::string>("base_frame");
  RCLCPP_INFO(get_logger(), "base_frame_id: %s", base_frame_.c_str());

  ndt_base_frame_ = this->declare_parameter<std::string>("ndt_base_frame");
  RCLCPP_INFO(get_logger(), "ndt_base_frame_id: %s", ndt_base_frame_.c_str());

  map_frame_ = this->declare_parameter<std::string>("map_frame");
  RCLCPP_INFO(get_logger(), "map_frame_id: %s", map_frame_.c_str());

  pclomp::NdtParams ndt_params{};
  ndt_params.trans_epsilon = this->declare_parameter<double>("trans_epsilon");
  ndt_params.step_size = this->declare_parameter<double>("step_size");
  ndt_params.resolution = this->declare_parameter<double>("resolution");
  ndt_params.max_iterations = static_cast<int>(this->declare_parameter<int64_t>("max_iterations"));
  ndt_params.num_threads = static_cast<int>(this->declare_parameter<int64_t>("num_threads"));
  ndt_params.num_threads = std::max(ndt_params.num_threads, 1);
  ndt_params.regularization_scale_factor =
    static_cast<float>(this->declare_parameter<float>("regularization_scale_factor"));
  ndt_ptr_->setParams(ndt_params);

  RCLCPP_INFO(
    get_logger(), "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d",
    ndt_params.trans_epsilon, ndt_params.step_size, ndt_params.resolution,
    ndt_params.max_iterations);

  const int64_t converged_param_type_tmp = this->declare_parameter<int64_t>("converged_param_type");
  converged_param_type_ = static_cast<ConvergedParamType>(converged_param_type_tmp);

  converged_param_transform_probability_ =
    this->declare_parameter<double>("converged_param_transform_probability");
  converged_param_nearest_voxel_transformation_likelihood_ =
    this->declare_parameter<double>("converged_param_nearest_voxel_transformation_likelihood");

  lidar_topic_timeout_sec_ = this->declare_parameter<double>("lidar_topic_timeout_sec");

  critical_upper_bound_exe_time_ms_ =
    this->declare_parameter<int64_t>("critical_upper_bound_exe_time_ms");

  initial_pose_timeout_sec_ = this->declare_parameter<double>("initial_pose_timeout_sec");

  initial_pose_distance_tolerance_m_ =
    this->declare_parameter<double>("initial_pose_distance_tolerance_m");

  initial_pose_buffer_ = std::make_unique<SmartPoseBuffer>(
    this->get_logger(), initial_pose_timeout_sec_, initial_pose_distance_tolerance_m_);

  use_cov_estimation_ = this->declare_parameter<bool>("use_covariance_estimation");
  if (use_cov_estimation_) {
    std::vector<double> initial_pose_offset_model_x =
      this->declare_parameter<std::vector<double>>("initial_pose_offset_model_x");
    std::vector<double> initial_pose_offset_model_y =
      this->declare_parameter<std::vector<double>>("initial_pose_offset_model_y");

    if (initial_pose_offset_model_x.size() == initial_pose_offset_model_y.size()) {
      const size_t size = initial_pose_offset_model_x.size();
      initial_pose_offset_model_.resize(size);
      for (size_t i = 0; i < size; i++) {
        initial_pose_offset_model_[i].x() = initial_pose_offset_model_x[i];
        initial_pose_offset_model_[i].y() = initial_pose_offset_model_y[i];
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Invalid initial pose offset model parameters. Disable covariance estimation.");
      use_cov_estimation_ = false;
    }
  }

  std::vector<double> output_pose_covariance =
    this->declare_parameter<std::vector<double>>("output_pose_covariance");
  for (std::size_t i = 0; i < output_pose_covariance.size(); ++i) {
    output_pose_covariance_[i] = output_pose_covariance[i];
  }

  initial_estimate_particles_num_ =
    this->declare_parameter<int64_t>("initial_estimate_particles_num");
  n_startup_trials_ = this->declare_parameter<int64_t>("n_startup_trials");

  estimate_scores_for_degrounded_scan_ =
    this->declare_parameter<bool>("estimate_scores_for_degrounded_scan");

  z_margin_for_ground_removal_ = this->declare_parameter<double>("z_margin_for_ground_removal");

  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = initial_pose_callback_group;
  auto sensor_sub_opt = rclcpp::SubscriptionOptions();
  sensor_sub_opt.callback_group = sensor_callback_group;

  constexpr double map_update_dt = 1.0;
  constexpr auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(map_update_dt));
  map_update_timer_ = rclcpp::create_timer(
    this, this->get_clock(), period_ns, std::bind(&NDTScanMatcher::callback_timer, this),
    timer_callback_group_);
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", 100,
    std::bind(&NDTScanMatcher::callback_initial_pose, this, std::placeholders::_1),
    initial_pose_sub_opt);
  sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(points_queue_size),
    std::bind(&NDTScanMatcher::callback_sensor_points, this, std::placeholders::_1),
    sensor_sub_opt);

  // Only if regularization is enabled, subscribe to the regularization base pose
  if (regularization_enabled_) {
    // NOTE: The reason that the regularization subscriber does not belong to the
    // sensor_callback_group is to ensure that the regularization callback is called even if
    // sensor_callback takes long time to process.
    // Both callback_initial_pose and callback_regularization_pose must not miss receiving data for
    // proper interpolation.
    regularization_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "regularization_pose_with_covariance", 10,
        std::bind(&NDTScanMatcher::callback_regularization_pose, this, std::placeholders::_1),
        initial_pose_sub_opt);
    const double value_as_unlimited = 1000.0;
    regularization_pose_buffer_ =
      std::make_unique<SmartPoseBuffer>(this->get_logger(), value_as_unlimited, value_as_unlimited);
  }

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
  multi_ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("multi_ndt_pose", 10);
  multi_initial_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("multi_initial_pose", 10);
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
  initial_to_result_relative_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("initial_to_result_relative_pose", 10);
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
  ndt_monte_carlo_initial_pose_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "monte_carlo_initial_pose_marker", 10);

  service_ = this->create_service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>(
    "ndt_align_srv",
    std::bind(
      &NDTScanMatcher::service_ndt_align, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), sensor_callback_group);
  service_trigger_node_ = this->create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &NDTScanMatcher::service_trigger_node, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), sensor_callback_group);

  tf2_listener_module_ = std::make_shared<Tf2ListenerModule>(this);

  use_dynamic_map_loading_ = this->declare_parameter<bool>("use_dynamic_map_loading");
  if (use_dynamic_map_loading_) {
    map_update_module_ = std::make_unique<MapUpdateModule>(this, &ndt_ptr_mtx_, ndt_ptr_);
  } else {
    map_module_ = std::make_unique<MapModule>(this, &ndt_ptr_mtx_, ndt_ptr_, sensor_callback_group);
  }

  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);
}

void NDTScanMatcher::publish_diagnostic()
{
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
    state_ptr_->count("lidar_topic_delay_time_sec") &&
    std::stod((*state_ptr_)["lidar_topic_delay_time_sec"]) > lidar_topic_timeout_sec_) {
    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag_status_msg.message += "lidar_topic_delay_time_sec exceed limit. ";
  }
  if (
    state_ptr_->count("skipping_publish_num") &&
    std::stoi((*state_ptr_)["skipping_publish_num"]) > 1 &&
    std::stoi((*state_ptr_)["skipping_publish_num"]) < 5) {
    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag_status_msg.message += "skipping_publish_num > 1. ";
  }
  if (
    state_ptr_->count("skipping_publish_num") &&
    std::stoi((*state_ptr_)["skipping_publish_num"]) >= 5) {
    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_status_msg.message += "skipping_publish_num exceed limit. ";
  }
  if (
    state_ptr_->count("nearest_voxel_transformation_likelihood") &&
    std::stod((*state_ptr_)["nearest_voxel_transformation_likelihood"]) <
      converged_param_nearest_voxel_transformation_likelihood_) {
    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag_status_msg.message += "NDT score is unreliably low. ";
  }
  if (
    state_ptr_->count("execution_time") &&
    std::stod((*state_ptr_)["execution_time"]) >=
      static_cast<double>(critical_upper_bound_exe_time_ms_)) {
    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag_status_msg.message +=
      "NDT exe time is too long. (took " + (*state_ptr_)["execution_time"] + " [ms])";
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
}

void NDTScanMatcher::callback_timer()
{
  if (!is_activated_) {
    return;
  }
  if (!use_dynamic_map_loading_) {
    return;
  }
  std::lock_guard<std::mutex> lock(latest_ekf_position_mtx_);
  if (latest_ekf_position_ == std::nullopt) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Cannot find the reference position for map update. Please check if the EKF odometry is "
      "provided to NDT.");
    return;
  }
  // continue only if we should update the map
  if (map_update_module_->should_update_map(latest_ekf_position_.value())) {
    RCLCPP_INFO(this->get_logger(), "Start updating NDT map (timer_callback)");
    map_update_module_->update_map(latest_ekf_position_.value());
  }
}

void NDTScanMatcher::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  if (!is_activated_) return;

  if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
    initial_pose_buffer_->push_back(initial_pose_msg_ptr);
  } else {
    RCLCPP_ERROR_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "Received initial pose message with frame_id "
        << initial_pose_msg_ptr->header.frame_id << ", but expected " << map_frame_
        << ". Please check the frame_id in the input topic and ensure it is correct.");
  }

  if (use_dynamic_map_loading_) {
    std::lock_guard<std::mutex> lock(latest_ekf_position_mtx_);
    latest_ekf_position_ = initial_pose_msg_ptr->pose.pose.position;
  }
}

void NDTScanMatcher::callback_regularization_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr)
{
  regularization_pose_buffer_->push_back(pose_conv_msg_ptr);
}

void NDTScanMatcher::callback_sensor_points(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame)
{
  if (sensor_points_msg_in_sensor_frame->data.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "Empty sensor points!");
    return;
  }

  const rclcpp::Time sensor_ros_time = sensor_points_msg_in_sensor_frame->header.stamp;
  const double lidar_topic_delay_time_sec = (this->now() - sensor_ros_time).seconds();
  (*state_ptr_)["lidar_topic_delay_time_sec"] = std::to_string(lidar_topic_delay_time_sec);

  if (lidar_topic_delay_time_sec > lidar_topic_timeout_sec_) {
    RCLCPP_WARN(
      this->get_logger(),
      "The LiDAR topic is experiencing latency. The delay time is %lf[sec] (the tolerance is "
      "%lf[sec])",
      lidar_topic_delay_time_sec, lidar_topic_timeout_sec_);

    // If the delay time of the LiDAR topic exceeds the delay compensation time of ekf_localizer,
    // even if further processing continues, the estimated result will be rejected by ekf_localizer.
    // Therefore, it would be acceptable to exit the function here.
    // However, for now, we will continue the processing as it is.

    // return;
  }

  // mutex ndt_ptr_
  std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);

  const auto exe_start_time = std::chrono::system_clock::now();

  // preprocess input pointcloud
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_sensor_frame(
    new pcl::PointCloud<PointSource>);
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame(
    new pcl::PointCloud<PointSource>);
  const std::string & sensor_frame = sensor_points_msg_in_sensor_frame->header.frame_id;

  pcl::fromROSMsg(*sensor_points_msg_in_sensor_frame, *sensor_points_in_sensor_frame);
  transform_sensor_measurement(
    sensor_frame, base_frame_, sensor_points_in_sensor_frame, sensor_points_in_baselink_frame);
  ndt_ptr_->setInputSource(sensor_points_in_baselink_frame);
  if (!is_activated_) return;

  // calculate initial pose
  std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
    initial_pose_buffer_->interpolate(sensor_ros_time);
  if (!interpolation_result_opt) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "No interpolated pose!");
    return;
  }
  initial_pose_buffer_->pop_old(sensor_ros_time);
  const SmartPoseBuffer::InterpolateResult & interpolation_result =
    interpolation_result_opt.value();

  // if regularization is enabled and available, set pose to NDT for regularization
  if (regularization_enabled_) {
    add_regularization_pose(sensor_ros_time);
  }

  if (ndt_ptr_->getInputTarget() == nullptr) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "No MAP!");
    return;
  }

  // perform ndt scan matching
  const Eigen::Matrix4f initial_pose_matrix =
    pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  ndt_ptr_->align(*output_cloud, initial_pose_matrix);
  const pclomp::NdtResult ndt_result = ndt_ptr_->getResult();

  const geometry_msgs::msg::Pose result_pose_msg = matrix4f_to_pose(ndt_result.pose);
  std::vector<geometry_msgs::msg::Pose> transformation_msg_array;
  for (const auto & pose_matrix : ndt_result.transformation_array) {
    geometry_msgs::msg::Pose pose_ros = matrix4f_to_pose(pose_matrix);
    transformation_msg_array.push_back(pose_ros);
  }

  // perform several validations
  bool is_ok_iteration_num =
    validate_num_iteration(ndt_result.iteration_num, ndt_ptr_->getMaximumIterations());
  const int oscillation_num = count_oscillation(transformation_msg_array);
  bool is_local_optimal_solution_oscillation = false;
  if (!is_ok_iteration_num) {
    constexpr int oscillation_threshold = 10;
    is_local_optimal_solution_oscillation = (oscillation_num > oscillation_threshold);
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

  // covariance estimation
  std::array<double, 36> ndt_covariance = output_pose_covariance_;
  if (is_converged && use_cov_estimation_) {
    const auto estimated_covariance =
      estimate_covariance(ndt_result, initial_pose_matrix, sensor_ros_time);
    ndt_covariance = estimated_covariance;
  }

  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  const auto exe_time = static_cast<float>(duration_micro_sec) / 1000.0f;

  // publish
  initial_pose_with_covariance_pub_->publish(interpolation_result.interpolated_pose);
  exe_time_pub_->publish(make_float32_stamped(sensor_ros_time, exe_time));
  transform_probability_pub_->publish(
    make_float32_stamped(sensor_ros_time, ndt_result.transform_probability));
  nearest_voxel_transformation_likelihood_pub_->publish(
    make_float32_stamped(sensor_ros_time, ndt_result.nearest_voxel_transformation_likelihood));
  iteration_num_pub_->publish(make_int32_stamped(sensor_ros_time, ndt_result.iteration_num));
  publish_tf(sensor_ros_time, result_pose_msg);
  publish_pose(sensor_ros_time, result_pose_msg, ndt_covariance, is_converged);
  publish_marker(sensor_ros_time, transformation_msg_array);
  publish_initial_to_result(
    sensor_ros_time, result_pose_msg, interpolation_result.interpolated_pose,
    interpolation_result.old_pose, interpolation_result.new_pose);

  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_map_ptr(
    new pcl::PointCloud<PointSource>);
  tier4_autoware_utils::transformPointCloud(
    *sensor_points_in_baselink_frame, *sensor_points_in_map_ptr, ndt_result.pose);
  publish_point_cloud(sensor_ros_time, map_frame_, sensor_points_in_map_ptr);

  // whether use de-grounded points calculate score
  if (estimate_scores_for_degrounded_scan_) {
    // remove ground
    pcl::shared_ptr<pcl::PointCloud<PointSource>> no_ground_points_in_map_ptr(
      new pcl::PointCloud<PointSource>);
    for (std::size_t i = 0; i < sensor_points_in_map_ptr->size(); i++) {
      const float point_z = sensor_points_in_map_ptr->points[i].z;  // NOLINT
      if (point_z - matrix4f_to_pose(ndt_result.pose).position.z > z_margin_for_ground_removal_) {
        no_ground_points_in_map_ptr->points.push_back(sensor_points_in_map_ptr->points[i]);
      }
    }
    // pub remove-ground points
    sensor_msgs::msg::PointCloud2 no_ground_points_msg_in_map;
    pcl::toROSMsg(*no_ground_points_in_map_ptr, no_ground_points_msg_in_map);
    no_ground_points_msg_in_map.header.stamp = sensor_ros_time;
    no_ground_points_msg_in_map.header.frame_id = map_frame_;
    no_ground_points_aligned_pose_pub_->publish(no_ground_points_msg_in_map);
    // calculate score
    const auto no_ground_transform_probability = static_cast<float>(
      ndt_ptr_->calculateTransformationProbability(*no_ground_points_in_map_ptr));
    const auto no_ground_nearest_voxel_transformation_likelihood = static_cast<float>(
      ndt_ptr_->calculateNearestVoxelTransformationLikelihood(*no_ground_points_in_map_ptr));
    // pub score
    no_ground_transform_probability_pub_->publish(
      make_float32_stamped(sensor_ros_time, no_ground_transform_probability));
    no_ground_nearest_voxel_transformation_likelihood_pub_->publish(
      make_float32_stamped(sensor_ros_time, no_ground_nearest_voxel_transformation_likelihood));
  }

  (*state_ptr_)["state"] = "Aligned";
  (*state_ptr_)["transform_probability"] = std::to_string(ndt_result.transform_probability);
  (*state_ptr_)["nearest_voxel_transformation_likelihood"] =
    std::to_string(ndt_result.nearest_voxel_transformation_likelihood);
  (*state_ptr_)["iteration_num"] = std::to_string(ndt_result.iteration_num);
  (*state_ptr_)["skipping_publish_num"] = std::to_string(skipping_publish_num);
  (*state_ptr_)["oscillation_count"] = std::to_string(oscillation_num);
  (*state_ptr_)["is_local_optimal_solution_oscillation"] =
    std::to_string(is_local_optimal_solution_oscillation);
  (*state_ptr_)["execution_time"] = std::to_string(exe_time);

  publish_diagnostic();
}

void NDTScanMatcher::transform_sensor_measurement(
  const std::string & source_frame, const std::string & target_frame,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_input_ptr,
  pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_output_ptr)
{
  auto tf_target_to_source_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  tf2_listener_module_->get_transform(
    this->now(), target_frame, source_frame, tf_target_to_source_ptr);
  const geometry_msgs::msg::PoseStamped target_to_source_pose_stamped =
    tier4_autoware_utils::transform2pose(*tf_target_to_source_ptr);
  const Eigen::Matrix4f base_to_sensor_matrix =
    pose_to_matrix4f(target_to_source_pose_stamped.pose);
  tier4_autoware_utils::transformPointCloud(
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
  const std::array<double, 36> & ndt_covariance, const bool is_converged)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;
  result_pose_with_cov_msg.pose.covariance = ndt_covariance;

  if (is_converged) {
    ndt_pose_pub_->publish(result_pose_stamped_msg);
    ndt_pose_with_covariance_pub_->publish(result_pose_with_cov_msg);
  }
}

void NDTScanMatcher::publish_point_cloud(
  const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr)
{
  sensor_msgs::msg::PointCloud2 sensor_points_msg_in_map;
  pcl::toROSMsg(*sensor_points_in_map_ptr, sensor_points_msg_in_map);
  sensor_points_msg_in_map.header.stamp = sensor_ros_time;
  sensor_points_msg_in_map.header.frame_id = frame_id;
  sensor_aligned_pose_pub_->publish(sensor_points_msg_in_map);
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

void NDTScanMatcher::publish_initial_to_result(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg)
{
  geometry_msgs::msg::PoseStamped initial_to_result_relative_pose_stamped;
  initial_to_result_relative_pose_stamped.pose =
    tier4_autoware_utils::inverseTransformPose(result_pose_msg, initial_pose_cov_msg.pose.pose);
  initial_to_result_relative_pose_stamped.header.stamp = sensor_ros_time;
  initial_to_result_relative_pose_stamped.header.frame_id = map_frame_;
  initial_to_result_relative_pose_pub_->publish(initial_to_result_relative_pose_stamped);

  const auto initial_to_result_distance =
    static_cast<float>(norm(initial_pose_cov_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance));

  const auto initial_to_result_distance_old =
    static_cast<float>(norm(initial_pose_old_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_old_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance_old));

  const auto initial_to_result_distance_new =
    static_cast<float>(norm(initial_pose_new_msg.pose.pose.position, result_pose_msg.position));
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
  const double score, const double score_threshold, const std::string & score_name)
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

int NDTScanMatcher::count_oscillation(
  const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array)
{
  constexpr double inversion_vector_threshold = -0.9;

  int oscillation_cnt = 0;
  int max_oscillation_cnt = 0;

  for (size_t i = 2; i < result_pose_msg_array.size(); ++i) {
    const Eigen::Vector3d current_pose = point_to_vector3d(result_pose_msg_array.at(i).position);
    const Eigen::Vector3d prev_pose = point_to_vector3d(result_pose_msg_array.at(i - 1).position);
    const Eigen::Vector3d prev_prev_pose =
      point_to_vector3d(result_pose_msg_array.at(i - 2).position);
    const auto current_vec = (current_pose - prev_pose).normalized();
    const auto prev_vec = (prev_pose - prev_prev_pose).normalized();
    const double cosine_value = current_vec.dot(prev_vec);
    const bool oscillation = cosine_value < inversion_vector_threshold;
    if (oscillation) {
      oscillation_cnt++;  // count consecutive oscillation
    } else {
      oscillation_cnt = 0;  // reset
    }
    max_oscillation_cnt = std::max(max_oscillation_cnt, oscillation_cnt);
  }
  return max_oscillation_cnt;
}

std::array<double, 36> NDTScanMatcher::estimate_covariance(
  const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
  const rclcpp::Time & sensor_ros_time)
{
  Eigen::Matrix2d rot = Eigen::Matrix2d::Identity();
  try {
    rot = find_rotation_matrix_aligning_covariance_to_principal_axes(
      ndt_result.hessian.inverse().block(0, 0, 2, 2));
  } catch (const std::exception & e) {
    RCLCPP_WARN(get_logger(), "Error in Eigen solver: %s", e.what());
    return output_pose_covariance_;
  }

  // first result is added to mean
  const int n = static_cast<int>(initial_pose_offset_model_.size()) + 1;
  const Eigen::Vector2d ndt_pose_2d(ndt_result.pose(0, 3), ndt_result.pose(1, 3));
  Eigen::Vector2d mean = ndt_pose_2d;
  std::vector<Eigen::Vector2d> ndt_pose_2d_vec;
  ndt_pose_2d_vec.reserve(n);
  ndt_pose_2d_vec.emplace_back(ndt_pose_2d);

  geometry_msgs::msg::PoseArray multi_ndt_result_msg;
  geometry_msgs::msg::PoseArray multi_initial_pose_msg;
  multi_ndt_result_msg.header.stamp = sensor_ros_time;
  multi_ndt_result_msg.header.frame_id = map_frame_;
  multi_initial_pose_msg.header.stamp = sensor_ros_time;
  multi_initial_pose_msg.header.frame_id = map_frame_;
  multi_ndt_result_msg.poses.push_back(matrix4f_to_pose(ndt_result.pose));
  multi_initial_pose_msg.poses.push_back(matrix4f_to_pose(initial_pose_matrix));

  // multiple searches
  for (const auto & pose_offset : initial_pose_offset_model_) {
    const Eigen::Vector2d rotated_pose_offset_2d = rot * pose_offset;

    Eigen::Matrix4f sub_initial_pose_matrix(Eigen::Matrix4f::Identity());
    sub_initial_pose_matrix = ndt_result.pose;
    sub_initial_pose_matrix(0, 3) += static_cast<float>(rotated_pose_offset_2d.x());
    sub_initial_pose_matrix(1, 3) += static_cast<float>(rotated_pose_offset_2d.y());

    auto sub_output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
    ndt_ptr_->align(*sub_output_cloud, sub_initial_pose_matrix);
    const Eigen::Matrix4f sub_ndt_result = ndt_ptr_->getResult().pose;

    const Eigen::Vector2d sub_ndt_pose_2d = sub_ndt_result.topRightCorner<2, 1>().cast<double>();
    mean += sub_ndt_pose_2d;
    ndt_pose_2d_vec.emplace_back(sub_ndt_pose_2d);

    multi_ndt_result_msg.poses.push_back(matrix4f_to_pose(sub_ndt_result));
    multi_initial_pose_msg.poses.push_back(matrix4f_to_pose(sub_initial_pose_matrix));
  }

  // calculate the covariance matrix
  mean /= n;
  Eigen::Matrix2d pca_covariance = Eigen::Matrix2d::Zero();
  for (const auto & temp_ndt_pose_2d : ndt_pose_2d_vec) {
    const Eigen::Vector2d diff_2d = temp_ndt_pose_2d - mean;
    pca_covariance += diff_2d * diff_2d.transpose();
  }
  pca_covariance /= (n - 1);  // unbiased covariance

  std::array<double, 36> ndt_covariance = output_pose_covariance_;
  ndt_covariance[0 + 6 * 0] += pca_covariance(0, 0);
  ndt_covariance[1 + 6 * 0] += pca_covariance(1, 0);
  ndt_covariance[0 + 6 * 1] += pca_covariance(0, 1);
  ndt_covariance[1 + 6 * 1] += pca_covariance(1, 1);

  multi_ndt_pose_pub_->publish(multi_ndt_result_msg);
  multi_initial_pose_pub_->publish(multi_initial_pose_msg);

  return ndt_covariance;
}

void NDTScanMatcher::add_regularization_pose(const rclcpp::Time & sensor_ros_time)
{
  ndt_ptr_->unsetRegularizationPose();
  std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
    regularization_pose_buffer_->interpolate(sensor_ros_time);
  if (!interpolation_result_opt) {
    return;
  }
  regularization_pose_buffer_->pop_old(sensor_ros_time);
  const SmartPoseBuffer::InterpolateResult & interpolation_result =
    interpolation_result_opt.value();
  const Eigen::Matrix4f pose = pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
  ndt_ptr_->setRegularizationPose(pose);
  RCLCPP_DEBUG_STREAM(get_logger(), "Regularization pose is set to NDT");
}

void NDTScanMatcher::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  is_activated_ = req->data;
  if (is_activated_) {
    initial_pose_buffer_->clear();
  }
  res->success = true;
}

void NDTScanMatcher::service_ndt_align(
  const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  // get TF from pose_frame to map_frame
  auto tf_pose_to_map_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  tf2_listener_module_->get_transform(
    get_clock()->now(), map_frame_, req->pose_with_covariance.header.frame_id, tf_pose_to_map_ptr);

  // transform pose_frame to map_frame
  const auto initial_pose_msg_in_map_frame =
    transform(req->pose_with_covariance, *tf_pose_to_map_ptr);
  if (use_dynamic_map_loading_) {
    map_update_module_->update_map(initial_pose_msg_in_map_frame.pose.pose.position);
  }

  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_ptr_mtx_);

  if (ndt_ptr_->getInputTarget() == nullptr) {
    res->success = false;
    RCLCPP_WARN(
      get_logger(), "No InputTarget. Please check the map file and the map_loader service");
    return;
  }

  if (ndt_ptr_->getInputSource() == nullptr) {
    res->success = false;
    RCLCPP_WARN(get_logger(), "No InputSource. Please check the input lidar topic");
    return;
  }

  res->pose_with_covariance = align_pose(initial_pose_msg_in_map_frame);
  res->success = true;
  res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;
}

geometry_msgs::msg::PoseWithCovarianceStamped NDTScanMatcher::align_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov)
{
  output_pose_with_cov_to_log(get_logger(), "align_pose_input", initial_pose_with_cov);

  const auto base_rpy = get_rpy(initial_pose_with_cov);
  const Eigen::Map<const RowMatrixXd> covariance = {
    initial_pose_with_cov.pose.covariance.data(), 6, 6};
  const double stddev_x = std::sqrt(covariance(0, 0));
  const double stddev_y = std::sqrt(covariance(1, 1));
  const double stddev_z = std::sqrt(covariance(2, 2));
  const double stddev_roll = std::sqrt(covariance(3, 3));
  const double stddev_pitch = std::sqrt(covariance(4, 4));

  // Let phi be the cumulative distribution function of the standard normal distribution.
  // It has the following relationship with the error function (erf).
  //   phi(x) = 1/2 (1 + erf(x / sqrt(2)))
  // so, 2 * phi(x) - 1 = erf(x / sqrt(2)).
  // The range taken by 2 * phi(x) - 1 is [-1, 1], so it can be used as a uniform distribution in
  // TPE. Let u = 2 * phi(x) - 1, then x = sqrt(2) * erf_inv(u). Computationally, it is not a good
  // to give erf_inv -1 and 1, so it is rounded off at (-1 + eps, 1 - eps).
  const double sqrt2 = std::sqrt(2);
  auto uniform_to_normal = [&sqrt2](const double uniform) {
    assert(-1.0 <= uniform && uniform <= 1.0);
    constexpr double epsilon = 1.0e-6;
    const double clamped = std::clamp(uniform, -1.0 + epsilon, 1.0 - epsilon);
    return boost::math::erf_inv(clamped) * sqrt2;
  };

  auto normal_to_uniform = [&sqrt2](const double normal) {
    return boost::math::erf(normal / sqrt2);
  };

  // Optimizing (x, y, z, roll, pitch, yaw) 6 dimensions.
  // The last dimension (yaw) is a loop variable.
  // Although roll and pitch are also angles, they are considered non-looping variables that follow
  // a normal distribution with a small standard deviation. This assumes that the initial pose of
  // the ego vehicle is aligned with the ground to some extent about roll and pitch.
  const std::vector<bool> is_loop_variable = {false, false, false, false, false, true};
  TreeStructuredParzenEstimator tpe(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE, n_startup_trials_, is_loop_variable);

  std::vector<Particle> particle_array;
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();

  // publish the estimated poses in 20 times to see the progress and to avoid dropping data
  visualization_msgs::msg::MarkerArray marker_array;
  constexpr int64_t publish_num = 20;
  const int64_t publish_interval = initial_estimate_particles_num_ / publish_num;

  for (int64_t i = 0; i < initial_estimate_particles_num_; i++) {
    const TreeStructuredParzenEstimator::Input input = tpe.get_next_input();

    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x =
      initial_pose_with_cov.pose.pose.position.x + uniform_to_normal(input[0]) * stddev_x;
    initial_pose.position.y =
      initial_pose_with_cov.pose.pose.position.y + uniform_to_normal(input[1]) * stddev_y;
    initial_pose.position.z =
      initial_pose_with_cov.pose.pose.position.z + uniform_to_normal(input[2]) * stddev_z;
    geometry_msgs::msg::Vector3 init_rpy;
    init_rpy.x = base_rpy.x + uniform_to_normal(input[3]) * stddev_roll;
    init_rpy.y = base_rpy.y + uniform_to_normal(input[4]) * stddev_pitch;
    init_rpy.z = base_rpy.z + input[5] * M_PI;
    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(init_rpy.x, init_rpy.y, init_rpy.z);
    initial_pose.orientation = tf2::toMsg(tf_quaternion);

    const Eigen::Matrix4f initial_pose_matrix = pose_to_matrix4f(initial_pose);
    ndt_ptr_->align(*output_cloud, initial_pose_matrix);
    const pclomp::NdtResult ndt_result = ndt_ptr_->getResult();

    Particle particle(
      initial_pose, matrix4f_to_pose(ndt_result.pose), ndt_result.transform_probability,
      ndt_result.iteration_num);
    particle_array.push_back(particle);
    push_debug_markers(marker_array, get_clock()->now(), map_frame_, particle, i);
    if ((i + 1) % publish_interval == 0 || (i + 1) == initial_estimate_particles_num_) {
      ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);
      marker_array.markers.clear();
    }

    const geometry_msgs::msg::Pose pose = matrix4f_to_pose(ndt_result.pose);
    const geometry_msgs::msg::Vector3 rpy = get_rpy(pose);

    const double diff_x = pose.position.x - initial_pose_with_cov.pose.pose.position.x;
    const double diff_y = pose.position.y - initial_pose_with_cov.pose.pose.position.y;
    const double diff_z = pose.position.z - initial_pose_with_cov.pose.pose.position.z;
    const double diff_roll = rpy.x - base_rpy.x;
    const double diff_pitch = rpy.y - base_rpy.y;
    const double diff_yaw = rpy.z - base_rpy.z;

    // Only yaw is a loop_variable, so only simple normalization is performed.
    // All other variables are converted from normal distribution to uniform distribution.
    TreeStructuredParzenEstimator::Input result(is_loop_variable.size());
    result[0] = normal_to_uniform(diff_x / stddev_x);
    result[1] = normal_to_uniform(diff_y / stddev_y);
    result[2] = normal_to_uniform(diff_z / stddev_z);
    result[3] = normal_to_uniform(diff_roll / stddev_roll);
    result[4] = normal_to_uniform(diff_pitch / stddev_pitch);
    result[5] = diff_yaw / M_PI;
    tpe.add_trial(TreeStructuredParzenEstimator::Trial{result, ndt_result.transform_probability});

    auto sensor_points_in_map_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    tier4_autoware_utils::transformPointCloud(
      *ndt_ptr_->getInputSource(), *sensor_points_in_map_ptr, ndt_result.pose);
    publish_point_cloud(initial_pose_with_cov.header.stamp, map_frame_, sensor_points_in_map_ptr);
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = initial_pose_with_cov.header.stamp;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = best_particle_ptr->result_pose;

  output_pose_with_cov_to_log(get_logger(), "align_pose_output", result_pose_with_cov_msg);
  RCLCPP_INFO_STREAM(get_logger(), "best_score," << best_particle_ptr->score);

  return result_pose_with_cov_msg;
}
