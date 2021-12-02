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

#include "ndt_scan_matcher/debug.hpp"
#include "ndt_scan_matcher/matrix_type.hpp"
#include "ndt_scan_matcher/particle.hpp"
#include "ndt_scan_matcher/util_func.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <boost/shared_ptr.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <thread>

autoware_debug_msgs::msg::Float32Stamped makeFloat32Stamped(
  const builtin_interfaces::msg::Time & stamp, const float data)
{
  using T = autoware_debug_msgs::msg::Float32Stamped;
  return autoware_debug_msgs::build<T>().stamp(stamp).data(data);
}

autoware_debug_msgs::msg::Int32Stamped makeInt32Stamped(
  const builtin_interfaces::msg::Time & stamp, const int32_t data)
{
  using T = autoware_debug_msgs::msg::Int32Stamped;
  return autoware_debug_msgs::build<T>().stamp(stamp).data(data);
}

geometry_msgs::msg::TransformStamped identityTransformStamped(
  const builtin_interfaces::msg::Time & timestamp, const std::string & header_frame_id,
  const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = timestamp;
  transform.header.frame_id = header_frame_id;
  transform.child_frame_id = child_frame_id;
  transform.transform.rotation = autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
  transform.transform.translation = autoware_utils::createTranslation(0.0, 0.0, 0.0);
  return transform;
}

double norm(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return std::sqrt(
    std::pow(p1.x - p2.x, 2.0) + std::pow(p1.y - p2.y, 2.0) + std::pow(p1.z - p2.z, 2.0));
}

bool isLocalOptimalSolutionOscillation(
  const std::vector<Eigen::Matrix4f> & result_pose_matrix_array, const float oscillation_threshold,
  const float inversion_vector_threshold)
{
  bool prev_oscillation = false;
  int oscillation_cnt = 0;
  for (size_t i = 2; i < result_pose_matrix_array.size(); ++i) {
    const Eigen::Vector3f current_pose = result_pose_matrix_array.at(i).block(0, 3, 3, 1);
    const Eigen::Vector3f prev_pose = result_pose_matrix_array.at(i - 1).block(0, 3, 3, 1);
    const Eigen::Vector3f prev_prev_pose = result_pose_matrix_array.at(i - 2).block(0, 3, 3, 1);
    const auto current_vec = (current_pose - prev_pose).normalized();
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

NDTScanMatcher::NDTScanMatcher()
: Node("ndt_scan_matcher"),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_),
  tf2_broadcaster_(*this),
  ndt_implement_type_(NDTImplementType::PCL_GENERIC),
  base_frame_("base_link"),
  ndt_base_frame_("ndt_base_link"),
  map_frame_("map"),
  converged_param_transform_probability_(4.5),
  inversion_vector_threshold_(-0.9),
  oscillation_threshold_(10)
{
  key_value_stdmap_["state"] = "Initializing";

  int ndt_implement_type_tmp = this->declare_parameter("ndt_implement_type", 0);
  ndt_implement_type_ = static_cast<NDTImplementType>(ndt_implement_type_tmp);

  RCLCPP_INFO(get_logger(), "NDT Implement Type is %d", ndt_implement_type_tmp);
  try {
    ndt_ptr_ = getNDT<PointSource, PointTarget>(ndt_implement_type_);
  } catch (std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return;
  }

  if (ndt_implement_type_ == NDTImplementType::OMP) {
    using T = NormalDistributionsTransformOMP<PointSource, PointTarget>;

    // FIXME(IshitaTakeshi) Not sure if this is safe
    std::shared_ptr<T> ndt_omp_ptr = std::dynamic_pointer_cast<T>(ndt_ptr_);
    int search_method = static_cast<int>(omp_params_.search_method);
    search_method = this->declare_parameter("omp_neighborhood_search_method", search_method);
    omp_params_.search_method = static_cast<pclomp::NeighborSearchMethod>(search_method);
    // TODO(Tier IV): check search_method is valid value.
    ndt_omp_ptr->setNeighborhoodSearchMethod(omp_params_.search_method);

    omp_params_.num_threads = this->declare_parameter("omp_num_threads", omp_params_.num_threads);
    omp_params_.num_threads = std::max(omp_params_.num_threads, 1);
    ndt_omp_ptr->setNumThreads(omp_params_.num_threads);
    ndt_ptr_ = ndt_omp_ptr;
  }

  int points_queue_size = this->declare_parameter("input_sensor_points_queue_size", 0);
  points_queue_size = std::max(points_queue_size, 0);
  RCLCPP_INFO(get_logger(), "points_queue_size: %d", points_queue_size);

  base_frame_ = this->declare_parameter("base_frame", base_frame_);
  RCLCPP_INFO(get_logger(), "base_frame_id: %s", base_frame_.c_str());

  ndt_base_frame_ = this->declare_parameter("ndt_base_frame", ndt_base_frame_);
  RCLCPP_INFO(get_logger(), "ndt_base_frame_id: %s", ndt_base_frame_.c_str());

  double trans_epsilon = ndt_ptr_->getTransformationEpsilon();
  double step_size = ndt_ptr_->getStepSize();
  double resolution = ndt_ptr_->getResolution();
  int max_iterations = ndt_ptr_->getMaximumIterations();
  trans_epsilon = this->declare_parameter("trans_epsilon", trans_epsilon);
  step_size = this->declare_parameter("step_size", step_size);
  resolution = this->declare_parameter("resolution", resolution);
  max_iterations = this->declare_parameter("max_iterations", max_iterations);
  ndt_ptr_->setTransformationEpsilon(trans_epsilon);
  ndt_ptr_->setStepSize(step_size);
  ndt_ptr_->setResolution(resolution);
  ndt_ptr_->setMaximumIterations(max_iterations);
  RCLCPP_INFO(
    get_logger(), "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d",
    trans_epsilon, step_size, resolution, max_iterations);

  converged_param_transform_probability_ = this->declare_parameter(
    "converged_param_transform_probability", converged_param_transform_probability_);

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", 100,
    std::bind(&NDTScanMatcher::callbackInitialPose, this, std::placeholders::_1));
  map_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&NDTScanMatcher::callbackMapPoints, this, std::placeholders::_1));
  sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(points_queue_size),
    std::bind(&NDTScanMatcher::callbackSensorPoints, this, std::placeholders::_1));

  sensor_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned", 10);
  ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "ndt_pose_with_covariance", 10);
  initial_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_pose_with_covariance", 10);
  exe_time_pub_ =
    this->create_publisher<autoware_debug_msgs::msg::Float32Stamped>("exe_time_ms", 10);
  transform_probability_pub_ =
    this->create_publisher<autoware_debug_msgs::msg::Float32Stamped>("transform_probability", 10);
  iteration_num_pub_ =
    this->create_publisher<autoware_debug_msgs::msg::Int32Stamped>("iteration_num", 10);
  initial_to_result_distance_pub_ =
    this->create_publisher<autoware_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance", 10);
  initial_to_result_distance_old_pub_ =
    this->create_publisher<autoware_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_old", 10);
  initial_to_result_distance_new_pub_ =
    this->create_publisher<autoware_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_new", 10);
  ndt_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ndt_marker", 10);
  ndt_monte_carlo_initial_pose_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "monte_carlo_initial_pose_marker", 10);
  diagnostics_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  service_ = this->create_service<autoware_localization_msgs::srv::PoseWithCovarianceStamped>(
    "ndt_align_srv",
    std::bind(
      &NDTScanMatcher::serviceNDTAlign, this, std::placeholders::_1, std::placeholders::_2));

  diagnostic_thread_ = std::thread(&NDTScanMatcher::timerDiagnostic, this);
  diagnostic_thread_.detach();
}

void NDTScanMatcher::timerDiagnostic()
{
  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    diagnostic_msgs::msg::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "ndt_scan_matcher";
    diag_status_msg.hardware_id = "";

    for (const auto & key_value : key_value_stdmap_) {
      diagnostic_msgs::msg::KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing") {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag_status_msg.message += "Initializing State. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      diag_status_msg.message += "skipping_publish_num > 1. ";
    }
    if (
      key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5) {
      diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
    }
    // Ignore local optimal solution
    if (
      key_value_stdmap_.count("is_local_optimal_solution_oscillation") &&
      std::stoi(key_value_stdmap_["is_local_optimal_solution_oscillation"])) {
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

void NDTScanMatcher::serviceNDTAlign(
  const autoware_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  autoware_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  // get TF from pose_frame to map_frame
  auto TF_pose_to_map_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  getTransform(map_frame_, req->pose_with_covariance.header.frame_id, TF_pose_to_map_ptr);

  // transform pose_frame to map_frame
  const auto mapTF_initial_pose_msg = transform(req->pose_with_covariance, *TF_pose_to_map_ptr);

  if (ndt_ptr_->getInputTarget() == nullptr) {
    res->success = false;
    res->seq = req->seq;
    RCLCPP_WARN(get_logger(), "No InputTarget");
    return;
  }

  if (ndt_ptr_->getInputSource() == nullptr) {
    res->success = false;
    res->seq = req->seq;
    RCLCPP_WARN(get_logger(), "No InputSource");
    return;
  }

  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  key_value_stdmap_["state"] = "Aligning";
  res->pose_with_covariance = alignUsingMonteCarlo(ndt_ptr_, mapTF_initial_pose_msg);
  key_value_stdmap_["state"] = "Sleeping";
  res->success = true;
  res->seq = req->seq;
  res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;
}

void NDTScanMatcher::callbackInitialPose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
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
    getTransform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map_ptr);

    // transform pose_frame to map_frame
    auto mapTF_initial_pose_msg_ptr =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    // mapTF_initial_pose_msg_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
    *mapTF_initial_pose_msg_ptr = transform(*initial_pose_msg_ptr, *TF_pose_to_map_ptr);
    initial_pose_msg_ptr_array_.push_back(mapTF_initial_pose_msg_ptr);
  }
}

void NDTScanMatcher::callbackMapPoints(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr map_points_msg_ptr)
{
  const auto trans_epsilon = ndt_ptr_->getTransformationEpsilon();
  const auto step_size = ndt_ptr_->getStepSize();
  const auto resolution = ndt_ptr_->getResolution();
  const auto max_iterations = ndt_ptr_->getMaximumIterations();

  using NDTBase = NormalDistributionsTransformBase<PointSource, PointTarget>;
  std::shared_ptr<NDTBase> new_ndt_ptr_ = getNDT<PointSource, PointTarget>(ndt_implement_type_);

  if (ndt_implement_type_ == NDTImplementType::OMP) {
    using T = NormalDistributionsTransformOMP<PointSource, PointTarget>;

    // FIXME(IshitaTakeshi) Not sure if this is safe
    std::shared_ptr<T> ndt_omp_ptr = std::dynamic_pointer_cast<T>(ndt_ptr_);
    ndt_omp_ptr->setNeighborhoodSearchMethod(omp_params_.search_method);
    ndt_omp_ptr->setNumThreads(omp_params_.num_threads);
    new_ndt_ptr_ = ndt_omp_ptr;
  }

  new_ndt_ptr_->setTransformationEpsilon(trans_epsilon);
  new_ndt_ptr_->setStepSize(step_size);
  new_ndt_ptr_->setResolution(resolution);
  new_ndt_ptr_->setMaximumIterations(max_iterations);

  boost::shared_ptr<pcl::PointCloud<PointTarget>> map_points_ptr(new pcl::PointCloud<PointTarget>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
  new_ndt_ptr_->setInputTarget(map_points_ptr);
  // create Thread
  // detach
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  new_ndt_ptr_->align(*output_cloud, Eigen::Matrix4f::Identity());

  // swap
  ndt_map_mtx_.lock();
  ndt_ptr_ = new_ndt_ptr_;
  ndt_map_mtx_.unlock();
}

void NDTScanMatcher::callbackSensorPoints(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_sensorTF_msg_ptr)
{
  const auto exe_start_time = std::chrono::system_clock::now();
  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  const std::string & sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
  const rclcpp::Time sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

  boost::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_sensorTF_ptr(
    new pcl::PointCloud<PointSource>);
  pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);
  // get TF base to sensor
  auto TF_base_to_sensor_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  getTransform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);
  const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
  const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();
  boost::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_baselinkTF_ptr(
    new pcl::PointCloud<PointSource>);
  pcl::transformPointCloud(
    *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);
  ndt_ptr_->setInputSource(sensor_points_baselinkTF_ptr);

  // check
  if (initial_pose_msg_ptr_array_.size() <= 1) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "No Pose!");
    return;
  }
  // searchNNPose using timestamp
  auto initial_pose_old_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  auto initial_pose_new_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getNearestTimeStampPose(
    initial_pose_msg_ptr_array_, sensor_ros_time, initial_pose_old_msg_ptr,
    initial_pose_new_msg_ptr);
  popOldPose(initial_pose_msg_ptr_array_, sensor_ros_time);
  // TODO(Tier IV): check pose_timestamp - sensor_ros_time
  const auto initial_pose_msg =
    interpolatePose(*initial_pose_old_msg_ptr, *initial_pose_new_msg_ptr, sensor_ros_time);

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_cov_msg;
  initial_pose_cov_msg.header = initial_pose_msg.header;
  initial_pose_cov_msg.pose.pose = initial_pose_msg.pose;

  if (ndt_ptr_->getInputTarget() == nullptr) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1, "No MAP!");
    return;
  }
  // align
  const Eigen::Affine3d initial_pose_affine = fromRosPoseToEigen(initial_pose_cov_msg.pose.pose);
  const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  key_value_stdmap_["state"] = "Aligning";
  ndt_ptr_->align(*output_cloud, initial_pose_matrix);
  key_value_stdmap_["state"] = "Sleeping";

  const Eigen::Matrix4f result_pose_matrix = ndt_ptr_->getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();
  const geometry_msgs::msg::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

  const std::vector<Eigen::Matrix4f> result_pose_matrix_array =
    ndt_ptr_->getFinalTransformationArray();
  std::vector<geometry_msgs::msg::Pose> result_pose_msg_array;
  for (const auto & pose_matrix : result_pose_matrix_array) {
    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose_matrix.cast<double>();
    const geometry_msgs::msg::Pose pose_msg = tf2::toMsg(pose_affine);
    result_pose_msg_array.push_back(pose_msg);
  }

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() /
    1000.0;

  const float transform_probability = ndt_ptr_->getTransformationProbability();

  const int iteration_num = ndt_ptr_->getFinalNumIteration();

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
  bool is_local_optimal_solution_oscillation = false;
  if (iteration_num >= ndt_ptr_->getMaximumIterations() + 2) {
    is_local_optimal_solution_oscillation = isLocalOptimalSolutionOscillation(
      result_pose_matrix_array, oscillation_threshold_, inversion_vector_threshold_);
  }

  bool is_converged = true;
  static size_t skipping_publish_num = 0;
  if (
    iteration_num >= ndt_ptr_->getMaximumIterations() + 2 ||
    transform_probability < converged_param_transform_probability_) {
    is_converged = false;
    ++skipping_publish_num;
    RCLCPP_WARN(get_logger(), "Not Converged");
  } else {
    skipping_publish_num = 0;
  }

  // publish
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;

  // TODO(Tier IV): temporary value
  Eigen::Map<RowMatrixXd> covariance(&result_pose_with_cov_msg.pose.covariance[0], 6, 6);
  covariance(0, 0) = 0.025;
  covariance(1, 1) = 0.025;
  covariance(2, 2) = 0.025;
  covariance(3, 3) = 0.000625;
  covariance(4, 4) = 0.000625;
  covariance(5, 5) = 0.000625;

  if (is_converged) {
    ndt_pose_pub_->publish(result_pose_stamped_msg);
    ndt_pose_with_covariance_pub_->publish(result_pose_with_cov_msg);
  }

  publishTF(ndt_base_frame_, result_pose_stamped_msg);

  auto sensor_points_mapTF_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
  pcl::transformPointCloud(
    *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
  sensor_msgs::msg::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = map_frame_;
  sensor_aligned_pose_pub_->publish(sensor_points_mapTF_msg);

  initial_pose_with_covariance_pub_->publish(initial_pose_cov_msg);

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = sensor_ros_time;
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = autoware_utils::createMarkerScale(0.3, 0.1, 0.1);
  int i = 0;
  marker.ns = "result_pose_matrix_array";
  marker.action = visualization_msgs::msg::Marker::ADD;
  for (const auto & pose_msg : result_pose_msg_array) {
    marker.id = i++;
    marker.pose = pose_msg;
    marker.color = ExchangeColorCrc((1.0 * i) / 15.0);
    marker_array.markers.push_back(marker);
  }
  // TODO(Tier IV): delete old marker
  for (; i < ndt_ptr_->getMaximumIterations() + 2;) {
    marker.id = i++;
    marker.pose = geometry_msgs::msg::Pose();
    marker.color = ExchangeColorCrc(0);
    marker_array.markers.push_back(marker);
  }
  ndt_marker_pub_->publish(marker_array);

  exe_time_pub_->publish(makeFloat32Stamped(sensor_ros_time, exe_time));

  transform_probability_pub_->publish(makeFloat32Stamped(sensor_ros_time, transform_probability));

  iteration_num_pub_->publish(makeInt32Stamped(sensor_ros_time, iteration_num));

  const float initial_to_result_distance =
    norm(initial_pose_cov_msg.pose.pose.position, result_pose_with_cov_msg.pose.pose.position);
  initial_to_result_distance_pub_->publish(
    makeFloat32Stamped(sensor_ros_time, initial_to_result_distance));

  const float initial_to_result_distance_old =
    norm(initial_pose_old_msg_ptr->pose.pose.position, result_pose_with_cov_msg.pose.pose.position);
  initial_to_result_distance_old_pub_->publish(
    makeFloat32Stamped(sensor_ros_time, initial_to_result_distance_old));

  const float initial_to_result_distance_new =
    norm(initial_pose_new_msg_ptr->pose.pose.position, result_pose_with_cov_msg.pose.pose.position);
  initial_to_result_distance_new_pub_->publish(
    makeFloat32Stamped(sensor_ros_time, initial_to_result_distance_new));

  key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
  key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
  key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);
  if (is_local_optimal_solution_oscillation) {
    key_value_stdmap_["is_local_optimal_solution_oscillation"] = "1";
  } else {
    key_value_stdmap_["is_local_optimal_solution_oscillation"] = "0";
  }
}

geometry_msgs::msg::PoseWithCovarianceStamped NDTScanMatcher::alignUsingMonteCarlo(
  const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov)
{
  if (ndt_ptr->getInputTarget() == nullptr || ndt_ptr->getInputSource() == nullptr) {
    RCLCPP_WARN(get_logger(), "No Map or Sensor PointCloud");
    return geometry_msgs::msg::PoseWithCovarianceStamped();
  }

  // generateParticle
  const auto initial_poses = createRandomPoseArray(initial_pose_with_cov, 100);

  std::vector<Particle> particle_array;
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();

  for (unsigned int i = 0; i < initial_poses.size(); i++) {
    const auto & initial_pose = initial_poses[i];

    const Eigen::Affine3d initial_pose_affine = fromRosPoseToEigen(initial_pose);
    const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

    ndt_ptr->align(*output_cloud, initial_pose_matrix);

    const Eigen::Matrix4f result_pose_matrix = ndt_ptr->getFinalTransformation();
    Eigen::Affine3d result_pose_affine;
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const geometry_msgs::msg::Pose result_pose = tf2::toMsg(result_pose_affine);

    const auto transform_probability = ndt_ptr->getTransformationProbability();
    const auto num_iteration = ndt_ptr->getFinalNumIteration();

    Particle particle(initial_pose, result_pose, transform_probability, num_iteration);
    particle_array.push_back(particle);
    const auto marker_array = makeDebugMarkers(
      this->now(), map_frame_, autoware_utils::createMarkerScale(0.3, 0.1, 0.1), particle, i);
    ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);

    auto sensor_points_mapTF_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    const auto sensor_points_baselinkTF_ptr = ndt_ptr->getInputSource();
    pcl::transformPointCloud(
      *sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
    sensor_msgs::msg::PointCloud2 sensor_points_mapTF_msg;
    pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
    sensor_points_mapTF_msg.header.stamp = initial_pose_with_cov.header.stamp;
    sensor_points_mapTF_msg.header.frame_id = map_frame_;
    sensor_aligned_pose_pub_->publish(sensor_points_mapTF_msg);
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = best_particle_ptr->result_pose;
  // ndt_pose_with_covariance_pub_->publish(result_pose_with_cov_msg);

  return result_pose_with_cov_msg;
}

void NDTScanMatcher::publishTF(
  const std::string & child_frame_id, const geometry_msgs::msg::PoseStamped & pose_msg)
{
  tf2_broadcaster_.sendTransform(autoware_utils::pose2transform(pose_msg, child_frame_id));
}

bool NDTScanMatcher::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::msg::TransformStamped::SharedPtr & transform_stamped_ptr)
{
  const geometry_msgs::msg::TransformStamped identity =
    identityTransformStamped(this->now(), target_frame, source_frame);

  if (target_frame == source_frame) {
    *transform_stamped_ptr = identity;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    *transform_stamped_ptr = identity;
    return false;
  }
  return true;
}
