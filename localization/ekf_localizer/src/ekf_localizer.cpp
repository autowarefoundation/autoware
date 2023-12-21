// Copyright 2018-2019 Autoware Foundation
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

#include "ekf_localizer/ekf_localizer.hpp"

#include "ekf_localizer/diagnostics.hpp"
#include "ekf_localizer/string.hpp"
#include "ekf_localizer/warning_message.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <tier4_autoware_utils/ros/msg_covariance.hpp>

#include <fmt/core.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <utility>

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) {if (params_.show_debug_info) {RCLCPP_INFO(__VA_ARGS__);}}
// clang-format on

using std::placeholders::_1;

EKFLocalizer::EKFLocalizer(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options),
  warning_(std::make_shared<Warning>(this)),
  params_(this),
  ekf_dt_(params_.ekf_dt),
  pose_queue_(params_.pose_smoothing_steps),
  twist_queue_(params_.twist_smoothing_steps)
{
  /* convert to continuous to discrete */
  proc_cov_vx_d_ = std::pow(params_.proc_stddev_vx_c * ekf_dt_, 2.0);
  proc_cov_wz_d_ = std::pow(params_.proc_stddev_wz_c * ekf_dt_, 2.0);
  proc_cov_yaw_d_ = std::pow(params_.proc_stddev_yaw_c * ekf_dt_, 2.0);

  is_activated_ = false;

  /* initialize ros system */
  timer_control_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(ekf_dt_),
    std::bind(&EKFLocalizer::timerCallback, this));

  timer_tf_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params_.tf_rate_).period(),
    std::bind(&EKFLocalizer::timerTFCallback, this));

  pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose", 1);
  pub_pose_cov_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf_pose_with_covariance", 1);
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("ekf_odom", 1);
  pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("ekf_twist", 1);
  pub_twist_cov_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "ekf_twist_with_covariance", 1);
  pub_yaw_bias_ = create_publisher<tier4_debug_msgs::msg::Float64Stamped>("estimated_yaw_bias", 1);
  pub_biased_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf_biased_pose", 1);
  pub_biased_pose_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_biased_pose_with_covariance", 1);
  pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  sub_initialpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&EKFLocalizer::callbackInitialPose, this, _1));
  sub_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_pose_with_covariance", 1, std::bind(&EKFLocalizer::callbackPoseWithCovariance, this, _1));
  sub_twist_with_cov_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_twist_with_covariance", 1, std::bind(&EKFLocalizer::callbackTwistWithCovariance, this, _1));
  service_trigger_node_ = create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &EKFLocalizer::serviceTriggerNode, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile());

  tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    std::shared_ptr<rclcpp::Node>(this, [](auto) {}));

  ekf_module_ = std::make_unique<EKFModule>(warning_, params_);
  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);

  z_filter_.set_proc_dev(params_.z_filter_proc_dev);
  roll_filter_.set_proc_dev(params_.roll_filter_proc_dev);
  pitch_filter_.set_proc_dev(params_.pitch_filter_proc_dev);
}

/*
 * updatePredictFrequency
 */
void EKFLocalizer::updatePredictFrequency()
{
  if (last_predict_time_) {
    if (get_clock()->now() < *last_predict_time_) {
      warning_->warn("Detected jump back in time");
    } else {
      /* Measure dt */
      ekf_dt_ = (get_clock()->now() - *last_predict_time_).seconds();
      DEBUG_INFO(
        get_logger(), "[EKF] update ekf_dt_ to %f seconds (= %f hz)", ekf_dt_, 1 / ekf_dt_);

      if (ekf_dt_ > 10.0) {
        ekf_dt_ = 10.0;
        RCLCPP_WARN(
          get_logger(), "Large ekf_dt_ detected!! (%f sec) Capped to 10.0 seconds", ekf_dt_);
      } else if (ekf_dt_ > params_.pose_smoothing_steps / params_.ekf_rate) {
        RCLCPP_WARN(
          get_logger(), "EKF period may be too slow to finish pose smoothing!! (%f sec) ", ekf_dt_);
      }

      /* Register dt and accumulate time delay */
      ekf_module_->accumulate_delay_time(ekf_dt_);

      /* Update discrete proc_cov*/
      proc_cov_vx_d_ = std::pow(params_.proc_stddev_vx_c * ekf_dt_, 2.0);
      proc_cov_wz_d_ = std::pow(params_.proc_stddev_wz_c * ekf_dt_, 2.0);
      proc_cov_yaw_d_ = std::pow(params_.proc_stddev_yaw_c * ekf_dt_, 2.0);
    }
  }
  last_predict_time_ = std::make_shared<const rclcpp::Time>(get_clock()->now());
}

/*
 * timerCallback
 */
void EKFLocalizer::timerCallback()
{
  if (!is_activated_) {
    warning_->warnThrottle(
      "The node is not activated. Provide initial pose to pose_initializer", 2000);
    publishDiagnostics();
    return;
  }

  DEBUG_INFO(get_logger(), "========================= timer called =========================");

  /* update predict frequency with measured timer rate */
  updatePredictFrequency();

  /* predict model in EKF */
  stop_watch_.tic();
  DEBUG_INFO(get_logger(), "------------------------- start prediction -------------------------");
  ekf_module_->predictWithDelay(ekf_dt_);
  DEBUG_INFO(get_logger(), "[EKF] predictKinematicsModel calc time = %f [ms]", stop_watch_.toc());
  DEBUG_INFO(get_logger(), "------------------------- end prediction -------------------------\n");

  /* pose measurement update */
  pose_diag_info_.queue_size = pose_queue_.size();
  pose_diag_info_.is_passed_delay_gate = true;
  pose_diag_info_.delay_time = 0.0;
  pose_diag_info_.delay_time_threshold = 0.0;
  pose_diag_info_.is_passed_mahalanobis_gate = true;
  pose_diag_info_.mahalanobis_distance = 0.0;

  bool pose_is_updated = false;

  if (!pose_queue_.empty()) {
    DEBUG_INFO(get_logger(), "------------------------- start Pose -------------------------");
    stop_watch_.tic();

    // save the initial size because the queue size can change in the loop
    const auto t_curr = this->now();
    const size_t n = pose_queue_.size();
    for (size_t i = 0; i < n; ++i) {
      const auto pose = pose_queue_.pop_increment_age();
      bool is_updated = ekf_module_->measurementUpdatePose(*pose, t_curr, pose_diag_info_);
      if (is_updated) {
        pose_is_updated = true;

        // Update Simple 1D filter with considering change of z value due to measurement pose delay
        const double delay_time =
          (t_curr - pose->header.stamp).seconds() + params_.pose_additional_delay;
        const auto pose_with_z_delay = ekf_module_->compensatePoseWithZDelay(*pose, delay_time);
        updateSimple1DFilters(pose_with_z_delay, params_.pose_smoothing_steps);
      }
    }
    DEBUG_INFO(get_logger(), "[EKF] measurementUpdatePose calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO(get_logger(), "------------------------- end Pose -------------------------\n");
  }
  pose_diag_info_.no_update_count = pose_is_updated ? 0 : (pose_diag_info_.no_update_count + 1);

  /* twist measurement update */
  twist_diag_info_.queue_size = twist_queue_.size();
  twist_diag_info_.is_passed_delay_gate = true;
  twist_diag_info_.delay_time = 0.0;
  twist_diag_info_.delay_time_threshold = 0.0;
  twist_diag_info_.is_passed_mahalanobis_gate = true;
  twist_diag_info_.mahalanobis_distance = 0.0;

  bool twist_is_updated = false;

  if (!twist_queue_.empty()) {
    DEBUG_INFO(get_logger(), "------------------------- start Twist -------------------------");
    stop_watch_.tic();

    // save the initial size because the queue size can change in the loop
    const auto t_curr = this->now();
    const size_t n = twist_queue_.size();
    for (size_t i = 0; i < n; ++i) {
      const auto twist = twist_queue_.pop_increment_age();
      bool is_updated = ekf_module_->measurementUpdateTwist(*twist, t_curr, twist_diag_info_);
      if (is_updated) {
        twist_is_updated = true;
      }
    }
    DEBUG_INFO(get_logger(), "[EKF] measurementUpdateTwist calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO(get_logger(), "------------------------- end Twist -------------------------\n");
  }
  twist_diag_info_.no_update_count = twist_is_updated ? 0 : (twist_diag_info_.no_update_count + 1);

  const double z = z_filter_.get_x();
  const double roll = roll_filter_.get_x();
  const double pitch = pitch_filter_.get_x();
  const geometry_msgs::msg::PoseStamped current_ekf_pose =
    ekf_module_->getCurrentPose(this->now(), z, roll, pitch, false);
  const geometry_msgs::msg::PoseStamped current_biased_ekf_pose =
    ekf_module_->getCurrentPose(this->now(), z, roll, pitch, true);
  const geometry_msgs::msg::TwistStamped current_ekf_twist =
    ekf_module_->getCurrentTwist(this->now());

  /* publish ekf result */
  publishEstimateResult(current_ekf_pose, current_biased_ekf_pose, current_ekf_twist);
  publishDiagnostics();
}

/*
 * timerTFCallback
 */
void EKFLocalizer::timerTFCallback()
{
  if (!is_activated_) {
    return;
  }

  if (params_.pose_frame_id == "") {
    return;
  }

  const double z = z_filter_.get_x();
  const double roll = roll_filter_.get_x();
  const double pitch = pitch_filter_.get_x();

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped = tier4_autoware_utils::pose2transform(
    ekf_module_->getCurrentPose(this->now(), z, roll, pitch, false), "base_link");
  transform_stamped.header.stamp = this->now();
  tf_br_->sendTransform(transform_stamped);
}

/*
 * getTransformFromTF
 */
bool EKFLocalizer::getTransformFromTF(
  std::string parent_frame, std::string child_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  tf2::BufferCore tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  parent_frame = eraseLeadingSlash(parent_frame);
  child_frame = eraseLeadingSlash(child_frame);

  for (int i = 0; i < 50; ++i) {
    try {
      transform = tf_buffer.lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
      return true;
    } catch (tf2::TransformException & ex) {
      warning_->warn(ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }
  return false;
}

/*
 * callbackInitialPose
 */
void EKFLocalizer::callbackInitialPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
{
  geometry_msgs::msg::TransformStamped transform;
  if (!getTransformFromTF(params_.pose_frame_id, initialpose->header.frame_id, transform)) {
    RCLCPP_ERROR(
      get_logger(), "[EKF] TF transform failed. parent = %s, child = %s",
      params_.pose_frame_id.c_str(), initialpose->header.frame_id.c_str());
  }
  ekf_module_->initialize(*initialpose, transform);
  initSimple1DFilters(*initialpose);
}

/*
 * callbackPoseWithCovariance
 */
void EKFLocalizer::callbackPoseWithCovariance(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (!is_activated_) {
    return;
  }

  pose_queue_.push(msg);
}

/*
 * callbackTwistWithCovariance
 */
void EKFLocalizer::callbackTwistWithCovariance(
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  // Ignore twist if velocity is too small.
  // Note that this inequality must not include "equal".
  if (std::abs(msg->twist.twist.linear.x) < params_.threshold_observable_velocity_mps) {
    msg->twist.covariance[0 * 6 + 0] = 10000.0;
  }
  twist_queue_.push(msg);
}

/*
 * publishEstimateResult
 */
void EKFLocalizer::publishEstimateResult(
  const geometry_msgs::msg::PoseStamped & current_ekf_pose,
  const geometry_msgs::msg::PoseStamped & current_biased_ekf_pose,
  const geometry_msgs::msg::TwistStamped & current_ekf_twist)
{
  rclcpp::Time current_time = this->now();

  /* publish latest pose */
  pub_pose_->publish(current_ekf_pose);
  pub_biased_pose_->publish(current_biased_ekf_pose);

  /* publish latest pose with covariance */
  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = current_time;
  pose_cov.header.frame_id = current_ekf_pose.header.frame_id;
  pose_cov.pose.pose = current_ekf_pose.pose;
  pose_cov.pose.covariance = ekf_module_->getCurrentPoseCovariance();
  pub_pose_cov_->publish(pose_cov);

  geometry_msgs::msg::PoseWithCovarianceStamped biased_pose_cov = pose_cov;
  biased_pose_cov.pose.pose = current_biased_ekf_pose.pose;
  pub_biased_pose_cov_->publish(biased_pose_cov);

  /* publish latest twist */
  pub_twist_->publish(current_ekf_twist);

  /* publish latest twist with covariance */
  geometry_msgs::msg::TwistWithCovarianceStamped twist_cov;
  twist_cov.header.stamp = current_time;
  twist_cov.header.frame_id = current_ekf_twist.header.frame_id;
  twist_cov.twist.twist = current_ekf_twist.twist;
  twist_cov.twist.covariance = ekf_module_->getCurrentTwistCovariance();
  pub_twist_cov_->publish(twist_cov);

  /* publish yaw bias */
  tier4_debug_msgs::msg::Float64Stamped yawb;
  yawb.stamp = current_time;
  yawb.data = ekf_module_->getYawBias();
  pub_yaw_bias_->publish(yawb);

  /* publish latest odometry */
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = current_time;
  odometry.header.frame_id = current_ekf_pose.header.frame_id;
  odometry.child_frame_id = "base_link";
  odometry.pose = pose_cov.pose;
  odometry.twist = twist_cov.twist;
  pub_odom_->publish(odometry);
}

void EKFLocalizer::publishDiagnostics()
{
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> diag_status_array;

  diag_status_array.push_back(checkProcessActivated(is_activated_));

  if (is_activated_) {
    diag_status_array.push_back(checkMeasurementUpdated(
      "pose", pose_diag_info_.no_update_count, params_.pose_no_update_count_threshold_warn,
      params_.pose_no_update_count_threshold_error));
    diag_status_array.push_back(checkMeasurementQueueSize("pose", pose_diag_info_.queue_size));
    diag_status_array.push_back(checkMeasurementDelayGate(
      "pose", pose_diag_info_.is_passed_delay_gate, pose_diag_info_.delay_time,
      pose_diag_info_.delay_time_threshold));
    diag_status_array.push_back(checkMeasurementMahalanobisGate(
      "pose", pose_diag_info_.is_passed_mahalanobis_gate, pose_diag_info_.mahalanobis_distance,
      params_.pose_gate_dist));

    diag_status_array.push_back(checkMeasurementUpdated(
      "twist", twist_diag_info_.no_update_count, params_.twist_no_update_count_threshold_warn,
      params_.twist_no_update_count_threshold_error));
    diag_status_array.push_back(checkMeasurementQueueSize("twist", twist_diag_info_.queue_size));
    diag_status_array.push_back(checkMeasurementDelayGate(
      "twist", twist_diag_info_.is_passed_delay_gate, twist_diag_info_.delay_time,
      twist_diag_info_.delay_time_threshold));
    diag_status_array.push_back(checkMeasurementMahalanobisGate(
      "twist", twist_diag_info_.is_passed_mahalanobis_gate, twist_diag_info_.mahalanobis_distance,
      params_.twist_gate_dist));
  }

  diagnostic_msgs::msg::DiagnosticStatus diag_merged_status;
  diag_merged_status = mergeDiagnosticStatus(diag_status_array);
  diag_merged_status.name = "localization: " + std::string(this->get_name());
  diag_merged_status.hardware_id = this->get_name();

  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = this->now();
  diag_msg.status.push_back(diag_merged_status);
  pub_diag_->publish(diag_msg);
}

void EKFLocalizer::updateSimple1DFilters(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose, const size_t smoothing_step)
{
  double z = pose.pose.pose.position.z;

  const auto rpy = tier4_autoware_utils::getRPY(pose.pose.pose.orientation);

  using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  double z_dev = pose.pose.covariance[COV_IDX::Z_Z] * static_cast<double>(smoothing_step);
  double roll_dev = pose.pose.covariance[COV_IDX::ROLL_ROLL] * static_cast<double>(smoothing_step);
  double pitch_dev =
    pose.pose.covariance[COV_IDX::PITCH_PITCH] * static_cast<double>(smoothing_step);

  z_filter_.update(z, z_dev, pose.header.stamp);
  roll_filter_.update(rpy.x, roll_dev, pose.header.stamp);
  pitch_filter_.update(rpy.y, pitch_dev, pose.header.stamp);
}

void EKFLocalizer::initSimple1DFilters(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  double z = pose.pose.pose.position.z;

  const auto rpy = tier4_autoware_utils::getRPY(pose.pose.pose.orientation);

  using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  double z_dev = pose.pose.covariance[COV_IDX::Z_Z];
  double roll_dev = pose.pose.covariance[COV_IDX::ROLL_ROLL];
  double pitch_dev = pose.pose.covariance[COV_IDX::PITCH_PITCH];

  z_filter_.init(z, z_dev, pose.header.stamp);
  roll_filter_.init(rpy.x, roll_dev, pose.header.stamp);
  pitch_filter_.init(rpy.y, pitch_dev, pose.header.stamp);
}

/**
 * @brief trigger node
 */
void EKFLocalizer::serviceTriggerNode(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (req->data) {
    pose_queue_.clear();
    twist_queue_.clear();
    is_activated_ = true;
  } else {
    is_activated_ = false;
  }
  res->success = true;
  return;
}
