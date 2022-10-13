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

#include "ekf_localizer/covariance.hpp"
#include "ekf_localizer/mahalanobis.hpp"
#include "ekf_localizer/matrix_types.hpp"
#include "ekf_localizer/measurement.hpp"
#include "ekf_localizer/numeric.hpp"
#include "ekf_localizer/state_index.hpp"
#include "ekf_localizer/state_transition.hpp"
#include "ekf_localizer/warning.hpp"

#include <rclcpp/logging.hpp>
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
#define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
#define DEBUG_PRINT_MAT(X) {if (show_debug_info_) {std::cout << #X << ": " << X << std::endl;}}

// clang-format on

using std::placeholders::_1;

EKFLocalizer::EKFLocalizer(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options), warning_(this), dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */)
{
  show_debug_info_ = declare_parameter("show_debug_info", false);
  ekf_rate_ = declare_parameter("predict_frequency", 50.0);
  ekf_dt_ = 1.0 / std::max(ekf_rate_, 0.1);
  tf_rate_ = declare_parameter("tf_rate", 10.0);
  enable_yaw_bias_estimation_ = declare_parameter("enable_yaw_bias_estimation", true);
  extend_state_step_ = declare_parameter("extend_state_step", 50);
  pose_frame_id_ = declare_parameter("pose_frame_id", std::string("map"));

  /* pose measurement */
  pose_additional_delay_ = declare_parameter("pose_additional_delay", 0.0);
  pose_measure_uncertainty_time_ = declare_parameter("pose_measure_uncertainty_time", 0.01);
  pose_gate_dist_ = declare_parameter("pose_gate_dist", 10000.0);  // Mahalanobis limit
  pose_smoothing_steps_ = declare_parameter("pose_smoothing_steps", 5);

  /* twist measurement */
  twist_additional_delay_ = declare_parameter("twist_additional_delay", 0.0);
  twist_gate_dist_ = declare_parameter("twist_gate_dist", 10000.0);  // Mahalanobis limit
  twist_smoothing_steps_ = declare_parameter("twist_smoothing_steps", 2);

  /* process noise */
  proc_stddev_yaw_c_ = declare_parameter("proc_stddev_yaw_c", 0.005);
  proc_stddev_vx_c_ = declare_parameter("proc_stddev_vx_c", 5.0);
  proc_stddev_wz_c_ = declare_parameter("proc_stddev_wz_c", 1.0);

  /* convert to continuous to discrete */
  proc_cov_vx_d_ = std::pow(proc_stddev_vx_c_ * ekf_dt_, 2.0);
  proc_cov_wz_d_ = std::pow(proc_stddev_wz_c_ * ekf_dt_, 2.0);
  proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c_ * ekf_dt_, 2.0);

  is_initialized_ = false;

  /* initialize ros system */
  auto period_control_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(ekf_dt_));
  timer_control_ = rclcpp::create_timer(
    this, get_clock(), period_control_ns, std::bind(&EKFLocalizer::timerCallback, this));

  const auto period_tf_ns = rclcpp::Rate(tf_rate_).period();
  timer_tf_ = rclcpp::create_timer(
    this, get_clock(), period_tf_ns, std::bind(&EKFLocalizer::timerTFCallback, this));

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
  sub_initialpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&EKFLocalizer::callbackInitialPose, this, _1));
  sub_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_pose_with_covariance", 1, std::bind(&EKFLocalizer::callbackPoseWithCovariance, this, _1));
  sub_twist_with_cov_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_twist_with_covariance", 1, std::bind(&EKFLocalizer::callbackTwistWithCovariance, this, _1));

  dim_x_ex_ = dim_x_ * extend_state_step_;

  tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    std::shared_ptr<rclcpp::Node>(this, [](auto) {}));

  initEKF();

  z_filter_.set_proc_dev(1.0);
  roll_filter_.set_proc_dev(0.01);
  pitch_filter_.set_proc_dev(0.01);

  /* debug */
  pub_debug_ = create_publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>("debug", 1);
  pub_measured_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("debug/measured_pose", 1);
}

/*
 * updatePredictFrequency
 */
void EKFLocalizer::updatePredictFrequency()
{
  if (last_predict_time_) {
    if (get_clock()->now() < *last_predict_time_) {
      warning_.warn("Detected jump back in time");
    } else {
      ekf_rate_ = 1.0 / (get_clock()->now() - *last_predict_time_).seconds();
      DEBUG_INFO(get_logger(), "[EKF] update ekf_rate_ to %f hz", ekf_rate_);
      ekf_dt_ = 1.0 / std::max(ekf_rate_, 0.1);

      /* Update discrete proc_cov*/
      proc_cov_vx_d_ = std::pow(proc_stddev_vx_c_ * ekf_dt_, 2.0);
      proc_cov_wz_d_ = std::pow(proc_stddev_wz_c_ * ekf_dt_, 2.0);
      proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c_ * ekf_dt_, 2.0);
    }
  }
  last_predict_time_ = std::make_shared<const rclcpp::Time>(get_clock()->now());
}

/*
 * timerCallback
 */
void EKFLocalizer::timerCallback()
{
  if (!is_initialized_) {
    return;
  }

  DEBUG_INFO(get_logger(), "========================= timer called =========================");

  /* update predict frequency with measured timer rate */
  updatePredictFrequency();

  /* predict model in EKF */
  stop_watch_.tic();
  DEBUG_INFO(get_logger(), "------------------------- start prediction -------------------------");
  predictKinematicsModel();
  DEBUG_INFO(get_logger(), "[EKF] predictKinematicsModel calc time = %f [ms]", stop_watch_.toc());
  DEBUG_INFO(get_logger(), "------------------------- end prediction -------------------------\n");

  /* pose measurement update */
  if (!current_pose_info_queue_.empty()) {
    DEBUG_INFO(get_logger(), "------------------------- start Pose -------------------------");
    stop_watch_.tic();

    int pose_info_queue_size = static_cast<int>(current_pose_info_queue_.size());
    for (int i = 0; i < pose_info_queue_size; ++i) {
      PoseInfo pose_info = current_pose_info_queue_.front();
      current_pose_info_queue_.pop();
      measurementUpdatePose(*pose_info.pose);
      ++pose_info.counter;
      if (pose_info.counter < pose_info.smoothing_steps) {
        current_pose_info_queue_.push(pose_info);
      }
    }
    DEBUG_INFO(get_logger(), "[EKF] measurementUpdatePose calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO(get_logger(), "------------------------- end Pose -------------------------\n");
  }

  /* twist measurement update */
  if (!current_twist_info_queue_.empty()) {
    DEBUG_INFO(get_logger(), "------------------------- start Twist -------------------------");
    stop_watch_.tic();

    int twist_info_queue_size = static_cast<int>(current_twist_info_queue_.size());
    for (int i = 0; i < twist_info_queue_size; ++i) {
      TwistInfo twist_info = current_twist_info_queue_.front();
      current_twist_info_queue_.pop();
      measurementUpdateTwist(*twist_info.twist);
      ++twist_info.counter;
      if (twist_info.counter < twist_info.smoothing_steps) {
        current_twist_info_queue_.push(twist_info);
      }
    }
    DEBUG_INFO(get_logger(), "[EKF] measurementUpdateTwist calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO(get_logger(), "------------------------- end Twist -------------------------\n");
  }

  const double x = ekf_.getXelement(IDX::X);
  const double y = ekf_.getXelement(IDX::Y);
  const double z = z_filter_.get_x();

  const double biased_yaw = ekf_.getXelement(IDX::YAW);
  const double yaw_bias = ekf_.getXelement(IDX::YAWB);

  const double roll = roll_filter_.get_x();
  const double pitch = pitch_filter_.get_x();
  const double yaw = biased_yaw + yaw_bias;
  const double vx = ekf_.getXelement(IDX::VX);
  const double wz = ekf_.getXelement(IDX::WZ);

  current_ekf_pose_.header.frame_id = pose_frame_id_;
  current_ekf_pose_.header.stamp = this->now();
  current_ekf_pose_.pose.position.x = x;
  current_ekf_pose_.pose.position.y = y;
  current_ekf_pose_.pose.position.z = z;
  current_ekf_pose_.pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(roll, pitch, yaw);

  current_biased_ekf_pose_ = current_ekf_pose_;
  current_biased_ekf_pose_.pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(roll, pitch, biased_yaw);

  current_ekf_twist_.header.frame_id = "base_link";
  current_ekf_twist_.header.stamp = this->now();
  current_ekf_twist_.twist.linear.x = vx;
  current_ekf_twist_.twist.angular.z = wz;

  /* publish ekf result */
  publishEstimateResult();
}

void EKFLocalizer::showCurrentX()
{
  if (show_debug_info_) {
    const Eigen::MatrixXd X = ekf_.getLatestX();
    DEBUG_PRINT_MAT(X.transpose());
  }
}

/*
 * timerTFCallback
 */
void EKFLocalizer::timerTFCallback()
{
  if (!is_initialized_) {
    return;
  }

  if (current_ekf_pose_.header.frame_id == "") {
    return;
  }

  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = current_ekf_pose_.header.frame_id;
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = current_ekf_pose_.pose.position.x;
  transformStamped.transform.translation.y = current_ekf_pose_.pose.position.y;
  transformStamped.transform.translation.z = current_ekf_pose_.pose.position.z;

  transformStamped.transform.rotation.x = current_ekf_pose_.pose.orientation.x;
  transformStamped.transform.rotation.y = current_ekf_pose_.pose.orientation.y;
  transformStamped.transform.rotation.z = current_ekf_pose_.pose.orientation.z;
  transformStamped.transform.rotation.w = current_ekf_pose_.pose.orientation.w;

  tf_br_->sendTransform(transformStamped);
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
  if (parent_frame.front() == '/') {
    parent_frame.erase(0, 1);
  }
  if (child_frame.front() == '/') {
    child_frame.erase(0, 1);
  }

  for (int i = 0; i < 50; ++i) {
    try {
      transform = tf_buffer.lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
      return true;
    } catch (tf2::TransformException & ex) {
      warning_.warn(ex.what());
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
  if (!getTransformFromTF(pose_frame_id_, initialpose->header.frame_id, transform)) {
    RCLCPP_ERROR(
      get_logger(), "[EKF] TF transform failed. parent = %s, child = %s", pose_frame_id_.c_str(),
      initialpose->header.frame_id.c_str());
  }

  Eigen::MatrixXd X(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  // TODO(mitsudome-r) need mutex

  X(IDX::X) = initialpose->pose.pose.position.x + transform.transform.translation.x;
  X(IDX::Y) = initialpose->pose.pose.position.y + transform.transform.translation.y;
  current_ekf_pose_.pose.position.z =
    initialpose->pose.pose.position.z + transform.transform.translation.z;
  X(IDX::YAW) =
    tf2::getYaw(initialpose->pose.pose.orientation) + tf2::getYaw(transform.transform.rotation);
  X(IDX::YAWB) = 0.0;
  X(IDX::VX) = 0.0;
  X(IDX::WZ) = 0.0;

  using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  P(IDX::X, IDX::X) = initialpose->pose.covariance[COV_IDX::X_X];
  P(IDX::Y, IDX::Y) = initialpose->pose.covariance[COV_IDX::Y_Y];
  P(IDX::YAW, IDX::YAW) = initialpose->pose.covariance[COV_IDX::YAW_YAW];

  if (enable_yaw_bias_estimation_) {
    P(IDX::YAWB, IDX::YAWB) = 0.0001;
  }
  P(IDX::VX, IDX::VX) = 0.01;
  P(IDX::WZ, IDX::WZ) = 0.01;

  ekf_.init(X, P, extend_state_step_);

  updateSimple1DFilters(*initialpose);

  while (!current_pose_info_queue_.empty()) current_pose_info_queue_.pop();

  is_initialized_ = true;
}

/*
 * callbackPoseWithCovariance
 */
void EKFLocalizer::callbackPoseWithCovariance(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  PoseInfo pose_info = {msg, 0, pose_smoothing_steps_};
  current_pose_info_queue_.push(pose_info);

  updateSimple1DFilters(*msg);
}

/*
 * callbackTwistWithCovariance
 */
void EKFLocalizer::callbackTwistWithCovariance(
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  TwistInfo twist_info = {msg, 0, twist_smoothing_steps_};
  current_twist_info_queue_.push(twist_info);
}

/*
 * initEKF
 */
void EKFLocalizer::initEKF()
{
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E15;  // for x & y
  P(IDX::YAW, IDX::YAW) = 50.0;                                            // for yaw
  if (enable_yaw_bias_estimation_) {
    P(IDX::YAWB, IDX::YAWB) = 50.0;  // for yaw bias
  }
  P(IDX::VX, IDX::VX) = 1000.0;  // for vx
  P(IDX::WZ, IDX::WZ) = 50.0;    // for wz

  ekf_.init(X, P, extend_state_step_);
}

/*
 * predictKinematicsModel
 */
void EKFLocalizer::predictKinematicsModel()
{
  /*  == Nonlinear model ==
   *
   * x_{k+1}   = x_k + vx_k * cos(yaw_k + b_k) * dt
   * y_{k+1}   = y_k + vx_k * sin(yaw_k + b_k) * dt
   * yaw_{k+1} = yaw_k + (wz_k) * dt
   * b_{k+1}   = b_k
   * vx_{k+1}  = vz_k
   * wz_{k+1}  = wz_k
   *
   * (b_k : yaw_bias_k)
   */

  /*  == Linearized model ==
   *
   * A = [ 1, 0, -vx*sin(yaw+b)*dt, -vx*sin(yaw+b)*dt, cos(yaw+b)*dt,  0]
   *     [ 0, 1,  vx*cos(yaw+b)*dt,  vx*cos(yaw+b)*dt, sin(yaw+b)*dt,  0]
   *     [ 0, 0,                 1,                 0,             0, dt]
   *     [ 0, 0,                 0,                 1,             0,  0]
   *     [ 0, 0,                 0,                 0,             1,  0]
   *     [ 0, 0,                 0,                 0,             0,  1]
   */

  const Eigen::MatrixXd X_curr = ekf_.getLatestX();
  DEBUG_PRINT_MAT(X_curr.transpose());

  const Eigen::MatrixXd P_curr = ekf_.getLatestP();

  const double dt = ekf_dt_;

  const Vector6d X_next = predictNextState(X_curr, dt);
  const Matrix6d A = createStateTransitionMatrix(X_curr, dt);
  const Matrix6d Q = processNoiseCovariance(proc_cov_yaw_d_, proc_cov_vx_d_, proc_cov_wz_d_);

  ekf_.predictWithDelay(X_next, A, Q);

  // debug
  const Eigen::MatrixXd X_result = ekf_.getLatestX();
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdatePose
 */
void EKFLocalizer::measurementUpdatePose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  if (pose.header.frame_id != pose_frame_id_) {
    warning_.warnThrottle(
      fmt::format(
        "pose frame_id is %s, but pose_frame is set as %s. They must be same.",
        pose.header.frame_id.c_str(), pose_frame_id_.c_str()),
      2000);
  }
  const Eigen::MatrixXd X_curr = ekf_.getLatestX();
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 3;  // pos_x, pos_y, yaw, depending on Pose output
  const rclcpp::Time t_curr = this->now();

  /* Calculate delay step */
  double delay_time = (t_curr - pose.header.stamp).seconds() + pose_additional_delay_;
  if (delay_time < 0.0) {
    delay_time = 0.0;
    warning_.warnThrottle(
      fmt::format("Pose time stamp is inappropriate, set delay to 0[s]. delay = %f", delay_time),
      1000);
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1) {
    warning_.warnThrottle(
      fmt::format(
        "Pose delay exceeds the compensation limit, ignored. delay: %f[s], limit = "
        "extend_state_step * ekf_dt : %f [s]",
        delay_time, extend_state_step_ * ekf_dt_),
      1000);
    return;
  }
  DEBUG_INFO(get_logger(), "delay_time: %f [s]", delay_time);

  /* Set yaw */
  double yaw = tf2::getYaw(pose.pose.pose.orientation);
  const double ekf_yaw = ekf_.getXelement(delay_step * dim_x_ + IDX::YAW);
  const double yaw_error = normalizeYaw(yaw - ekf_yaw);  // normalize the error not to exceed 2 pi
  yaw = yaw_error + ekf_yaw;

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << pose.pose.pose.position.x, pose.pose.pose.position.y, yaw;

  if (hasNan(y) || hasInf(y)) {
    warning_.warn(
      "[EKF] pose measurement matrix includes NaN of Inf. ignore update. check pose message.");
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::X),
    ekf_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw;
  const Eigen::MatrixXd P_curr = ekf_.getLatestP();
  const Eigen::MatrixXd P_y = P_curr.block(0, 0, dim_y, dim_y);
  if (!mahalanobisGate(pose_gate_dist_, y_ekf, y, P_y)) {
    warning_.warnThrottle(
      "[EKF] Pose measurement update, mahalanobis distance is over limit. ignore "
      "measurement data.",
      2000);
    return;
  }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  const Eigen::Matrix<double, 3, 6> C = poseMeasurementMatrix();
  const Eigen::Matrix3d R = poseMeasurementCovariance(pose.pose.covariance, pose_smoothing_steps_);

  ekf_.updateWithDelay(y, C, R, delay_step);

  // debug
  const Eigen::MatrixXd X_result = ekf_.getLatestX();
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdateTwist
 */
void EKFLocalizer::measurementUpdateTwist(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist)
{
  if (twist.header.frame_id != "base_link") {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(2000).count(),
      "twist frame_id must be base_link");
  }

  const Eigen::MatrixXd X_curr = ekf_.getLatestX();
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 2;  // vx, wz
  const rclcpp::Time t_curr = this->now();

  /* Calculate delay step */
  double delay_time = (t_curr - twist.header.stamp).seconds() + twist_additional_delay_;
  if (delay_time < 0.0) {
    warning_.warnThrottle(
      fmt::format(
        "Twist time stamp is inappropriate (delay = %f [s]), set delay to 0[s].", delay_time),
      1000);
    delay_time = 0.0;
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1) {
    warning_.warnThrottle(
      fmt::format(
        "Twist delay exceeds the compensation limit, ignored. delay: %f[s], limit = "
        "extend_state_step * ekf_dt : %f [s]",
        delay_time, extend_state_step_ * ekf_dt_),
      1000);
    return;
  }
  DEBUG_INFO(get_logger(), "delay_time: %f [s]", delay_time);

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << twist.twist.twist.linear.x, twist.twist.twist.angular.z;

  if (hasNan(y) || hasInf(y)) {
    warning_.warn(
      "[EKF] twist measurement matrix includes NaN of Inf. ignore update. check twist message.");
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::VX),
    ekf_.getXelement(delay_step * dim_x_ + IDX::WZ);
  const Eigen::MatrixXd P_curr = ekf_.getLatestP();
  const Eigen::MatrixXd P_y = P_curr.block(4, 4, dim_y, dim_y);
  if (!mahalanobisGate(twist_gate_dist_, y_ekf, y, P_y)) {
    warning_.warnThrottle(
      "[EKF] Twist measurement update, mahalanobis distance is over limit. ignore "
      "measurement data.",
      2000);
    return;
  }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  const Eigen::Matrix<double, 2, 6> C = twistMeasurementMatrix();
  const Eigen::Matrix2d R =
    twistMeasurementCovariance(twist.twist.covariance, twist_smoothing_steps_);

  ekf_.updateWithDelay(y, C, R, delay_step);

  // debug
  const Eigen::MatrixXd X_result = ekf_.getLatestX();
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * publishEstimateResult
 */
void EKFLocalizer::publishEstimateResult()
{
  rclcpp::Time current_time = this->now();
  const Eigen::MatrixXd X = ekf_.getLatestX();
  const Eigen::MatrixXd P = ekf_.getLatestP();

  /* publish latest pose */
  pub_pose_->publish(current_ekf_pose_);
  pub_biased_pose_->publish(current_biased_ekf_pose_);

  /* publish latest pose with covariance */
  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = current_time;
  pose_cov.header.frame_id = current_ekf_pose_.header.frame_id;
  pose_cov.pose.pose = current_ekf_pose_.pose;
  pose_cov.pose.covariance = ekfCovarianceToPoseMessageCovariance(P);
  pub_pose_cov_->publish(pose_cov);

  geometry_msgs::msg::PoseWithCovarianceStamped biased_pose_cov = pose_cov;
  biased_pose_cov.pose.pose = current_biased_ekf_pose_.pose;
  pub_biased_pose_cov_->publish(biased_pose_cov);

  /* publish latest twist */
  pub_twist_->publish(current_ekf_twist_);

  /* publish latest twist with covariance */
  geometry_msgs::msg::TwistWithCovarianceStamped twist_cov;
  twist_cov.header.stamp = current_time;
  twist_cov.header.frame_id = current_ekf_twist_.header.frame_id;
  twist_cov.twist.twist = current_ekf_twist_.twist;
  twist_cov.twist.covariance = ekfCovarianceToTwistMessageCovariance(P);
  pub_twist_cov_->publish(twist_cov);

  /* publish yaw bias */
  tier4_debug_msgs::msg::Float64Stamped yawb;
  yawb.stamp = current_time;
  yawb.data = X(IDX::YAWB);
  pub_yaw_bias_->publish(yawb);

  /* publish latest odometry */
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = current_time;
  odometry.header.frame_id = current_ekf_pose_.header.frame_id;
  odometry.child_frame_id = "base_link";
  odometry.pose = pose_cov.pose;
  odometry.twist = twist_cov.twist;
  pub_odom_->publish(odometry);

  /* debug measured pose */
  if (!current_pose_info_queue_.empty()) {
    geometry_msgs::msg::PoseStamped p;
    p.pose = current_pose_info_queue_.back().pose->pose.pose;
    p.header.stamp = current_time;
    pub_measured_pose_->publish(p);
  }

  /* debug publish */
  double pose_yaw = 0.0;
  if (!current_pose_info_queue_.empty()) {
    pose_yaw = tf2::getYaw(current_pose_info_queue_.back().pose->pose.pose.orientation);
  }

  tier4_debug_msgs::msg::Float64MultiArrayStamped msg;
  msg.stamp = current_time;
  msg.data.push_back(tier4_autoware_utils::rad2deg(X(IDX::YAW)));   // [0] ekf yaw angle
  msg.data.push_back(tier4_autoware_utils::rad2deg(pose_yaw));      // [1] measurement yaw angle
  msg.data.push_back(tier4_autoware_utils::rad2deg(X(IDX::YAWB)));  // [2] yaw bias
  pub_debug_->publish(msg);
}

void EKFLocalizer::updateSimple1DFilters(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  double z = pose.pose.pose.position.z;
  double roll = 0.0, pitch = 0.0, yaw_tmp = 0.0;

  tf2::Quaternion q_tf;
  tf2::fromMsg(pose.pose.pose.orientation, q_tf);
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw_tmp);

  using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  double z_dev = pose.pose.covariance[COV_IDX::Z_Z];
  double roll_dev = pose.pose.covariance[COV_IDX::ROLL_ROLL];
  double pitch_dev = pose.pose.covariance[COV_IDX::PITCH_PITCH];

  z_filter_.update(z, z_dev, pose.header.stamp);
  roll_filter_.update(roll, roll_dev, pose.header.stamp);
  pitch_filter_.update(pitch, pitch_dev, pose.header.stamp);
}
