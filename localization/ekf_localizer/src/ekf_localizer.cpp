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

#include <rclcpp/logging.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
#define DEBUG_PRINT_MAT(X) {if (show_debug_info_) {std::cout << #X << ": " << X << std::endl;}}

// clang-format on

using std::placeholders::_1;

EKFLocalizer::EKFLocalizer(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options), dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */)
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
  pose_rate_ = declare_parameter("pose_rate", 10.0);  // used for covariance calculation
  pose_gate_dist_ = declare_parameter("pose_gate_dist", 10000.0);  // Mahalanobis limit

  /* twist measurement */
  twist_additional_delay_ = declare_parameter("twist_additional_delay", 0.0);
  twist_rate_ = declare_parameter("twist_rate", 10.0);  // used for covariance calculation
  twist_gate_dist_ = declare_parameter("twist_gate_dist", 10000.0);  // Mahalanobis limit

  /* process noise */
  double proc_stddev_yaw_c, proc_stddev_yaw_bias_c, proc_stddev_vx_c, proc_stddev_wz_c;
  proc_stddev_yaw_c = declare_parameter("proc_stddev_yaw_c", 0.005);
  proc_stddev_yaw_bias_c = declare_parameter("proc_stddev_yaw_bias_c", 0.001);
  proc_stddev_vx_c = declare_parameter("proc_stddev_vx_c", 5.0);
  proc_stddev_wz_c = declare_parameter("proc_stddev_wz_c", 1.0);
  if (!enable_yaw_bias_estimation_) {
    proc_stddev_yaw_bias_c = 0.0;
  }

  /* convert to continuous to discrete */
  proc_cov_vx_d_ = std::pow(proc_stddev_vx_c * ekf_dt_, 2.0);
  proc_cov_wz_d_ = std::pow(proc_stddev_wz_c * ekf_dt_, 2.0);
  proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c * ekf_dt_, 2.0);
  proc_cov_yaw_bias_d_ = std::pow(proc_stddev_yaw_bias_c * ekf_dt_, 2.0);

  /* initialize ros system */
  auto timer_control_callback = std::bind(&EKFLocalizer::timerCallback, this);
  auto period_control =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(ekf_dt_));
  timer_control_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_control_callback)>>(
    get_clock(), period_control, std::move(timer_control_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_control_, nullptr);

  auto timer_tf_callback = std::bind(&EKFLocalizer::timerTFCallback, this);
  auto period_tf = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / tf_rate_));
  timer_tf_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_tf_callback)>>(
    get_clock(), period_tf, std::move(timer_tf_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_tf_, nullptr);

  pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose", 1);
  pub_pose_cov_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf_pose_with_covariance", 1);
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("ekf_odom", 1);
  pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("ekf_twist", 1);
  pub_twist_cov_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "ekf_twist_with_covariance", 1);
  pub_yaw_bias_ = create_publisher<tier4_debug_msgs::msg::Float64Stamped>("estimated_yaw_bias", 1);
  pub_pose_no_yawbias_ =
    create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose_without_yawbias", 1);
  pub_pose_cov_no_yawbias_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance_without_yawbias", 1);
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

  /* debug */
  pub_debug_ = create_publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>("debug", 1);
  pub_measured_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("debug/measured_pose", 1);
}

/*
 * timerCallback
 */
void EKFLocalizer::timerCallback()
{
  DEBUG_INFO(get_logger(), "========================= timer called =========================");

  /* predict model in EKF */
  stop_watch_.tic();
  DEBUG_INFO(get_logger(), "------------------------- start prediction -------------------------");
  predictKinematicsModel();
  DEBUG_INFO(get_logger(), "[EKF] predictKinematicsModel calc time = %f [ms]", stop_watch_.toc());
  DEBUG_INFO(get_logger(), "------------------------- end prediction -------------------------\n");

  /* pose measurement update */
  if (current_pose_ptr_ != nullptr) {
    DEBUG_INFO(get_logger(), "------------------------- start Pose -------------------------");
    stop_watch_.tic();
    measurementUpdatePose(*current_pose_ptr_);
    DEBUG_INFO(get_logger(), "[EKF] measurementUpdatePose calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO(get_logger(), "------------------------- end Pose -------------------------\n");
  }

  /* twist measurement update */
  if (current_twist_ptr_ != nullptr) {
    DEBUG_INFO(get_logger(), "------------------------- start twist -------------------------");
    stop_watch_.tic();
    measurementUpdateTwist(*current_twist_ptr_);
    DEBUG_INFO(get_logger(), "[EKF] measurementUpdateTwist calc time = %f [ms]", stop_watch_.toc());
    DEBUG_INFO(get_logger(), "------------------------- end twist -------------------------\n");
  }

  /* set current pose, twist */
  setCurrentResult();

  /* publish ekf result */
  publishEstimateResult();
}

void EKFLocalizer::showCurrentX()
{
  if (show_debug_info_) {
    Eigen::MatrixXd X(dim_x_, 1);
    ekf_.getLatestX(X);
    DEBUG_PRINT_MAT(X.transpose());
  }
}

/*
 * setCurrentResult
 */
void EKFLocalizer::setCurrentResult()
{
  current_ekf_pose_.header.frame_id = pose_frame_id_;
  current_ekf_pose_.header.stamp = this->now();
  current_ekf_pose_.pose.position.x = ekf_.getXelement(IDX::X);
  current_ekf_pose_.pose.position.y = ekf_.getXelement(IDX::Y);

  tf2::Quaternion q_tf;
  double roll = 0.0, pitch = 0.0;
  if (current_pose_ptr_ != nullptr) {
    current_ekf_pose_.pose.position.z = current_pose_ptr_->pose.position.z;
    tf2::fromMsg(current_pose_ptr_->pose.orientation, q_tf); /* use Pose pitch and roll */
    double yaw_tmp;
    tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw_tmp);
  }
  double yaw = ekf_.getXelement(IDX::YAW) + ekf_.getXelement(IDX::YAWB);
  current_ekf_pose_.pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(roll, pitch, yaw);

  current_ekf_pose_no_yawbias_ = current_ekf_pose_;
  current_ekf_pose_no_yawbias_.pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(roll, pitch, ekf_.getXelement(IDX::YAW));

  current_ekf_twist_.header.frame_id = "base_link";
  current_ekf_twist_.header.stamp = this->now();
  current_ekf_twist_.twist.linear.x = ekf_.getXelement(IDX::VX);
  current_ekf_twist_.twist.angular.z = ekf_.getXelement(IDX::WZ);
}

/*
 * timerTFCallback
 */
void EKFLocalizer::timerTFCallback()
{
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
      RCLCPP_WARN(get_logger(), "%s", ex.what());
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

  P(IDX::X, IDX::X) = initialpose->pose.covariance[0];
  P(IDX::Y, IDX::Y) = initialpose->pose.covariance[6 + 1];
  P(IDX::YAW, IDX::YAW) = initialpose->pose.covariance[6 * 5 + 5];
  P(IDX::YAWB, IDX::YAWB) = 0.0001;
  P(IDX::VX, IDX::VX) = 0.01;
  P(IDX::WZ, IDX::WZ) = 0.01;

  ekf_.init(X, P, extend_state_step_);

  current_pose_ptr_ = nullptr;
}

/*
 * callbackPoseWithCovariance
 */
void EKFLocalizer::callbackPoseWithCovariance(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(pose);
  current_pose_covariance_ = msg->pose.covariance;
}

/*
 * callbackTwistWithCovariance
 */
void EKFLocalizer::callbackTwistWithCovariance(
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;
  current_twist_ptr_ = std::make_shared<geometry_msgs::msg::TwistStamped>(twist);
  current_twist_covariance_ = msg->twist.covariance;
}

/*
 * initEKF
 */
void EKFLocalizer::initEKF()
{
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E15;  // for x & y
  P(IDX::YAW, IDX::YAW) = 50.0;                                            // for yaw
  P(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_;                          // for yaw bias
  P(IDX::VX, IDX::VX) = 1000.0;                                            // for vx
  P(IDX::WZ, IDX::WZ) = 50.0;                                              // for wz

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

  Eigen::MatrixXd X_curr(dim_x_, 1);  // current state
  Eigen::MatrixXd X_next(dim_x_, 1);  // predicted state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  Eigen::MatrixXd P_curr;
  ekf_.getLatestP(P_curr);

  const double yaw = X_curr(IDX::YAW);
  const double yaw_bias = X_curr(IDX::YAWB);
  const double vx = X_curr(IDX::VX);
  const double wz = X_curr(IDX::WZ);
  const double dt = ekf_dt_;

  /* Update for latest state */
  X_next(IDX::X) = X_curr(IDX::X) + vx * cos(yaw + yaw_bias) * dt;  // dx = v * cos(yaw)
  X_next(IDX::Y) = X_curr(IDX::Y) + vx * sin(yaw + yaw_bias) * dt;  // dy = v * sin(yaw)
  X_next(IDX::YAW) = X_curr(IDX::YAW) + (wz)*dt;                    // dyaw = omega + omega_bias
  X_next(IDX::YAWB) = yaw_bias;
  X_next(IDX::VX) = vx;
  X_next(IDX::WZ) = wz;

  X_next(IDX::YAW) = std::atan2(std::sin(X_next(IDX::YAW)), std::cos(X_next(IDX::YAW)));

  /* Set A matrix for latest state */
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  A(IDX::X, IDX::YAW) = -vx * sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::YAWB) = -vx * sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::VX) = cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAW) = vx * cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAWB) = vx * cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::VX) = sin(yaw + yaw_bias) * dt;
  A(IDX::YAW, IDX::WZ) = dt;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  Q(IDX::X, IDX::X) = 0.0;
  Q(IDX::Y, IDX::Y) = 0.0;
  Q(IDX::YAW, IDX::YAW) = proc_cov_yaw_d_;         // for yaw
  Q(IDX::YAWB, IDX::YAWB) = proc_cov_yaw_bias_d_;  // for yaw bias
  Q(IDX::VX, IDX::VX) = proc_cov_vx_d_;            // for vx
  Q(IDX::WZ, IDX::WZ) = proc_cov_wz_d_;            // for wz

  ekf_.predictWithDelay(X_next, A, Q);

  // debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdatePose
 */
void EKFLocalizer::measurementUpdatePose(const geometry_msgs::msg::PoseStamped & pose)
{
  if (pose.header.frame_id != pose_frame_id_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(2000).count(),
      "pose frame_id is %s, but pose_frame is set as %s. They must be same.",
      pose.header.frame_id.c_str(), pose_frame_id_.c_str());
  }
  Eigen::MatrixXd X_curr(dim_x_, 1);  // current state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 3;  // pos_x, pos_y, yaw, depending on Pose output
  const rclcpp::Time t_curr = this->now();

  /* Calculate delay step */
  double delay_time = (t_curr - pose.header.stamp).seconds() + pose_additional_delay_;
  if (delay_time < 0.0) {
    delay_time = 0.0;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Pose time stamp is inappropriate, set delay to 0[s]. delay = %f", delay_time);
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Pose delay exceeds the compensation limit, ignored. delay: %f[s], limit = "
      "extend_state_step * ekf_dt : %f [s]",
      delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  DEBUG_INFO(get_logger(), "delay_time: %f [s]", delay_time);

  /* Set yaw */
  double yaw = tf2::getYaw(pose.pose.orientation);
  const double ekf_yaw = ekf_.getXelement(delay_step * dim_x_ + IDX::YAW);
  const double yaw_error = normalizeYaw(yaw - ekf_yaw);  // normalize the error not to exceed 2 pi
  yaw = yaw_error + ekf_yaw;

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << pose.pose.position.x, pose.pose.position.y, yaw;

  if (isnan(y.array()).any() || isinf(y.array()).any()) {
    RCLCPP_WARN(
      get_logger(),
      "[EKF] pose measurement matrix includes NaN of Inf. ignore update. check pose message.");
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::X),
    ekf_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw;
  Eigen::MatrixXd P_curr, P_y;
  ekf_.getLatestP(P_curr);
  P_y = P_curr.block(0, 0, dim_y, dim_y);
  if (!mahalanobisGate(pose_gate_dist_, y_ekf, y, P_y)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(2000).count(),
      "[EKF] Pose measurement update, mahalanobis distance is over limit. ignore "
      "measurement data.");
    return;
  }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::X) = 1.0;    // for pos x
  C(1, IDX::Y) = 1.0;    // for pos y
  C(2, IDX::YAW) = 1.0;  // for yaw

  /* Set measurement noise covariance */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  R(0, 0) = current_pose_covariance_.at(0);   // x - x
  R(0, 1) = current_pose_covariance_.at(1);   // x - y
  R(0, 2) = current_pose_covariance_.at(5);   // x - yaw
  R(1, 0) = current_pose_covariance_.at(6);   // y - x
  R(1, 1) = current_pose_covariance_.at(7);   // y - y
  R(1, 2) = current_pose_covariance_.at(11);  // y - yaw
  R(2, 0) = current_pose_covariance_.at(30);  // yaw - x
  R(2, 1) = current_pose_covariance_.at(31);  // yaw - y
  R(2, 2) = current_pose_covariance_.at(35);  // yaw - yaw

  /* In order to avoid a large change at the time of updating,
   * measurement update is performed by dividing at every step. */
  R *= (ekf_rate_ / pose_rate_);

  ekf_.updateWithDelay(y, C, R, delay_step);

  // debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdateTwist
 */
void EKFLocalizer::measurementUpdateTwist(const geometry_msgs::msg::TwistStamped & twist)
{
  if (twist.header.frame_id != "base_link") {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(2000).count(),
      "twist frame_id must be base_link");
  }

  Eigen::MatrixXd X_curr(dim_x_, 1);  // current state
  ekf_.getLatestX(X_curr);
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 2;  // vx, wz
  const rclcpp::Time t_curr = this->now();

  /* Calculate delay step */
  double delay_time = (t_curr - twist.header.stamp).seconds() + twist_additional_delay_;
  if (delay_time < 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Twist time stamp is inappropriate (delay = %f [s]), set delay to 0[s].", delay_time);
    delay_time = 0.0;
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Twist delay exceeds the compensation limit, ignored. delay: %f[s], limit = "
      "extend_state_step * ekf_dt : %f [s]",
      delay_time, extend_state_step_ * ekf_dt_);
    return;
  }
  DEBUG_INFO(get_logger(), "delay_time: %f [s]", delay_time);

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << twist.twist.linear.x, twist.twist.angular.z;

  if (isnan(y.array()).any() || isinf(y.array()).any()) {
    RCLCPP_WARN(
      get_logger(),
      "[EKF] twist measurement matrix includes NaN of Inf. ignore update. check twist message.");
    return;
  }

  /* Gate */
  Eigen::MatrixXd y_ekf(dim_y, 1);
  y_ekf << ekf_.getXelement(delay_step * dim_x_ + IDX::VX),
    ekf_.getXelement(delay_step * dim_x_ + IDX::WZ);
  Eigen::MatrixXd P_curr, P_y;
  ekf_.getLatestP(P_curr);
  P_y = P_curr.block(4, 4, dim_y, dim_y);
  if (!mahalanobisGate(twist_gate_dist_, y_ekf, y, P_y)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(2000).count(),
      "[EKF] Twist measurement update, mahalanobis distance is over limit. ignore "
      "measurement data.");
    return;
  }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, IDX::VX) = 1.0;  // for vx
  C(1, IDX::WZ) = 1.0;  // for wz

  /* Set measurement noise covariance */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  R(0, 0) = current_twist_covariance_.at(0);   // vx - vx
  R(0, 1) = current_twist_covariance_.at(5);   // vx - wz
  R(1, 0) = current_twist_covariance_.at(30);  // wz - vx
  R(1, 1) = current_twist_covariance_.at(35);  // wz - wz

  /* In order to avoid a large change by update, measurement update is performed
   * by dividing at every step. measurement update is performed by dividing at every step. */
  R *= (ekf_rate_ / twist_rate_);

  ekf_.updateWithDelay(y, C, R, delay_step);

  // debug
  Eigen::MatrixXd X_result(dim_x_, 1);
  ekf_.getLatestX(X_result);
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * mahalanobisGate
 */
bool EKFLocalizer::mahalanobisGate(
  const double & dist_max, const Eigen::MatrixXd & x, const Eigen::MatrixXd & obj_x,
  const Eigen::MatrixXd & cov) const
{
  Eigen::MatrixXd mahalanobis_squared = (x - obj_x).transpose() * cov.inverse() * (x - obj_x);
  DEBUG_INFO(
    get_logger(), "measurement update: mahalanobis = %f, gate limit = %f",
    std::sqrt(mahalanobis_squared(0)), dist_max);
  if (mahalanobis_squared(0) > dist_max * dist_max) {
    return false;
  }

  return true;
}

/*
 * publishEstimateResult
 */
void EKFLocalizer::publishEstimateResult()
{
  rclcpp::Time current_time = this->now();
  Eigen::MatrixXd X(dim_x_, 1);
  Eigen::MatrixXd P(dim_x_, dim_x_);
  ekf_.getLatestX(X);
  ekf_.getLatestP(P);

  /* publish latest pose */
  pub_pose_->publish(current_ekf_pose_);
  pub_pose_no_yawbias_->publish(current_ekf_pose_no_yawbias_);

  /* publish latest pose with covariance */
  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = current_time;
  pose_cov.header.frame_id = current_ekf_pose_.header.frame_id;
  pose_cov.pose.pose = current_ekf_pose_.pose;
  pose_cov.pose.covariance[0] = P(IDX::X, IDX::X);
  pose_cov.pose.covariance[1] = P(IDX::X, IDX::Y);
  pose_cov.pose.covariance[5] = P(IDX::X, IDX::YAW);
  pose_cov.pose.covariance[6] = P(IDX::Y, IDX::X);
  pose_cov.pose.covariance[7] = P(IDX::Y, IDX::Y);
  pose_cov.pose.covariance[11] = P(IDX::Y, IDX::YAW);
  pose_cov.pose.covariance[30] = P(IDX::YAW, IDX::X);
  pose_cov.pose.covariance[31] = P(IDX::YAW, IDX::Y);
  pose_cov.pose.covariance[35] = P(IDX::YAW, IDX::YAW);
  pub_pose_cov_->publish(pose_cov);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_no_yawbias = pose_cov;
  pose_cov_no_yawbias.pose.pose = current_ekf_pose_no_yawbias_.pose;
  pub_pose_cov_no_yawbias_->publish(pose_cov_no_yawbias);

  /* publish latest twist */
  pub_twist_->publish(current_ekf_twist_);

  /* publish latest twist with covariance */
  geometry_msgs::msg::TwistWithCovarianceStamped twist_cov;
  twist_cov.header.stamp = current_time;
  twist_cov.header.frame_id = current_ekf_twist_.header.frame_id;
  twist_cov.twist.twist = current_ekf_twist_.twist;
  twist_cov.twist.covariance[0] = P(IDX::VX, IDX::VX);
  twist_cov.twist.covariance[5] = P(IDX::VX, IDX::WZ);
  twist_cov.twist.covariance[30] = P(IDX::WZ, IDX::VX);
  twist_cov.twist.covariance[35] = P(IDX::WZ, IDX::WZ);
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
  if (current_pose_ptr_ != nullptr) {
    geometry_msgs::msg::PoseStamped p;
    p = *current_pose_ptr_;
    p.header.stamp = current_time;
    pub_measured_pose_->publish(p);
  }

  /* debug publish */
  double pose_yaw = 0.0;
  if (current_pose_ptr_ != nullptr) {
    pose_yaw = tf2::getYaw(current_pose_ptr_->pose.orientation);
  }

  tier4_debug_msgs::msg::Float64MultiArrayStamped msg;
  msg.stamp = current_time;
  msg.data.push_back(tier4_autoware_utils::rad2deg(X(IDX::YAW)));   // [0] ekf yaw angle
  msg.data.push_back(tier4_autoware_utils::rad2deg(pose_yaw));      // [1] measurement yaw angle
  msg.data.push_back(tier4_autoware_utils::rad2deg(X(IDX::YAWB)));  // [2] yaw bias
  pub_debug_->publish(msg);
}

double EKFLocalizer::normalizeYaw(const double & yaw) const
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}
