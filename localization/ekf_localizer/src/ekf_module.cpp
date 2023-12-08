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

#include "ekf_localizer/ekf_module.hpp"

#include "ekf_localizer/covariance.hpp"
#include "ekf_localizer/mahalanobis.hpp"
#include "ekf_localizer/matrix_types.hpp"
#include "ekf_localizer/measurement.hpp"
#include "ekf_localizer/numeric.hpp"
#include "ekf_localizer/state_transition.hpp"
#include "ekf_localizer/warning_message.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/msg_covariance.hpp>

#include <fmt/core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

// clang-format off
#define DEBUG_PRINT_MAT(X) {\
  if (params_.show_debug_info) {std::cout << #X << ": " << X << std::endl;}\
}
// clang-format on

EKFModule::EKFModule(std::shared_ptr<Warning> warning, const HyperParameters params)
: warning_(std::move(warning)),
  dim_x_(6),  // x, y, yaw, yaw_bias, vx, wz
  accumulated_delay_times_(params.extend_state_step, 1.0E15),
  params_(params)
{
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E15;  // for x & y
  P(IDX::YAW, IDX::YAW) = 50.0;                                            // for yaw
  if (params_.enable_yaw_bias_estimation) {
    P(IDX::YAWB, IDX::YAWB) = 50.0;  // for yaw bias
  }
  P(IDX::VX, IDX::VX) = 1000.0;  // for vx
  P(IDX::WZ, IDX::WZ) = 50.0;    // for wz

  kalman_filter_.init(X, P, params_.extend_state_step);
}

void EKFModule::initialize(
  const PoseWithCovariance & initial_pose, const geometry_msgs::msg::TransformStamped & transform)
{
  Eigen::MatrixXd X(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  X(IDX::X) = initial_pose.pose.pose.position.x + transform.transform.translation.x;
  X(IDX::Y) = initial_pose.pose.pose.position.y + transform.transform.translation.y;
  X(IDX::YAW) =
    tf2::getYaw(initial_pose.pose.pose.orientation) + tf2::getYaw(transform.transform.rotation);
  X(IDX::YAWB) = 0.0;
  X(IDX::VX) = 0.0;
  X(IDX::WZ) = 0.0;

  using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  P(IDX::X, IDX::X) = initial_pose.pose.covariance[COV_IDX::X_X];
  P(IDX::Y, IDX::Y) = initial_pose.pose.covariance[COV_IDX::Y_Y];
  P(IDX::YAW, IDX::YAW) = initial_pose.pose.covariance[COV_IDX::YAW_YAW];

  if (params_.enable_yaw_bias_estimation) {
    P(IDX::YAWB, IDX::YAWB) = 0.0001;
  }
  P(IDX::VX, IDX::VX) = 0.01;
  P(IDX::WZ, IDX::WZ) = 0.01;

  kalman_filter_.init(X, P, params_.extend_state_step);
}

geometry_msgs::msg::PoseStamped EKFModule::getCurrentPose(
  const rclcpp::Time & current_time, const double z, const double roll, const double pitch,
  bool get_biased_yaw) const
{
  const double x = kalman_filter_.getXelement(IDX::X);
  const double y = kalman_filter_.getXelement(IDX::Y);
  const double biased_yaw = kalman_filter_.getXelement(IDX::YAW);
  const double yaw_bias = kalman_filter_.getXelement(IDX::YAWB);
  const double yaw = biased_yaw + yaw_bias;
  Pose current_ekf_pose;
  current_ekf_pose.header.frame_id = params_.pose_frame_id;
  current_ekf_pose.header.stamp = current_time;
  current_ekf_pose.pose.position = tier4_autoware_utils::createPoint(x, y, z);
  if (get_biased_yaw) {
    current_ekf_pose.pose.orientation =
      tier4_autoware_utils::createQuaternionFromRPY(roll, pitch, biased_yaw);
  } else {
    current_ekf_pose.pose.orientation =
      tier4_autoware_utils::createQuaternionFromRPY(roll, pitch, yaw);
  }
  return current_ekf_pose;
}

geometry_msgs::msg::TwistStamped EKFModule::getCurrentTwist(const rclcpp::Time & current_time) const
{
  const double vx = kalman_filter_.getXelement(IDX::VX);
  const double wz = kalman_filter_.getXelement(IDX::WZ);

  Twist current_ekf_twist;
  current_ekf_twist.header.frame_id = "base_link";
  current_ekf_twist.header.stamp = current_time;
  current_ekf_twist.twist.linear.x = vx;
  current_ekf_twist.twist.angular.z = wz;
  return current_ekf_twist;
}

std::array<double, 36> EKFModule::getCurrentPoseCovariance() const
{
  return ekfCovarianceToPoseMessageCovariance(kalman_filter_.getLatestP());
}

std::array<double, 36> EKFModule::getCurrentTwistCovariance() const
{
  return ekfCovarianceToTwistMessageCovariance(kalman_filter_.getLatestP());
}

double EKFModule::getYawBias() const
{
  return kalman_filter_.getLatestX()(IDX::YAWB);
}

size_t EKFModule::find_closest_delay_time_index(double target_value) const
{
  // If target_value is too large, return last index + 1
  if (target_value > accumulated_delay_times_.back()) {
    return accumulated_delay_times_.size();
  }

  auto lower = std::lower_bound(
    accumulated_delay_times_.begin(), accumulated_delay_times_.end(), target_value);

  // If the lower bound is the first element, return its index.
  // If the lower bound is beyond the last element, return the last index.
  // If else, take the closest element.
  if (lower == accumulated_delay_times_.begin()) {
    return 0;
  } else if (lower == accumulated_delay_times_.end()) {
    return accumulated_delay_times_.size() - 1;
  } else {
    // Compare the target with the lower bound and the previous element.
    auto prev = lower - 1;
    bool is_closer_to_prev = (target_value - *prev) < (*lower - target_value);

    // Return the index of the closer element.
    return is_closer_to_prev ? std::distance(accumulated_delay_times_.begin(), prev)
                             : std::distance(accumulated_delay_times_.begin(), lower);
  }
}

void EKFModule::accumulate_delay_time(const double dt)
{
  // Shift the delay times to the right.
  std::copy_backward(
    accumulated_delay_times_.begin(), accumulated_delay_times_.end() - 1,
    accumulated_delay_times_.end());

  // Add a new element (=0) and, and add delay time to the previous elements.
  accumulated_delay_times_.front() = 0.0;
  for (size_t i = 1; i < accumulated_delay_times_.size(); ++i) {
    accumulated_delay_times_[i] += dt;
  }
}

void EKFModule::predictWithDelay(const double dt)
{
  const Eigen::MatrixXd X_curr = kalman_filter_.getLatestX();
  const Eigen::MatrixXd P_curr = kalman_filter_.getLatestP();

  const double proc_cov_vx_d = std::pow(params_.proc_stddev_vx_c * dt, 2.0);
  const double proc_cov_wz_d = std::pow(params_.proc_stddev_wz_c * dt, 2.0);
  const double proc_cov_yaw_d = std::pow(params_.proc_stddev_yaw_c * dt, 2.0);

  const Vector6d X_next = predictNextState(X_curr, dt);
  const Matrix6d A = createStateTransitionMatrix(X_curr, dt);
  const Matrix6d Q = processNoiseCovariance(proc_cov_yaw_d, proc_cov_vx_d, proc_cov_wz_d);
  kalman_filter_.predictWithDelay(X_next, A, Q);
}

bool EKFModule::measurementUpdatePose(
  const PoseWithCovariance & pose, const rclcpp::Time & t_curr, EKFDiagnosticInfo & pose_diag_info)
{
  if (pose.header.frame_id != params_.pose_frame_id) {
    warning_->warnThrottle(
      fmt::format(
        "pose frame_id is %s, but pose_frame is set as %s. They must be same.",
        pose.header.frame_id.c_str(), params_.pose_frame_id.c_str()),
      2000);
  }
  const Eigen::MatrixXd X_curr = kalman_filter_.getLatestX();
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 3;  // pos_x, pos_y, yaw, depending on Pose output

  /* Calculate delay step */
  double delay_time = (t_curr - pose.header.stamp).seconds() + params_.pose_additional_delay;
  if (delay_time < 0.0) {
    warning_->warnThrottle(poseDelayTimeWarningMessage(delay_time), 1000);
  }

  delay_time = std::max(delay_time, 0.0);

  const int delay_step = static_cast<int>(find_closest_delay_time_index(delay_time));

  pose_diag_info.delay_time = std::max(delay_time, pose_diag_info.delay_time);
  pose_diag_info.delay_time_threshold = accumulated_delay_times_.back();
  if (delay_step >= params_.extend_state_step) {
    pose_diag_info.is_passed_delay_gate = false;
    warning_->warnThrottle(
      poseDelayStepWarningMessage(pose_diag_info.delay_time, pose_diag_info.delay_time_threshold),
      2000);
    return false;
  }

  /* Set yaw */
  double yaw = tf2::getYaw(pose.pose.pose.orientation);
  const double ekf_yaw = kalman_filter_.getXelement(delay_step * dim_x_ + IDX::YAW);
  const double yaw_error = normalizeYaw(yaw - ekf_yaw);  // normalize the error not to exceed 2 pi
  yaw = yaw_error + ekf_yaw;

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << pose.pose.pose.position.x, pose.pose.pose.position.y, yaw;

  if (hasNan(y) || hasInf(y)) {
    warning_->warn(
      "[EKF] pose measurement matrix includes NaN of Inf. ignore update. check pose message.");
    return false;
  }

  /* Gate */
  const Eigen::Vector3d y_ekf(
    kalman_filter_.getXelement(delay_step * dim_x_ + IDX::X),
    kalman_filter_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw);
  const Eigen::MatrixXd P_curr = kalman_filter_.getLatestP();
  const Eigen::MatrixXd P_y = P_curr.block(0, 0, dim_y, dim_y);

  const double distance = mahalanobis(y_ekf, y, P_y);
  pose_diag_info.mahalanobis_distance = std::max(distance, pose_diag_info.mahalanobis_distance);
  if (distance > params_.pose_gate_dist) {
    pose_diag_info.is_passed_mahalanobis_gate = false;
    warning_->warnThrottle(mahalanobisWarningMessage(distance, params_.pose_gate_dist), 2000);
    warning_->warnThrottle("Ignore the measurement data.", 2000);
    return false;
  }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  const Eigen::Matrix<double, 3, 6> C = poseMeasurementMatrix();
  const Eigen::Matrix3d R =
    poseMeasurementCovariance(pose.pose.covariance, params_.pose_smoothing_steps);

  kalman_filter_.updateWithDelay(y, C, R, delay_step);

  // debug
  const Eigen::MatrixXd X_result = kalman_filter_.getLatestX();
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());

  return true;
}

geometry_msgs::msg::PoseWithCovarianceStamped EKFModule::compensatePoseWithZDelay(
  const PoseWithCovariance & pose, const double delay_time)
{
  const auto rpy = tier4_autoware_utils::getRPY(pose.pose.pose.orientation);
  const double dz_delay = kalman_filter_.getXelement(IDX::VX) * delay_time * std::sin(-rpy.y);
  PoseWithCovariance pose_with_z_delay;
  pose_with_z_delay = pose;
  pose_with_z_delay.pose.pose.position.z += dz_delay;
  return pose_with_z_delay;
}

bool EKFModule::measurementUpdateTwist(
  const TwistWithCovariance & twist, const rclcpp::Time & t_curr,
  EKFDiagnosticInfo & twist_diag_info)
{
  if (twist.header.frame_id != "base_link") {
    warning_->warnThrottle("twist frame_id must be base_link", 2000);
  }

  const Eigen::MatrixXd X_curr = kalman_filter_.getLatestX();
  DEBUG_PRINT_MAT(X_curr.transpose());

  constexpr int dim_y = 2;  // vx, wz

  /* Calculate delay step */
  double delay_time = (t_curr - twist.header.stamp).seconds() + params_.twist_additional_delay;
  if (delay_time < 0.0) {
    warning_->warnThrottle(twistDelayTimeWarningMessage(delay_time), 1000);
  }
  delay_time = std::max(delay_time, 0.0);

  const int delay_step = static_cast<int>(find_closest_delay_time_index(delay_time));

  twist_diag_info.delay_time = std::max(delay_time, twist_diag_info.delay_time);
  twist_diag_info.delay_time_threshold = accumulated_delay_times_.back();
  if (delay_step >= params_.extend_state_step) {
    twist_diag_info.is_passed_delay_gate = false;
    warning_->warnThrottle(
      twistDelayStepWarningMessage(
        twist_diag_info.delay_time, twist_diag_info.delay_time_threshold),
      2000);
    return false;
  }

  /* Set measurement matrix */
  Eigen::MatrixXd y(dim_y, 1);
  y << twist.twist.twist.linear.x, twist.twist.twist.angular.z;

  if (hasNan(y) || hasInf(y)) {
    warning_->warn(
      "[EKF] twist measurement matrix includes NaN of Inf. ignore update. check twist message.");
    return false;
  }

  const Eigen::Vector2d y_ekf(
    kalman_filter_.getXelement(delay_step * dim_x_ + IDX::VX),
    kalman_filter_.getXelement(delay_step * dim_x_ + IDX::WZ));
  const Eigen::MatrixXd P_curr = kalman_filter_.getLatestP();
  const Eigen::MatrixXd P_y = P_curr.block(4, 4, dim_y, dim_y);

  const double distance = mahalanobis(y_ekf, y, P_y);
  twist_diag_info.mahalanobis_distance = std::max(distance, twist_diag_info.mahalanobis_distance);
  if (distance > params_.twist_gate_dist) {
    twist_diag_info.is_passed_mahalanobis_gate = false;
    warning_->warnThrottle(mahalanobisWarningMessage(distance, params_.twist_gate_dist), 2000);
    warning_->warnThrottle("Ignore the measurement data.", 2000);
    return false;
  }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  const Eigen::Matrix<double, 2, 6> C = twistMeasurementMatrix();
  const Eigen::Matrix2d R =
    twistMeasurementCovariance(twist.twist.covariance, params_.twist_smoothing_steps);

  kalman_filter_.updateWithDelay(y, C, R, delay_step);

  // debug
  const Eigen::MatrixXd X_result = kalman_filter_.getLatestX();
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());

  return true;
}
