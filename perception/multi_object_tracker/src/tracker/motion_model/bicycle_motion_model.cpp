// Copyright 2024 Tier IV, Inc.
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
//
//
// Author: v1.0 Taekjin Lee
//
#define EIGEN_MPL2_ONLY

#include "multi_object_tracker/tracker/motion_model/bicycle_motion_model.hpp"

#include "autoware/universe_utils/math/normalization.hpp"
#include "autoware/universe_utils/math/unit_conversion.hpp"
#include "autoware/universe_utils/ros/msg_covariance.hpp"
#include "multi_object_tracker/tracker/motion_model/motion_model_base.hpp"
#include "multi_object_tracker/utils/utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

// cspell: ignore CTRV
// Bicycle CTRV motion model
// CTRV : Constant Turn Rate and constant Velocity
using autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

BicycleMotionModel::BicycleMotionModel() : logger_(rclcpp::get_logger("BicycleMotionModel"))
{
  // Initialize motion parameters
  setDefaultParams();
}

void BicycleMotionModel::setDefaultParams()
{
  // set default motion parameters
  constexpr double q_stddev_acc_long = 9.8 * 0.35;  // [m/(s*s)] uncertain longitudinal acceleration
  constexpr double q_stddev_acc_lat = 9.8 * 0.15;   // [m/(s*s)] uncertain lateral acceleration
  constexpr double q_stddev_yaw_rate_min =
    autoware::universe_utils::deg2rad(1.5);  // [rad/s] uncertain yaw change rate
  constexpr double q_stddev_yaw_rate_max =
    autoware::universe_utils::deg2rad(15.0);  // [rad/s] uncertain yaw change rate
  constexpr double q_stddev_slip_rate_min =
    autoware::universe_utils::deg2rad(0.3);  // [rad/s] uncertain slip angle change rate
  constexpr double q_stddev_slip_rate_max =
    autoware::universe_utils::deg2rad(10.0);  // [rad/s] uncertain slip angle change rate
  constexpr double q_max_slip_angle =
    autoware::universe_utils::deg2rad(30.0);  // [rad] max slip angle
  // extended state parameters
  constexpr double lf_ratio = 0.3;   // 30% front from the center
  constexpr double lf_min = 1.0;     // minimum of 1.0m
  constexpr double lr_ratio = 0.25;  // 25% rear from the center
  constexpr double lr_min = 1.0;     // minimum of 1.0m
  setMotionParams(
    q_stddev_acc_long, q_stddev_acc_lat, q_stddev_yaw_rate_min, q_stddev_yaw_rate_max,
    q_stddev_slip_rate_min, q_stddev_slip_rate_max, q_max_slip_angle, lf_ratio, lf_min, lr_ratio,
    lr_min);

  // set motion limitations
  constexpr double max_vel = autoware::universe_utils::kmph2mps(100);  // [m/s] maximum velocity
  constexpr double max_slip = 30.0;                                    // [deg] maximum slip angle
  setMotionLimits(max_vel, max_slip);

  // set prediction parameters
  constexpr double dt_max = 0.11;  // [s] maximum time interval for prediction
  setMaxDeltaTime(dt_max);
}

void BicycleMotionModel::setMotionParams(
  const double & q_stddev_acc_long, const double & q_stddev_acc_lat,
  const double & q_stddev_yaw_rate_min, const double & q_stddev_yaw_rate_max,
  const double & q_stddev_slip_rate_min, const double & q_stddev_slip_rate_max,
  const double & q_max_slip_angle, const double & lf_ratio, const double & lf_min,
  const double & lr_ratio, const double & lr_min)
{
  // set process noise covariance parameters
  motion_params_.q_stddev_acc_long = q_stddev_acc_long;
  motion_params_.q_stddev_acc_lat = q_stddev_acc_lat;
  motion_params_.q_cov_acc_long = q_stddev_acc_long * q_stddev_acc_long;
  motion_params_.q_cov_acc_lat = q_stddev_acc_lat * q_stddev_acc_lat;
  motion_params_.q_stddev_yaw_rate_min = q_stddev_yaw_rate_min;
  motion_params_.q_stddev_yaw_rate_max = q_stddev_yaw_rate_max;
  motion_params_.q_cov_slip_rate_min = q_stddev_slip_rate_min * q_stddev_slip_rate_min;
  motion_params_.q_cov_slip_rate_max = q_stddev_slip_rate_max * q_stddev_slip_rate_max;
  motion_params_.q_max_slip_angle = q_max_slip_angle;

  constexpr double minimum_wheel_pos = 0.01;  // minimum of 0.01m
  if (lf_min < minimum_wheel_pos || lr_min < minimum_wheel_pos) {
    RCLCPP_WARN(
      logger_,
      "BicycleMotionModel::setMotionParams: minimum wheel position should be greater than 0.01m.");
  }
  motion_params_.lf_min = (lf_min < minimum_wheel_pos) ? minimum_wheel_pos : lf_min;
  motion_params_.lr_min = (lr_min < minimum_wheel_pos) ? minimum_wheel_pos : lr_min;
  motion_params_.lf_ratio = lf_ratio;
  motion_params_.lr_ratio = lr_ratio;
}

void BicycleMotionModel::setMotionLimits(const double & max_vel, const double & max_slip)
{
  // set motion limitations
  motion_params_.max_vel = max_vel;
  motion_params_.max_slip = max_slip;
}

bool BicycleMotionModel::initialize(
  const rclcpp::Time & time, const double & x, const double & y, const double & yaw,
  const std::array<double, 36> & pose_cov, const double & vel, const double & vel_cov,
  const double & slip, const double & slip_cov, const double & length)
{
  // initialize state vector X
  Eigen::MatrixXd X(DIM, 1);
  X << x, y, yaw, vel, slip;

  // initialize covariance matrix P
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(DIM, DIM);
  P(IDX::X, IDX::X) = pose_cov[XYZRPY_COV_IDX::X_X];
  P(IDX::Y, IDX::Y) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  P(IDX::YAW, IDX::YAW) = pose_cov[XYZRPY_COV_IDX::YAW_YAW];
  P(IDX::VEL, IDX::VEL) = vel_cov;
  P(IDX::SLIP, IDX::SLIP) = slip_cov;

  // set initial extended state
  if (!updateExtendedState(length)) return false;

  return MotionModel::initialize(time, X, P);
}

bool BicycleMotionModel::updateStatePose(
  const double & x, const double & y, const std::array<double, 36> & pose_cov)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // update state, without velocity
  constexpr int DIM_Y = 2;

  // update state
  Eigen::MatrixXd Y(DIM_Y, 1);
  Y << x, y;

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(DIM_Y, DIM);
  C(0, IDX::X) = 1.0;
  C(1, IDX::Y) = 1.0;

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  R(0, 0) = pose_cov[XYZRPY_COV_IDX::X_X];
  R(0, 1) = pose_cov[XYZRPY_COV_IDX::X_Y];
  R(1, 0) = pose_cov[XYZRPY_COV_IDX::Y_X];
  R(1, 1) = pose_cov[XYZRPY_COV_IDX::Y_Y];

  return ekf_.update(Y, C, R);
}

bool BicycleMotionModel::updateStatePoseHead(
  const double & x, const double & y, const double & yaw, const std::array<double, 36> & pose_cov)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // update state, without velocity
  constexpr int DIM_Y = 3;

  // fix yaw
  double estimated_yaw = getStateElement(IDX::YAW);
  double fixed_yaw = yaw;
  double limiting_delta_yaw = M_PI_2;
  while (std::fabs(estimated_yaw - fixed_yaw) > limiting_delta_yaw) {
    if (fixed_yaw < estimated_yaw) {
      fixed_yaw += 2 * limiting_delta_yaw;
    } else {
      fixed_yaw -= 2 * limiting_delta_yaw;
    }
  }

  // update state
  Eigen::MatrixXd Y(DIM_Y, 1);
  Y << x, y, fixed_yaw;

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(DIM_Y, DIM);
  C(0, IDX::X) = 1.0;
  C(1, IDX::Y) = 1.0;
  C(2, IDX::YAW) = 1.0;

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  R(0, 0) = pose_cov[XYZRPY_COV_IDX::X_X];
  R(0, 1) = pose_cov[XYZRPY_COV_IDX::X_Y];
  R(1, 0) = pose_cov[XYZRPY_COV_IDX::Y_X];
  R(1, 1) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  R(0, 2) = pose_cov[XYZRPY_COV_IDX::X_YAW];
  R(1, 2) = pose_cov[XYZRPY_COV_IDX::Y_YAW];
  R(2, 0) = pose_cov[XYZRPY_COV_IDX::YAW_X];
  R(2, 1) = pose_cov[XYZRPY_COV_IDX::YAW_Y];
  R(2, 2) = pose_cov[XYZRPY_COV_IDX::YAW_YAW];

  return ekf_.update(Y, C, R);
}

bool BicycleMotionModel::updateStatePoseHeadVel(
  const double & x, const double & y, const double & yaw, const std::array<double, 36> & pose_cov,
  const double & vel, const std::array<double, 36> & twist_cov)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // update state, with velocity
  constexpr int DIM_Y = 4;

  // fix yaw
  double estimated_yaw = getStateElement(IDX::YAW);
  double fixed_yaw = yaw;
  double limiting_delta_yaw = M_PI_2;
  while (std::fabs(estimated_yaw - fixed_yaw) > limiting_delta_yaw) {
    if (fixed_yaw < estimated_yaw) {
      fixed_yaw += 2 * limiting_delta_yaw;
    } else {
      fixed_yaw -= 2 * limiting_delta_yaw;
    }
  }

  // update state
  Eigen::MatrixXd Y(DIM_Y, 1);
  Y << x, y, yaw, vel;

  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(DIM_Y, DIM);
  C(0, IDX::X) = 1.0;
  C(1, IDX::Y) = 1.0;
  C(2, IDX::YAW) = 1.0;
  C(3, IDX::VEL) = 1.0;

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  R(0, 0) = pose_cov[XYZRPY_COV_IDX::X_X];
  R(0, 1) = pose_cov[XYZRPY_COV_IDX::X_Y];
  R(1, 0) = pose_cov[XYZRPY_COV_IDX::Y_X];
  R(1, 1) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  R(0, 2) = pose_cov[XYZRPY_COV_IDX::X_YAW];
  R(1, 2) = pose_cov[XYZRPY_COV_IDX::Y_YAW];
  R(2, 0) = pose_cov[XYZRPY_COV_IDX::YAW_X];
  R(2, 1) = pose_cov[XYZRPY_COV_IDX::YAW_Y];
  R(2, 2) = pose_cov[XYZRPY_COV_IDX::YAW_YAW];
  R(3, 3) = twist_cov[XYZRPY_COV_IDX::X_X];

  return ekf_.update(Y, C, R);
}

bool BicycleMotionModel::limitStates()
{
  Eigen::MatrixXd X_t(DIM, 1);
  Eigen::MatrixXd P_t(DIM, DIM);
  ekf_.getX(X_t);
  ekf_.getP(P_t);
  X_t(IDX::YAW) = autoware::universe_utils::normalizeRadian(X_t(IDX::YAW));
  if (!(-motion_params_.max_vel <= X_t(IDX::VEL) && X_t(IDX::VEL) <= motion_params_.max_vel)) {
    X_t(IDX::VEL) = X_t(IDX::VEL) < 0 ? -motion_params_.max_vel : motion_params_.max_vel;
  }
  if (!(-motion_params_.max_slip <= X_t(IDX::SLIP) && X_t(IDX::SLIP) <= motion_params_.max_slip)) {
    X_t(IDX::SLIP) = X_t(IDX::SLIP) < 0 ? -motion_params_.max_slip : motion_params_.max_slip;
  }
  ekf_.init(X_t, P_t);

  return true;
}

bool BicycleMotionModel::updateExtendedState(const double & length)
{
  lf_ = std::max(length * motion_params_.lf_ratio, motion_params_.lf_min);
  lr_ = std::max(length * motion_params_.lr_ratio, motion_params_.lr_min);
  return true;
}

bool BicycleMotionModel::adjustPosition(const double & x, const double & y)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  // adjust position
  Eigen::MatrixXd X_t(DIM, 1);
  Eigen::MatrixXd P_t(DIM, DIM);
  ekf_.getX(X_t);
  ekf_.getP(P_t);
  X_t(IDX::X) += x;
  X_t(IDX::Y) += y;
  ekf_.init(X_t, P_t);

  return true;
}

bool BicycleMotionModel::predictStateStep(const double dt, KalmanFilter & ekf) const
{
  /*  Motion model: static bicycle model (constant slip angle, constant velocity)
   *
   * w_k = vel_k * sin(slip_k) / l_r
   * x_{k+1}   = x_k + vel_k*cos(yaw_k+slip_k)*dt - 0.5*w_k*vel_k*sin(yaw_k+slip_k)*dt*dt
   * y_{k+1}   = y_k + vel_k*sin(yaw_k+slip_k)*dt + 0.5*w_k*vel_k*cos(yaw_k+slip_k)*dt*dt
   * yaw_{k+1} = yaw_k + w_k * dt
   * vel_{k+1}  = vel_k
   * slip_{k+1}  = slip_k
   *
   */

  /*  Jacobian Matrix
   *
   * A = [ 1, 0, -vel*sin(yaw+slip)*dt - 0.5*w_k*vel_k*cos(yaw+slip)*dt*dt,
   *  cos(yaw+slip)*dt - w*sin(yaw+slip)*dt*dt,
   * -vel*sin(yaw+slip)*dt - 0.5*vel*vel/l_r*(cos(slip)sin(yaw+slip)+sin(slip)cos(yaw+slip))*dt*dt ]
   *     [ 0, 1,  vel*cos(yaw+slip)*dt - 0.5*w_k*vel_k*sin(yaw+slip)*dt*dt,
   *  sin(yaw+slip)*dt + w*cos(yaw+slip)*dt*dt,
   *  vel*cos(yaw+slip)*dt + 0.5*vel*vel/l_r*(cos(slip)cos(yaw+slip)-sin(slip)sin(yaw+slip))*dt*dt ]
   *     [ 0, 0,  1, 1/l_r*sin(slip)*dt, vel/l_r*cos(slip)*dt]
   *     [ 0, 0,  0,                  1,                   0]
   *     [ 0, 0,  0,                  0,                   1]
   *
   */

  // Current state vector X t
  Eigen::MatrixXd X_t(DIM, 1);
  ekf.getX(X_t);

  const double cos_yaw = std::cos(X_t(IDX::YAW) + X_t(IDX::SLIP));
  const double sin_yaw = std::sin(X_t(IDX::YAW) + X_t(IDX::SLIP));
  const double vel = X_t(IDX::VEL);
  const double cos_slip = std::cos(X_t(IDX::SLIP));
  const double sin_slip = std::sin(X_t(IDX::SLIP));

  double w = vel * sin_slip / lr_;
  const double sin_2yaw = std::sin(2.0f * X_t(IDX::YAW));
  const double w_dtdt = w * dt * dt;
  const double vv_dtdt__lr = vel * vel * dt * dt / lr_;

  // Predict state vector X t+1
  Eigen::MatrixXd X_next_t(DIM, 1);  // predicted state
  X_next_t(IDX::X) =
    X_t(IDX::X) + vel * cos_yaw * dt - 0.5 * vel * sin_slip * w_dtdt;  // dx = v * cos(yaw) * dt
  X_next_t(IDX::Y) =
    X_t(IDX::Y) + vel * sin_yaw * dt + 0.5 * vel * cos_slip * w_dtdt;  // dy = v * sin(yaw) * dt
  X_next_t(IDX::YAW) = X_t(IDX::YAW) + w * dt;                         // d(yaw) = w * dt
  X_next_t(IDX::VEL) = X_t(IDX::VEL);
  X_next_t(IDX::SLIP) = X_t(IDX::SLIP);  // slip_angle = asin(lr * w / v)

  // State transition matrix A
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(DIM, DIM);
  A(IDX::X, IDX::YAW) = -vel * sin_yaw * dt - 0.5 * vel * cos_yaw * w_dtdt;
  A(IDX::X, IDX::VEL) = cos_yaw * dt - sin_yaw * w_dtdt;
  A(IDX::X, IDX::SLIP) =
    -vel * sin_yaw * dt - 0.5 * (cos_slip * sin_yaw + sin_slip * cos_yaw) * vv_dtdt__lr;
  A(IDX::Y, IDX::YAW) = vel * cos_yaw * dt - 0.5 * vel * sin_yaw * w_dtdt;
  A(IDX::Y, IDX::VEL) = sin_yaw * dt + cos_yaw * w_dtdt;
  A(IDX::Y, IDX::SLIP) =
    vel * cos_yaw * dt + 0.5 * (cos_slip * cos_yaw - sin_slip * sin_yaw) * vv_dtdt__lr;
  A(IDX::YAW, IDX::VEL) = 1.0 / lr_ * sin_slip * dt;
  A(IDX::YAW, IDX::SLIP) = vel / lr_ * cos_slip * dt;

  // Process noise covariance Q
  double q_stddev_yaw_rate = motion_params_.q_stddev_yaw_rate_min;
  if (vel > 0.01) {
    /* uncertainty of the yaw rate is limited by the following:
     *  - centripetal acceleration a_lat : d(yaw)/dt = w = a_lat/v
     *  - or maximum slip angle slip_max : w = v*sin(slip_max)/l_r
     */
    q_stddev_yaw_rate = std::min(
      motion_params_.q_stddev_acc_lat / vel,
      vel * std::sin(motion_params_.q_max_slip_angle) / lr_);  // [rad/s]
    q_stddev_yaw_rate = std::clamp(
      q_stddev_yaw_rate, motion_params_.q_stddev_yaw_rate_min,
      motion_params_.q_stddev_yaw_rate_max);
  }
  double q_cov_slip_rate{0.0};
  if (vel <= 0.01) {
    q_cov_slip_rate = motion_params_.q_cov_slip_rate_min;
  } else {
    /* The slip angle rate uncertainty is modeled as follows:
     * d(slip)/dt ~ - sin(slip)/v * d(v)/dt + l_r/v * d(w)/dt
     * where sin(slip) = w * l_r / v
     *
     * d(w)/dt is assumed to be proportional to w (more uncertain when slip is large)
     * d(v)/dt and d(w)/t are considered to be uncorrelated
     */
    q_cov_slip_rate =
      std::pow(motion_params_.q_stddev_acc_lat * sin_slip / vel, 2) + std::pow(sin_slip * 1.5, 2);
    q_cov_slip_rate = std::min(q_cov_slip_rate, motion_params_.q_cov_slip_rate_max);
    q_cov_slip_rate = std::max(q_cov_slip_rate, motion_params_.q_cov_slip_rate_min);
  }
  const double dt2 = dt * dt;
  const double dt4 = dt2 * dt2;
  const double q_cov_x = 0.25 * motion_params_.q_cov_acc_long * dt4;
  const double q_cov_y = 0.25 * motion_params_.q_cov_acc_lat * dt4;
  const double q_cov_yaw = q_stddev_yaw_rate * q_stddev_yaw_rate * dt2;
  const double q_cov_vel = motion_params_.q_cov_acc_long * dt2;
  const double q_cov_slip = q_cov_slip_rate * dt2;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(DIM, DIM);
  // Rotate the covariance matrix according to the vehicle yaw
  // because q_cov_x and y are in the vehicle coordinate system.
  Q(IDX::X, IDX::X) = (q_cov_x * cos_yaw * cos_yaw + q_cov_y * sin_yaw * sin_yaw);
  Q(IDX::X, IDX::Y) = (0.5f * (q_cov_x - q_cov_y) * sin_2yaw);
  Q(IDX::Y, IDX::Y) = (q_cov_x * sin_yaw * sin_yaw + q_cov_y * cos_yaw * cos_yaw);
  Q(IDX::Y, IDX::X) = Q(IDX::X, IDX::Y);
  Q(IDX::YAW, IDX::YAW) = q_cov_yaw;
  Q(IDX::VEL, IDX::VEL) = q_cov_vel;
  Q(IDX::SLIP, IDX::SLIP) = q_cov_slip;

  // control-input model B and control-input u are not used
  // Eigen::MatrixXd B = Eigen::MatrixXd::Zero(DIM, DIM);
  // Eigen::MatrixXd u = Eigen::MatrixXd::Zero(DIM, 1);

  // predict state
  return ekf.predict(X_next_t, A, Q);
}

bool BicycleMotionModel::getPredictedState(
  const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
  geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const
{
  // get predicted state
  Eigen::MatrixXd X(DIM, 1);
  Eigen::MatrixXd P(DIM, DIM);
  if (!MotionModel::getPredictedState(time, X, P)) {
    return false;
  }

  // set position
  pose.position.x = X(IDX::X);
  pose.position.y = X(IDX::Y);
  pose.position.z = 0.0;

  // set orientation
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, X(IDX::YAW));
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();

  // set twist
  twist.linear.x = X(IDX::VEL) * std::cos(X(IDX::SLIP));
  twist.linear.y = X(IDX::VEL) * std::sin(X(IDX::SLIP));
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = twist.linear.y / lr_;

  // set pose covariance
  constexpr double zz_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double rr_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double pp_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  pose_cov[XYZRPY_COV_IDX::X_X] = P(IDX::X, IDX::X);
  pose_cov[XYZRPY_COV_IDX::X_Y] = P(IDX::X, IDX::Y);
  pose_cov[XYZRPY_COV_IDX::Y_X] = P(IDX::Y, IDX::X);
  pose_cov[XYZRPY_COV_IDX::Y_Y] = P(IDX::Y, IDX::Y);
  pose_cov[XYZRPY_COV_IDX::YAW_YAW] = P(IDX::YAW, IDX::YAW);
  pose_cov[XYZRPY_COV_IDX::Z_Z] = zz_cov;
  pose_cov[XYZRPY_COV_IDX::ROLL_ROLL] = rr_cov;
  pose_cov[XYZRPY_COV_IDX::PITCH_PITCH] = pp_cov;

  // set twist covariance
  Eigen::MatrixXd cov_jacob(3, 2);
  cov_jacob << std::cos(X(IDX::SLIP)), -X(IDX::VEL) * std::sin(X(IDX::SLIP)),
    std::sin(X(IDX::SLIP)), X(IDX::VEL) * std::cos(X(IDX::SLIP)), std::sin(X(IDX::SLIP)) / lr_,
    X(IDX::VEL) * std::cos(X(IDX::SLIP)) / lr_;
  Eigen::MatrixXd cov_twist(2, 2);
  cov_twist << P(IDX::VEL, IDX::VEL), P(IDX::VEL, IDX::SLIP), P(IDX::SLIP, IDX::VEL),
    P(IDX::SLIP, IDX::SLIP);
  Eigen::MatrixXd twist_cov_mat = cov_jacob * cov_twist * cov_jacob.transpose();
  constexpr double vz_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double wx_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double wy_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  twist_cov[XYZRPY_COV_IDX::X_X] = twist_cov_mat(0, 0);
  twist_cov[XYZRPY_COV_IDX::X_Y] = twist_cov_mat(0, 1);
  twist_cov[XYZRPY_COV_IDX::X_YAW] = twist_cov_mat(0, 2);
  twist_cov[XYZRPY_COV_IDX::Y_X] = twist_cov_mat(1, 0);
  twist_cov[XYZRPY_COV_IDX::Y_Y] = twist_cov_mat(1, 1);
  twist_cov[XYZRPY_COV_IDX::Y_YAW] = twist_cov_mat(1, 2);
  twist_cov[XYZRPY_COV_IDX::YAW_X] = twist_cov_mat(2, 0);
  twist_cov[XYZRPY_COV_IDX::YAW_Y] = twist_cov_mat(2, 1);
  twist_cov[XYZRPY_COV_IDX::YAW_YAW] = twist_cov_mat(2, 2);
  twist_cov[XYZRPY_COV_IDX::Z_Z] = vz_cov;
  twist_cov[XYZRPY_COV_IDX::ROLL_ROLL] = wx_cov;
  twist_cov[XYZRPY_COV_IDX::PITCH_PITCH] = wy_cov;

  return true;
}
