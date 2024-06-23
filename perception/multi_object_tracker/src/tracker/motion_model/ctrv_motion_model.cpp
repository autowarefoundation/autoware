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

#include "multi_object_tracker/tracker/motion_model/ctrv_motion_model.hpp"

#include "autoware/universe_utils/math/normalization.hpp"
#include "autoware/universe_utils/math/unit_conversion.hpp"
#include "autoware/universe_utils/ros/msg_covariance.hpp"
#include "multi_object_tracker/tracker/motion_model/motion_model_base.hpp"
#include "multi_object_tracker/utils/utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

// cspell: ignore CTRV
// Constant Turn Rate and constant Velocity (CTRV) motion model
using autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

CTRVMotionModel::CTRVMotionModel() : logger_(rclcpp::get_logger("CTRVMotionModel"))
{
  // Initialize motion parameters
  setDefaultParams();
}

void CTRVMotionModel::setDefaultParams()
{
  // process noise covariance
  constexpr double q_stddev_x = 0.5;                                      // [m/s]
  constexpr double q_stddev_y = 0.5;                                      // [m/s]
  constexpr double q_stddev_yaw = autoware::universe_utils::deg2rad(20);  // [rad/s]
  constexpr double q_stddev_vel = 9.8 * 0.3;                              // [m/(s*s)]
  constexpr double q_stddev_wz = autoware::universe_utils::deg2rad(30);   // [rad/(s*s)]

  setMotionParams(q_stddev_x, q_stddev_y, q_stddev_yaw, q_stddev_vel, q_stddev_wz);

  // set motion limitations
  constexpr double max_vel = autoware::universe_utils::kmph2mps(10);  // [m/s] maximum velocity
  constexpr double max_wz = 30.0;                                     // [deg] maximum yaw rate
  setMotionLimits(max_vel, max_wz);

  // set prediction parameters
  constexpr double dt_max = 0.11;  // [s] maximum time interval for prediction
  setMaxDeltaTime(dt_max);
}

void CTRVMotionModel::setMotionParams(
  const double & q_stddev_x, const double & q_stddev_y, const double & q_stddev_yaw,
  const double & q_stddev_vel, const double & q_stddev_wz)
{
  // set process noise covariance parameters
  motion_params_.q_cov_x = q_stddev_x * q_stddev_x;
  motion_params_.q_cov_y = q_stddev_y * q_stddev_y;
  motion_params_.q_cov_yaw = q_stddev_yaw * q_stddev_yaw;
  motion_params_.q_cov_vel = q_stddev_vel * q_stddev_vel;
  motion_params_.q_cov_wz = q_stddev_wz * q_stddev_wz;
}

void CTRVMotionModel::setMotionLimits(const double & max_vel, const double & max_wz)
{
  // set motion limitations
  motion_params_.max_vel = max_vel;
  motion_params_.max_wz = autoware::universe_utils::deg2rad(max_wz);
}

bool CTRVMotionModel::initialize(
  const rclcpp::Time & time, const double & x, const double & y, const double & yaw,
  const std::array<double, 36> & pose_cov, const double & vel, const double & vel_cov,
  const double & wz, const double & wz_cov)
{
  // initialize state vector X
  Eigen::MatrixXd X(DIM, 1);
  X << x, y, yaw, vel, wz;

  // initialize covariance matrix P
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(DIM, DIM);
  P(IDX::X, IDX::X) = pose_cov[XYZRPY_COV_IDX::X_X];
  P(IDX::Y, IDX::Y) = pose_cov[XYZRPY_COV_IDX::Y_Y];
  P(IDX::YAW, IDX::YAW) = pose_cov[XYZRPY_COV_IDX::YAW_YAW];
  P(IDX::VEL, IDX::VEL) = vel_cov;
  P(IDX::WZ, IDX::WZ) = wz_cov;

  return MotionModel::initialize(time, X, P);
}

bool CTRVMotionModel::updateStatePose(
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

bool CTRVMotionModel::updateStatePoseHead(
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

bool CTRVMotionModel::updateStatePoseHeadVel(
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

bool CTRVMotionModel::limitStates()
{
  Eigen::MatrixXd X_t(DIM, 1);
  Eigen::MatrixXd P_t(DIM, DIM);
  ekf_.getX(X_t);
  ekf_.getP(P_t);
  X_t(IDX::YAW) = autoware::universe_utils::normalizeRadian(X_t(IDX::YAW));
  if (!(-motion_params_.max_vel <= X_t(IDX::VEL) && X_t(IDX::VEL) <= motion_params_.max_vel)) {
    X_t(IDX::VEL) = X_t(IDX::VEL) < 0 ? -motion_params_.max_vel : motion_params_.max_vel;
  }
  if (!(-motion_params_.max_wz <= X_t(IDX::WZ) && X_t(IDX::WZ) <= motion_params_.max_wz)) {
    X_t(IDX::WZ) = X_t(IDX::WZ) < 0 ? -motion_params_.max_wz : motion_params_.max_wz;
  }
  ekf_.init(X_t, P_t);

  return true;
}

bool CTRVMotionModel::adjustPosition(const double & x, const double & y)
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

bool CTRVMotionModel::predictStateStep(const double dt, KalmanFilter & ekf) const
{
  /*  Motion model: Constant Turn Rate and constant Velocity model (CTRV)
   *
   * x_{k+1}   = x_k + vx_k * cos(yaw_k) * dt
   * y_{k+1}   = y_k + vx_k * sin(yaw_k) * dt
   * yaw_{k+1} = yaw_k + (wz_k) * dt
   * vx_{k+1}  = vx_k
   * wz_{k+1}  = wz_k
   *
   */

  /*  Jacobian Matrix
   *
   * A = [ 1, 0, -vx*sin(yaw)*dt, cos(yaw)*dt,  0]
   *     [ 0, 1,  vx*cos(yaw)*dt, sin(yaw)*dt,  0]
   *     [ 0, 0,               1,           0, dt]
   *     [ 0, 0,               0,           1,  0]
   *     [ 0, 0,               0,           0,  1]
   */

  // Current state vector X t
  Eigen::MatrixXd X_t(DIM, 1);
  ekf.getX(X_t);

  const double cos_yaw = std::cos(X_t(IDX::YAW));
  const double sin_yaw = std::sin(X_t(IDX::YAW));
  const double sin_2yaw = std::sin(2.0f * X_t(IDX::YAW));

  // Predict state vector X t+1
  Eigen::MatrixXd X_next_t(DIM, 1);                               // predicted state
  X_next_t(IDX::X) = X_t(IDX::X) + X_t(IDX::VEL) * cos_yaw * dt;  // dx = v * cos(yaw)
  X_next_t(IDX::Y) = X_t(IDX::Y) + X_t(IDX::VEL) * sin_yaw * dt;  // dy = v * sin(yaw)
  X_next_t(IDX::YAW) = X_t(IDX::YAW) + (X_t(IDX::WZ)) * dt;       // dyaw = omega
  X_next_t(IDX::VEL) = X_t(IDX::VEL);
  X_next_t(IDX::WZ) = X_t(IDX::WZ);

  // State transition matrix A
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(DIM, DIM);
  A(IDX::X, IDX::YAW) = -X_t(IDX::VEL) * sin_yaw * dt;
  A(IDX::X, IDX::VEL) = cos_yaw * dt;
  A(IDX::Y, IDX::YAW) = X_t(IDX::VEL) * cos_yaw * dt;
  A(IDX::Y, IDX::VEL) = sin_yaw * dt;
  A(IDX::YAW, IDX::WZ) = dt;

  // Process noise covariance Q
  const double dt2 = dt * dt;
  const double q_cov_x = motion_params_.q_cov_x * dt2;
  const double q_cov_y = motion_params_.q_cov_y * dt2;
  const double q_cov_yaw = motion_params_.q_cov_yaw * dt2;
  const double q_cov_vel = motion_params_.q_cov_vel * dt2;
  const double q_cov_wz = motion_params_.q_cov_wz * dt2;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(DIM, DIM);
  // Rotate the covariance matrix according to the vehicle yaw
  // because q_cov_x and y are in the vehicle coordinate system.
  Q(IDX::X, IDX::X) = (q_cov_x * cos_yaw * cos_yaw + q_cov_y * sin_yaw * sin_yaw);
  Q(IDX::X, IDX::Y) = (0.5f * (q_cov_x - q_cov_y) * sin_2yaw);
  Q(IDX::Y, IDX::Y) = (q_cov_x * sin_yaw * sin_yaw + q_cov_y * cos_yaw * cos_yaw);
  Q(IDX::Y, IDX::X) = Q(IDX::X, IDX::Y);
  Q(IDX::YAW, IDX::YAW) = q_cov_yaw;
  Q(IDX::VEL, IDX::VEL) = q_cov_vel;
  Q(IDX::WZ, IDX::WZ) = q_cov_wz;

  // control-input model B and control-input u are not used
  // Eigen::MatrixXd B = Eigen::MatrixXd::Zero(DIM, DIM);
  // Eigen::MatrixXd u = Eigen::MatrixXd::Zero(DIM, 1);

  // predict state
  return ekf.predict(X_next_t, A, Q);
}

bool CTRVMotionModel::getPredictedState(
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
  twist.linear.x = X(IDX::VEL);
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = X(IDX::WZ);

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
  constexpr double vy_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double vz_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double wx_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double wy_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  twist_cov[XYZRPY_COV_IDX::X_X] = P(IDX::VEL, IDX::VEL);
  twist_cov[XYZRPY_COV_IDX::X_Y] = 0.0;
  twist_cov[XYZRPY_COV_IDX::X_YAW] = P(IDX::VEL, IDX::WZ);
  twist_cov[XYZRPY_COV_IDX::Y_X] = 0.0;
  twist_cov[XYZRPY_COV_IDX::Y_Y] = vy_cov;
  twist_cov[XYZRPY_COV_IDX::Y_YAW] = 0.0;
  twist_cov[XYZRPY_COV_IDX::YAW_X] = P(IDX::WZ, IDX::VEL);
  twist_cov[XYZRPY_COV_IDX::YAW_Y] = 0.0;
  twist_cov[XYZRPY_COV_IDX::YAW_YAW] = P(IDX::WZ, IDX::WZ);
  twist_cov[XYZRPY_COV_IDX::Z_Z] = vz_cov;
  twist_cov[XYZRPY_COV_IDX::ROLL_ROLL] = wx_cov;
  twist_cov[XYZRPY_COV_IDX::PITCH_PITCH] = wy_cov;

  return true;
}
