// Copyright 2020 Tier IV, Inc.
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

#include <bits/stdc++.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#define EIGEN_MPL2_ONLY
#include "multi_object_tracker/tracker/model/unknown_tracker.hpp"
#include "multi_object_tracker/utils/utils.hpp"
#include "object_recognition_utils/object_recognition_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

UnknownTracker::UnknownTracker(
  const rclcpp::Time & time, const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & /*self_transform*/)
: Tracker(time, object.classification),
  logger_(rclcpp::get_logger("UnknownTracker")),
  last_update_time_(time),
  z_(object.kinematics.pose_with_covariance.pose.position.z)
{
  object_ = object;

  // initialize params
  float q_stddev_x = 0.0;                                   // [m/s]
  float q_stddev_y = 0.0;                                   // [m/s]
  float q_stddev_vx = tier4_autoware_utils::kmph2mps(20);   // [m/(s*s)]
  float q_stddev_vy = tier4_autoware_utils::kmph2mps(20);   // [m/(s*s)]
  float r_stddev_x = 1.0;                                   // [m]
  float r_stddev_y = 1.0;                                   // [m]
  float p0_stddev_x = 1.0;                                  // [m/s]
  float p0_stddev_y = 1.0;                                  // [m/s]
  float p0_stddev_vx = tier4_autoware_utils::kmph2mps(10);  // [m/(s*s)]
  float p0_stddev_vy = tier4_autoware_utils::kmph2mps(10);  // [m/(s*s)]
  ekf_params_.q_cov_x = std::pow(q_stddev_x, 2.0);
  ekf_params_.q_cov_y = std::pow(q_stddev_y, 2.0);
  ekf_params_.q_cov_vx = std::pow(q_stddev_vx, 2.0);
  ekf_params_.q_cov_vy = std::pow(q_stddev_vy, 2.0);
  ekf_params_.r_cov_x = std::pow(r_stddev_x, 2.0);
  ekf_params_.r_cov_y = std::pow(r_stddev_y, 2.0);
  ekf_params_.p0_cov_x = std::pow(p0_stddev_x, 2.0);
  ekf_params_.p0_cov_y = std::pow(p0_stddev_y, 2.0);
  ekf_params_.p0_cov_vx = std::pow(p0_stddev_vx, 2.0);
  ekf_params_.p0_cov_vy = std::pow(p0_stddev_vy, 2.0);
  max_vx_ = tier4_autoware_utils::kmph2mps(60);  // [m/s]
  max_vy_ = tier4_autoware_utils::kmph2mps(60);  // [m/s]

  // initialize X matrix
  Eigen::MatrixXd X(ekf_params_.dim_x, 1);
  X(IDX::X) = object.kinematics.pose_with_covariance.pose.position.x;
  X(IDX::Y) = object.kinematics.pose_with_covariance.pose.position.y;
  if (object.kinematics.has_twist) {
    X(IDX::VX) = object.kinematics.twist_with_covariance.twist.linear.x;
    X(IDX::VY) = object.kinematics.twist_with_covariance.twist.linear.y;
  } else {
    X(IDX::VX) = 0.0;
    X(IDX::VY) = 0.0;
  }

  // initialize P matrix
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  if (!object.kinematics.has_position_covariance) {
    // Rotate the covariance matrix according to the vehicle yaw
    // because p0_cov_x and y are in the vehicle coordinate system.
    P(IDX::X, IDX::X) = ekf_params_.p0_cov_x;
    P(IDX::X, IDX::Y) = 0.0;
    P(IDX::Y, IDX::Y) = ekf_params_.p0_cov_y;
    P(IDX::Y, IDX::X) = P(IDX::X, IDX::Y);
    P(IDX::VX, IDX::VX) = ekf_params_.p0_cov_vx;
    P(IDX::VY, IDX::VY) = ekf_params_.p0_cov_vy;
  } else {
    P(IDX::X, IDX::X) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_X];
    P(IDX::X, IDX::Y) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_Y];
    P(IDX::Y, IDX::Y) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
    P(IDX::Y, IDX::X) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_X];
    if (object.kinematics.has_twist_covariance) {
      P(IDX::VX, IDX::VX) =
        object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::X_X];
      P(IDX::VY, IDX::VY) =
        object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
    } else {
      P(IDX::VX, IDX::VX) = ekf_params_.p0_cov_vx;
      P(IDX::VY, IDX::VY) = ekf_params_.p0_cov_vy;
    }
  }

  ekf_.init(X, P);
}

bool UnknownTracker::predict(const rclcpp::Time & time)
{
  const double dt = (time - last_update_time_).seconds();
  bool ret = predict(dt, ekf_);
  if (ret) {
    last_update_time_ = time;
  }
  return ret;
}

bool UnknownTracker::predict(const double dt, KalmanFilter & ekf) const
{
  /*  == Nonlinear model ==
   *
   * x_{k+1}   = x_k + vx_k * dt
   * y_{k+1}   = y_k + vx_k * dt
   * vx_{k+1}  = vx_k
   * vy_{k+1}  = vy_k
   *
   */

  /*  == Linearized model ==
   *
   * A = [ 1, 0, dt,  0]
   *     [ 0, 1,  0, dt]
   *     [ 0, 0,  1,  0]
   *     [ 0, 0,  0,  1]
   */

  // X t
  Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);  // predicted state
  ekf.getX(X_t);

  // X t+1
  Eigen::MatrixXd X_next_t(ekf_params_.dim_x, 1);  // predicted state
  X_next_t(IDX::X) = X_t(IDX::X) + X_t(IDX::VX) * dt;
  X_next_t(IDX::Y) = X_t(IDX::Y) + X_t(IDX::VY) * dt;
  X_next_t(IDX::VX) = X_t(IDX::VX);
  X_next_t(IDX::VY) = X_t(IDX::VY);

  // A
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(ekf_params_.dim_x, ekf_params_.dim_x);
  A(IDX::X, IDX::VX) = dt;
  A(IDX::Y, IDX::VY) = dt;

  // Q
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  // Rotate the covariance matrix according to the vehicle yaw
  // because q_cov_x and y are in the vehicle coordinate system.
  Q(IDX::X, IDX::X) = ekf_params_.q_cov_x * dt * dt;
  Q(IDX::X, IDX::Y) = 0.0;
  Q(IDX::Y, IDX::Y) = ekf_params_.q_cov_y * dt * dt;
  Q(IDX::Y, IDX::X) = Q(IDX::X, IDX::Y);
  Q(IDX::VX, IDX::VX) = ekf_params_.q_cov_vx * dt * dt;
  Q(IDX::VY, IDX::VY) = ekf_params_.q_cov_vy * dt * dt;
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  Eigen::MatrixXd u = Eigen::MatrixXd::Zero(ekf_params_.dim_x, 1);

  if (!ekf.predict(X_next_t, A, Q)) {
    RCLCPP_WARN(logger_, "Pedestrian : Cannot predict");
  }

  return true;
}

bool UnknownTracker::measureWithPose(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  constexpr int dim_y = 2;  // pos x, pos y depending on Pose output

  /* Set measurement matrix */
  Eigen::MatrixXd Y(dim_y, 1);
  Y << object.kinematics.pose_with_covariance.pose.position.x,
    object.kinematics.pose_with_covariance.pose.position.y;

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, ekf_params_.dim_x);
  C(0, IDX::X) = 1.0;  // for pos x
  C(1, IDX::Y) = 1.0;  // for pos y

  /* Set measurement noise covariance */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (!object.kinematics.has_position_covariance) {
    R(0, 0) = ekf_params_.r_cov_x;  // x - x
    R(0, 1) = 0.0;                  // x - y
    R(1, 1) = ekf_params_.r_cov_y;  // y - y
    R(1, 0) = R(0, 1);              // y - x
  } else {
    R(0, 0) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_X];
    R(0, 1) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_Y];
    R(1, 0) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_X];
    R(1, 1) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
  }
  if (!ekf_.update(Y, C, R)) {
    RCLCPP_WARN(logger_, "Pedestrian : Cannot update");
  }

  // limit vx, vy
  {
    Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);
    Eigen::MatrixXd P_t(ekf_params_.dim_x, ekf_params_.dim_x);
    ekf_.getX(X_t);
    ekf_.getP(P_t);
    if (!(-max_vx_ <= X_t(IDX::VX) && X_t(IDX::VX) <= max_vx_)) {
      X_t(IDX::VX) = X_t(IDX::VX) < 0 ? -max_vx_ : max_vx_;
    }
    if (!(-max_vy_ <= X_t(IDX::VY) && X_t(IDX::VY) <= max_vy_)) {
      X_t(IDX::VY) = X_t(IDX::VY) < 0 ? -max_vy_ : max_vy_;
    }
    ekf_.init(X_t, P_t);
  }

  // position z
  constexpr float gain = 0.9;
  z_ = gain * z_ + (1.0 - gain) * object.kinematics.pose_with_covariance.pose.position.z;

  return true;
}

bool UnknownTracker::measure(
  const autoware_auto_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & /*self_transform*/)
{
  object_ = object;

  if (0.01 /*10msec*/ < std::fabs((time - last_update_time_).seconds())) {
    RCLCPP_WARN(
      logger_,
      "Pedestrian : There is a large gap between predicted time and measurement time. (%f)",
      (time - last_update_time_).seconds());
  }

  measureWithPose(object);

  return true;
}

bool UnknownTracker::getTrackedObject(
  const rclcpp::Time & time, autoware_auto_perception_msgs::msg::TrackedObject & object) const
{
  object = object_recognition_utils::toTrackedObject(object_);
  object.object_id = getUUID();
  object.classification = getClassification();

  // predict kinematics
  KalmanFilter tmp_ekf_for_no_update = ekf_;
  const double dt = (time - last_update_time_).seconds();
  if (0.001 /*1msec*/ < dt) {
    predict(dt, tmp_ekf_for_no_update);
  }
  Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);                // predicted state
  Eigen::MatrixXd P(ekf_params_.dim_x, ekf_params_.dim_x);  // predicted state
  tmp_ekf_for_no_update.getX(X_t);
  tmp_ekf_for_no_update.getP(P);

  auto & pose_with_cov = object.kinematics.pose_with_covariance;
  auto & twist_with_cov = object.kinematics.twist_with_covariance;
  // position
  pose_with_cov.pose.position.x = X_t(IDX::X);
  pose_with_cov.pose.position.y = X_t(IDX::Y);
  pose_with_cov.pose.position.z = z_;
  constexpr double z_cov = 0.1 * 0.1;    // TODO(yukkysaito) Currently tentative
  constexpr double r_cov = 0.1 * 0.1;    // TODO(yukkysaito) Currently tentative
  constexpr double p_cov = 0.1 * 0.1;    // TODO(yukkysaito) Currently tentative
  constexpr double yaw_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  pose_with_cov.covariance[utils::MSG_COV_IDX::X_X] = P(IDX::X, IDX::X);
  pose_with_cov.covariance[utils::MSG_COV_IDX::X_Y] = P(IDX::X, IDX::Y);
  pose_with_cov.covariance[utils::MSG_COV_IDX::Y_X] = P(IDX::Y, IDX::X);
  pose_with_cov.covariance[utils::MSG_COV_IDX::Y_Y] = P(IDX::Y, IDX::Y);
  pose_with_cov.covariance[utils::MSG_COV_IDX::Z_Z] = z_cov;
  pose_with_cov.covariance[utils::MSG_COV_IDX::ROLL_ROLL] = r_cov;
  pose_with_cov.covariance[utils::MSG_COV_IDX::PITCH_PITCH] = p_cov;
  pose_with_cov.covariance[utils::MSG_COV_IDX::YAW_YAW] = yaw_cov;

  // twist
  const auto pose_yaw = tf2::getYaw(pose_with_cov.pose.orientation);
  const double cos = std::cos(-pose_yaw);
  const double sin = std::sin(-pose_yaw);
  twist_with_cov.twist.linear.x = cos * X_t(IDX::VX) - sin * X_t(IDX::VY);
  twist_with_cov.twist.linear.y = sin * X_t(IDX::VX) + cos * X_t(IDX::VY);

  // twist covariance
  constexpr double vz_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double wx_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double wy_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double wz_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  twist_with_cov.covariance[utils::MSG_COV_IDX::X_X] = P(IDX::VX, IDX::VX);
  twist_with_cov.covariance[utils::MSG_COV_IDX::Y_Y] = P(IDX::VY, IDX::VY);
  twist_with_cov.covariance[utils::MSG_COV_IDX::Z_Z] = vz_cov;
  twist_with_cov.covariance[utils::MSG_COV_IDX::ROLL_ROLL] = wx_cov;
  twist_with_cov.covariance[utils::MSG_COV_IDX::PITCH_PITCH] = wy_cov;
  twist_with_cov.covariance[utils::MSG_COV_IDX::YAW_YAW] = wz_cov;

  return true;
}
