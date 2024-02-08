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
//
//
// Author: v1.0 Yukihiro Saito
//

#include "multi_object_tracker/tracker/model/normal_vehicle_tracker.hpp"

#include "multi_object_tracker/utils/utils.hpp"

#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <bits/stdc++.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include "object_recognition_utils/object_recognition_utils.hpp"

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

NormalVehicleTracker::NormalVehicleTracker(
  const rclcpp::Time & time, const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & self_transform)
: Tracker(time, object.classification),
  logger_(rclcpp::get_logger("NormalVehicleTracker")),
  last_update_time_(time),
  z_(object.kinematics.pose_with_covariance.pose.position.z),
  tracking_offset_(Eigen::Vector2d::Zero())
{
  object_ = object;

  // Initialize parameters
  // measurement noise covariance: detector uncertainty + ego vehicle motion uncertainty
  float r_stddev_x = 0.5;                                  // in vehicle coordinate [m]
  float r_stddev_y = 0.4;                                  // in vehicle coordinate [m]
  float r_stddev_yaw = tier4_autoware_utils::deg2rad(20);  // in map coordinate [rad]
  float r_stddev_vel = 1.0;                                // in object coordinate [m/s]
  ekf_params_.r_cov_x = std::pow(r_stddev_x, 2.0);
  ekf_params_.r_cov_y = std::pow(r_stddev_y, 2.0);
  ekf_params_.r_cov_yaw = std::pow(r_stddev_yaw, 2.0);
  ekf_params_.r_cov_vel = std::pow(r_stddev_vel, 2.0);
  // initial state covariance
  float p0_stddev_x = 1.0;                                     // in object coordinate [m]
  float p0_stddev_y = 0.3;                                     // in object coordinate [m]
  float p0_stddev_yaw = tier4_autoware_utils::deg2rad(25);     // in map coordinate [rad]
  float p0_stddev_vel = tier4_autoware_utils::kmph2mps(1000);  // in object coordinate [m/s]
  float p0_stddev_slip = tier4_autoware_utils::deg2rad(5);     // in object coordinate [rad/s]
  ekf_params_.p0_cov_x = std::pow(p0_stddev_x, 2.0);
  ekf_params_.p0_cov_y = std::pow(p0_stddev_y, 2.0);
  ekf_params_.p0_cov_yaw = std::pow(p0_stddev_yaw, 2.0);
  ekf_params_.p0_cov_vel = std::pow(p0_stddev_vel, 2.0);
  ekf_params_.p0_cov_slip = std::pow(p0_stddev_slip, 2.0);
  // process noise covariance
  ekf_params_.q_stddev_acc_long = 9.8 * 0.35;  // [m/(s*s)] uncertain longitudinal acceleration
  ekf_params_.q_stddev_acc_lat = 9.8 * 0.15;   // [m/(s*s)] uncertain lateral acceleration
  ekf_params_.q_stddev_yaw_rate_min =
    tier4_autoware_utils::deg2rad(1.5);  // [rad/s] uncertain yaw change rate
  ekf_params_.q_stddev_yaw_rate_max =
    tier4_autoware_utils::deg2rad(15.0);  // [rad/s] uncertain yaw change rate
  float q_stddev_slip_rate_min =
    tier4_autoware_utils::deg2rad(0.3);  // [rad/s] uncertain slip angle change rate
  float q_stddev_slip_rate_max =
    tier4_autoware_utils::deg2rad(10.0);  // [rad/s] uncertain slip angle change rate
  ekf_params_.q_cov_slip_rate_min = std::pow(q_stddev_slip_rate_min, 2.0);
  ekf_params_.q_cov_slip_rate_max = std::pow(q_stddev_slip_rate_max, 2.0);
  ekf_params_.q_max_slip_angle = tier4_autoware_utils::deg2rad(30);  // [rad] max slip angle
  // limitations
  max_vel_ = tier4_autoware_utils::kmph2mps(100);                      // [m/s]
  max_slip_ = tier4_autoware_utils::deg2rad(30);                       // [rad]
  velocity_deviation_threshold_ = tier4_autoware_utils::kmph2mps(10);  // [m/s]

  // initialize state vector X
  Eigen::MatrixXd X(ekf_params_.dim_x, 1);
  X(IDX::X) = object.kinematics.pose_with_covariance.pose.position.x;
  X(IDX::Y) = object.kinematics.pose_with_covariance.pose.position.y;
  X(IDX::YAW) = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  X(IDX::SLIP) = 0.0;
  if (object.kinematics.has_twist) {
    X(IDX::VEL) = object.kinematics.twist_with_covariance.twist.linear.x;
  } else {
    X(IDX::VEL) = 0.0;
  }

  // initialize state covariance matrix P
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  if (!object.kinematics.has_position_covariance) {
    const double cos_yaw = std::cos(X(IDX::YAW));
    const double sin_yaw = std::sin(X(IDX::YAW));
    const double sin_2yaw = std::sin(2.0f * X(IDX::YAW));
    // Rotate the covariance matrix according to the vehicle yaw
    // because p0_cov_x and y are in the vehicle coordinate system.
    P(IDX::X, IDX::X) =
      ekf_params_.p0_cov_x * cos_yaw * cos_yaw + ekf_params_.p0_cov_y * sin_yaw * sin_yaw;
    P(IDX::X, IDX::Y) = 0.5f * (ekf_params_.p0_cov_x - ekf_params_.p0_cov_y) * sin_2yaw;
    P(IDX::Y, IDX::Y) =
      ekf_params_.p0_cov_x * sin_yaw * sin_yaw + ekf_params_.p0_cov_y * cos_yaw * cos_yaw;
    P(IDX::Y, IDX::X) = P(IDX::X, IDX::Y);
    P(IDX::YAW, IDX::YAW) = ekf_params_.p0_cov_yaw;
    P(IDX::VEL, IDX::VEL) = ekf_params_.p0_cov_vel;
    P(IDX::SLIP, IDX::SLIP) = ekf_params_.p0_cov_slip;
  } else {
    P(IDX::X, IDX::X) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_X];
    P(IDX::X, IDX::Y) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_Y];
    P(IDX::Y, IDX::Y) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
    P(IDX::Y, IDX::X) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_X];
    P(IDX::YAW, IDX::YAW) =
      object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW];
    if (object.kinematics.has_twist_covariance) {
      P(IDX::VEL, IDX::VEL) =
        object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::X_X];
    } else {
      P(IDX::VEL, IDX::VEL) = ekf_params_.p0_cov_vel;
    }
    P(IDX::SLIP, IDX::SLIP) = ekf_params_.p0_cov_slip;
  }

  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    bounding_box_ = {
      object.shape.dimensions.x, object.shape.dimensions.y, object.shape.dimensions.z};
    last_input_bounding_box_ = bounding_box_;
  } else {
    // past default value
    // bounding_box_ = {4.0, 1.7, 2.0};
    autoware_auto_perception_msgs::msg::DetectedObject bbox_object;
    utils::convertConvexHullToBoundingBox(object, bbox_object);
    bounding_box_ = {
      bbox_object.shape.dimensions.x, bbox_object.shape.dimensions.y,
      bbox_object.shape.dimensions.z};
    last_input_bounding_box_ = bounding_box_;
  }
  ekf_.init(X, P);

  /* calc nearest corner index*/
  setNearestCornerOrSurfaceIndex(self_transform);  // this index is used in next measure step

  // Set lf, lr
  lf_ = std::max(bounding_box_.length * 0.3, 1.0);   // 30% front from the center, minimum of 1.0m
  lr_ = std::max(bounding_box_.length * 0.25, 1.0);  // 25% rear from the center, minimum of 1.0m
}

bool NormalVehicleTracker::predict(const rclcpp::Time & time)
{
  const double dt = (time - last_update_time_).seconds();
  if (dt < 0.0) {
    RCLCPP_WARN(logger_, "dt is negative. (%f)", dt);
    return false;
  }
  // if dt is too large, shorten dt and repeat prediction
  const double dt_max = 0.11;
  const uint32_t repeat = std::ceil(dt / dt_max);
  const double dt_ = dt / repeat;
  bool ret = false;
  for (uint32_t i = 0; i < repeat; ++i) {
    ret = predict(dt_, ekf_);
    if (!ret) {
      return false;
    }
  }
  if (ret) {
    last_update_time_ = time;
  }
  return ret;
}

bool NormalVehicleTracker::predict(const double dt, KalmanFilter & ekf) const
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
  Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);  // predicted state
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
  Eigen::MatrixXd X_next_t(ekf_params_.dim_x, 1);  // predicted state
  X_next_t(IDX::X) =
    X_t(IDX::X) + vel * cos_yaw * dt - 0.5 * vel * sin_slip * w_dtdt;  // dx = v * cos(yaw) * dt
  X_next_t(IDX::Y) =
    X_t(IDX::Y) + vel * sin_yaw * dt + 0.5 * vel * cos_slip * w_dtdt;  // dy = v * sin(yaw) * dt
  X_next_t(IDX::YAW) = X_t(IDX::YAW) + w * dt;                         // d(yaw) = w * dt
  X_next_t(IDX::VEL) = X_t(IDX::VEL);
  X_next_t(IDX::SLIP) = X_t(IDX::SLIP);  // slip_angle = asin(lr * w / v)

  // State transition matrix A
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(ekf_params_.dim_x, ekf_params_.dim_x);
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
  float q_stddev_yaw_rate{0.0};
  if (vel <= 0.01) {
    q_stddev_yaw_rate = ekf_params_.q_stddev_yaw_rate_min;
  } else {
    // uncertainty of the yaw rate is limited by
    // centripetal acceleration a_lat : d(yaw)/dt = w = a_lat/v
    // or maximum slip angle slip_max : w = v*sin(slip_max)/l_r
    q_stddev_yaw_rate = std::min(
      ekf_params_.q_stddev_acc_lat / vel,
      vel * std::sin(ekf_params_.q_max_slip_angle) / lr_);  // [rad/s]
    q_stddev_yaw_rate = std::min(q_stddev_yaw_rate, ekf_params_.q_stddev_yaw_rate_max);
    q_stddev_yaw_rate = std::max(q_stddev_yaw_rate, ekf_params_.q_stddev_yaw_rate_min);
  }
  float q_cov_slip_rate{0.0f};
  if (vel <= 0.01) {
    q_cov_slip_rate = ekf_params_.q_cov_slip_rate_min;
  } else {
    // The slip angle rate uncertainty is modeled as follows:
    // d(slip)/dt ~ - sin(slip)/v * d(v)/dt + l_r/v * d(w)/dt
    // where sin(slip) = w * l_r / v
    // d(w)/dt is assumed to be proportional to w (more uncertain when slip is large)
    // d(v)/dt and d(w)/t are considered to be uncorrelated
    q_cov_slip_rate =
      std::pow(ekf_params_.q_stddev_acc_lat * sin_slip / vel, 2) + std::pow(sin_slip * 1.5, 2);
    q_cov_slip_rate = std::min(q_cov_slip_rate, ekf_params_.q_cov_slip_rate_max);
    q_cov_slip_rate = std::max(q_cov_slip_rate, ekf_params_.q_cov_slip_rate_min);
  }
  const float q_cov_x = std::pow(0.5 * ekf_params_.q_stddev_acc_long * dt * dt, 2);
  const float q_cov_y = std::pow(0.5 * ekf_params_.q_stddev_acc_lat * dt * dt, 2);
  const float q_cov_yaw = std::pow(q_stddev_yaw_rate * dt, 2);
  const float q_cov_vel = std::pow(ekf_params_.q_stddev_acc_long * dt, 2);
  const float q_cov_slip = q_cov_slip_rate * dt * dt;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
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
  // Eigen::MatrixXd B = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  // Eigen::MatrixXd u = Eigen::MatrixXd::Zero(ekf_params_.dim_x, 1);

  if (!ekf.predict(X_next_t, A, Q)) {
    RCLCPP_WARN(logger_, "Cannot predict");
  }

  return true;
}

bool NormalVehicleTracker::measureWithPose(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

  float r_cov_x;
  float r_cov_y;
  const uint8_t label = object_recognition_utils::getHighestProbLabel(object.classification);

  if (label == Label::CAR) {
    r_cov_x = ekf_params_.r_cov_x;
    r_cov_y = ekf_params_.r_cov_y;
  } else if (utils::isLargeVehicleLabel(label)) {
    constexpr float r_stddev_x = 2.0;  // [m]
    constexpr float r_stddev_y = 2.0;  // [m]
    r_cov_x = std::pow(r_stddev_x, 2.0);
    r_cov_y = std::pow(r_stddev_y, 2.0);
  } else {
    r_cov_x = ekf_params_.r_cov_x;
    r_cov_y = ekf_params_.r_cov_y;
  }

  // Decide dimension of measurement vector
  bool enable_velocity_measurement = false;
  if (object.kinematics.has_twist) {
    Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);  // predicted state
    ekf_.getX(X_t);
    const double predicted_vel = X_t(IDX::VEL);
    const double observed_vel = object.kinematics.twist_with_covariance.twist.linear.x;

    if (std::fabs(predicted_vel - observed_vel) < velocity_deviation_threshold_) {
      // Velocity deviation is small
      enable_velocity_measurement = true;
    }
  }

  // pos x, pos y, yaw, vel depending on pose measurement
  const int dim_y = enable_velocity_measurement ? 4 : 3;
  double measurement_yaw = getMeasurementYaw(object);  // get sign-solved yaw angle
  Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);           // predicted state
  ekf_.getX(X_t);

  // convert to boundingbox if input is convex shape
  autoware_auto_perception_msgs::msg::DetectedObject bbox_object;
  if (object.shape.type != autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    utils::convertConvexHullToBoundingBox(object, bbox_object);
  } else {
    bbox_object = object;
  }

  /* get offset measurement*/
  autoware_auto_perception_msgs::msg::DetectedObject offset_object;
  utils::calcAnchorPointOffset(
    last_input_bounding_box_.width, last_input_bounding_box_.length, last_nearest_corner_index_,
    bbox_object, X_t(IDX::YAW), offset_object, tracking_offset_);

  // Set measurement matrix C and observation vector Y
  Eigen::MatrixXd Y(dim_y, 1);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, ekf_params_.dim_x);
  Y(IDX::X, 0) = offset_object.kinematics.pose_with_covariance.pose.position.x;
  Y(IDX::Y, 0) = offset_object.kinematics.pose_with_covariance.pose.position.y;
  Y(IDX::YAW, 0) = measurement_yaw;
  C(0, IDX::X) = 1.0;    // for pos x
  C(1, IDX::Y) = 1.0;    // for pos y
  C(2, IDX::YAW) = 1.0;  // for yaw

  // Set noise covariance matrix R
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  if (!object.kinematics.has_position_covariance) {
    const double cos_yaw = std::cos(measurement_yaw);
    const double sin_yaw = std::sin(measurement_yaw);
    const double sin_2yaw = std::sin(2.0f * measurement_yaw);
    R(0, 0) = r_cov_x * cos_yaw * cos_yaw + r_cov_y * sin_yaw * sin_yaw;  // x - x
    R(0, 1) = 0.5f * (r_cov_x - r_cov_y) * sin_2yaw;                      // x - y
    R(1, 1) = r_cov_x * sin_yaw * sin_yaw + r_cov_y * cos_yaw * cos_yaw;  // y - y
    R(1, 0) = R(0, 1);                                                    // y - x
    R(2, 2) = ekf_params_.r_cov_yaw;                                      // yaw - yaw
  } else {
    R(0, 0) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_X];
    R(0, 1) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_Y];
    R(0, 2) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_YAW];
    R(1, 0) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_X];
    R(1, 1) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
    R(1, 2) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_YAW];
    R(2, 0) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::YAW_X];
    R(2, 1) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::YAW_Y];
    R(2, 2) = object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW];
  }

  // Update the velocity when necessary
  if (dim_y == 4) {
    Y(IDX::VEL, 0) = object.kinematics.twist_with_covariance.twist.linear.x;
    C(3, IDX::VEL) = 1.0;  // for vel

    if (!object.kinematics.has_twist_covariance) {
      R(3, 3) = ekf_params_.r_cov_vel;  // vel -vel
    } else {
      R(3, 3) = object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::X_X];
    }
  }

  // ekf update
  if (!ekf_.update(Y, C, R)) {
    RCLCPP_WARN(logger_, "Cannot update");
  }

  // normalize yaw and limit vel, slip
  {
    Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);
    Eigen::MatrixXd P_t(ekf_params_.dim_x, ekf_params_.dim_x);
    ekf_.getX(X_t);
    ekf_.getP(P_t);
    X_t(IDX::YAW) = tier4_autoware_utils::normalizeRadian(X_t(IDX::YAW));
    if (!(-max_vel_ <= X_t(IDX::VEL) && X_t(IDX::VEL) <= max_vel_)) {
      X_t(IDX::VEL) = X_t(IDX::VEL) < 0 ? -max_vel_ : max_vel_;
    }
    if (!(-max_slip_ <= X_t(IDX::SLIP) && X_t(IDX::SLIP) <= max_slip_)) {
      X_t(IDX::SLIP) = X_t(IDX::SLIP) < 0 ? -max_slip_ : max_slip_;
    }
    ekf_.init(X_t, P_t);
  }

  // position z
  constexpr float gain = 0.9;
  z_ = gain * z_ + (1.0 - gain) * object.kinematics.pose_with_covariance.pose.position.z;

  return true;
}

bool NormalVehicleTracker::measureWithShape(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  // if the input shape is convex type, convert it to bbox type
  autoware_auto_perception_msgs::msg::DetectedObject bbox_object;
  if (object.shape.type != autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    utils::convertConvexHullToBoundingBox(object, bbox_object);
  } else {
    bbox_object = object;
  }

  constexpr float gain = 0.9;
  bounding_box_.length =
    gain * bounding_box_.length + (1.0 - gain) * bbox_object.shape.dimensions.x;
  bounding_box_.width = gain * bounding_box_.width + (1.0 - gain) * bbox_object.shape.dimensions.y;
  bounding_box_.height =
    gain * bounding_box_.height + (1.0 - gain) * bbox_object.shape.dimensions.z;
  last_input_bounding_box_ = {
    bbox_object.shape.dimensions.x, bbox_object.shape.dimensions.y, bbox_object.shape.dimensions.z};

  // update lf, lr
  lf_ = std::max(bounding_box_.length * 0.3, 1.0);   // 30% front from the center, minimum of 1.0m
  lr_ = std::max(bounding_box_.length * 0.25, 1.0);  // 25% rear from the center, minimum of 1.0m

  return true;
}

bool NormalVehicleTracker::measure(
  const autoware_auto_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & self_transform)
{
  const auto & current_classification = getClassification();
  object_ = object;
  if (object_recognition_utils::getHighestProbLabel(object.classification) == Label::UNKNOWN) {
    setClassification(current_classification);
  }

  if (0.01 /*10msec*/ < std::fabs((time - last_update_time_).seconds())) {
    RCLCPP_WARN(
      logger_, "There is a large gap between predicted time and measurement time. (%f)",
      (time - last_update_time_).seconds());
  }

  measureWithPose(object);
  measureWithShape(object);

  /* calc nearest corner index*/
  setNearestCornerOrSurfaceIndex(self_transform);  // this index is used in next measure step

  return true;
}

bool NormalVehicleTracker::getTrackedObject(
  const rclcpp::Time & time, autoware_auto_perception_msgs::msg::TrackedObject & object) const
{
  object = object_recognition_utils::toTrackedObject(object_);
  object.object_id = getUUID();
  object.classification = getClassification();

  // predict state
  KalmanFilter tmp_ekf_for_no_update = ekf_;
  const double dt = (time - last_update_time_).seconds();
  if (0.001 /*1msec*/ < dt) {
    predict(dt, tmp_ekf_for_no_update);
  }
  Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);                // predicted state
  Eigen::MatrixXd P(ekf_params_.dim_x, ekf_params_.dim_x);  // predicted state
  tmp_ekf_for_no_update.getX(X_t);
  tmp_ekf_for_no_update.getP(P);

  /*  put predicted pose and twist to output object  */
  auto & pose_with_cov = object.kinematics.pose_with_covariance;
  auto & twist_with_cov = object.kinematics.twist_with_covariance;

  // recover bounding box from tracking point
  const double dl = bounding_box_.length - last_input_bounding_box_.length;
  const double dw = bounding_box_.width - last_input_bounding_box_.width;
  const Eigen::Vector2d recovered_pose = utils::recoverFromTrackingPoint(
    X_t(IDX::X), X_t(IDX::Y), X_t(IDX::YAW), dw, dl, last_nearest_corner_index_, tracking_offset_);
  X_t(IDX::X) = recovered_pose.x();
  X_t(IDX::Y) = recovered_pose.y();

  // position
  pose_with_cov.pose.position.x = X_t(IDX::X);
  pose_with_cov.pose.position.y = X_t(IDX::Y);
  pose_with_cov.pose.position.z = z_;
  // quaternion
  {
    double roll, pitch, yaw;
    tf2::Quaternion original_quaternion;
    tf2::fromMsg(object_.kinematics.pose_with_covariance.pose.orientation, original_quaternion);
    tf2::Matrix3x3(original_quaternion).getRPY(roll, pitch, yaw);
    tf2::Quaternion filtered_quaternion;
    filtered_quaternion.setRPY(roll, pitch, X_t(IDX::YAW));
    pose_with_cov.pose.orientation.x = filtered_quaternion.x();
    pose_with_cov.pose.orientation.y = filtered_quaternion.y();
    pose_with_cov.pose.orientation.z = filtered_quaternion.z();
    pose_with_cov.pose.orientation.w = filtered_quaternion.w();
    object.kinematics.orientation_availability =
      autoware_auto_perception_msgs::msg::TrackedObjectKinematics::SIGN_UNKNOWN;
  }
  // position covariance
  constexpr double z_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double r_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double p_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  pose_with_cov.covariance[utils::MSG_COV_IDX::X_X] = P(IDX::X, IDX::X);
  pose_with_cov.covariance[utils::MSG_COV_IDX::X_Y] = P(IDX::X, IDX::Y);
  pose_with_cov.covariance[utils::MSG_COV_IDX::Y_X] = P(IDX::Y, IDX::X);
  pose_with_cov.covariance[utils::MSG_COV_IDX::Y_Y] = P(IDX::Y, IDX::Y);
  pose_with_cov.covariance[utils::MSG_COV_IDX::Z_Z] = z_cov;
  pose_with_cov.covariance[utils::MSG_COV_IDX::ROLL_ROLL] = r_cov;
  pose_with_cov.covariance[utils::MSG_COV_IDX::PITCH_PITCH] = p_cov;
  pose_with_cov.covariance[utils::MSG_COV_IDX::YAW_YAW] = P(IDX::YAW, IDX::YAW);

  // twist
  twist_with_cov.twist.linear.x = X_t(IDX::VEL) * std::cos(X_t(IDX::SLIP));
  twist_with_cov.twist.linear.y = X_t(IDX::VEL) * std::sin(X_t(IDX::SLIP));
  twist_with_cov.twist.angular.z =
    X_t(IDX::VEL) * std::sin(X_t(IDX::SLIP)) / lr_;  // yaw_rate = vel_k * sin(slip_k) / l_r
  /* twist covariance
   * convert covariance from velocity, slip angle to vx, vy, and yaw angle
   *
   * vx = vel * cos(slip)
   * vy = vel * sin(slip)
   * wz = vel * sin(slip) / l_r = vy / l_r
   *
   */

  constexpr double vz_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double wx_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative
  constexpr double wy_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative

  Eigen::MatrixXd cov_jacob(3, 2);
  cov_jacob << std::cos(X_t(IDX::SLIP)), -X_t(IDX::VEL) * std::sin(X_t(IDX::SLIP)),
    std::sin(X_t(IDX::SLIP)), X_t(IDX::VEL) * std::cos(X_t(IDX::SLIP)),
    std::sin(X_t(IDX::SLIP)) / lr_, X_t(IDX::VEL) * std::cos(X_t(IDX::SLIP)) / lr_;
  Eigen::MatrixXd cov_twist(2, 2);
  cov_twist << P(IDX::VEL, IDX::VEL), P(IDX::VEL, IDX::SLIP), P(IDX::SLIP, IDX::VEL),
    P(IDX::SLIP, IDX::SLIP);
  Eigen::MatrixXd twist_cov_mat = cov_jacob * cov_twist * cov_jacob.transpose();

  twist_with_cov.covariance[utils::MSG_COV_IDX::X_X] = twist_cov_mat(0, 0);
  twist_with_cov.covariance[utils::MSG_COV_IDX::X_Y] = twist_cov_mat(0, 1);
  twist_with_cov.covariance[utils::MSG_COV_IDX::X_YAW] = twist_cov_mat(0, 2);
  twist_with_cov.covariance[utils::MSG_COV_IDX::Y_X] = twist_cov_mat(1, 0);
  twist_with_cov.covariance[utils::MSG_COV_IDX::Y_Y] = twist_cov_mat(1, 1);
  twist_with_cov.covariance[utils::MSG_COV_IDX::Y_YAW] = twist_cov_mat(1, 2);
  twist_with_cov.covariance[utils::MSG_COV_IDX::YAW_X] = twist_cov_mat(2, 0);
  twist_with_cov.covariance[utils::MSG_COV_IDX::YAW_Y] = twist_cov_mat(2, 1);
  twist_with_cov.covariance[utils::MSG_COV_IDX::YAW_YAW] = twist_cov_mat(2, 2);
  twist_with_cov.covariance[utils::MSG_COV_IDX::Z_Z] = vz_cov;
  twist_with_cov.covariance[utils::MSG_COV_IDX::ROLL_ROLL] = wx_cov;
  twist_with_cov.covariance[utils::MSG_COV_IDX::PITCH_PITCH] = wy_cov;

  // set shape
  object.shape.dimensions.x = bounding_box_.length;
  object.shape.dimensions.y = bounding_box_.width;
  object.shape.dimensions.z = bounding_box_.height;
  const auto origin_yaw = tf2::getYaw(object_.kinematics.pose_with_covariance.pose.orientation);
  const auto ekf_pose_yaw = tf2::getYaw(pose_with_cov.pose.orientation);
  object.shape.footprint =
    tier4_autoware_utils::rotatePolygon(object.shape.footprint, origin_yaw - ekf_pose_yaw);

  return true;
}

void NormalVehicleTracker::setNearestCornerOrSurfaceIndex(
  const geometry_msgs::msg::Transform & self_transform)
{
  Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);
  ekf_.getX(X_t);
  last_nearest_corner_index_ = utils::getNearestCornerOrSurface(
    X_t(IDX::X), X_t(IDX::Y), X_t(IDX::YAW), bounding_box_.width, bounding_box_.length,
    self_transform);
}

double NormalVehicleTracker::getMeasurementYaw(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  double measurement_yaw = tier4_autoware_utils::normalizeRadian(
    tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation));
  {
    Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);
    ekf_.getX(X_t);
    // Fixed measurement_yaw to be in the range of +-90 degrees of X_t(IDX::YAW)
    while (M_PI_2 <= X_t(IDX::YAW) - measurement_yaw) {
      measurement_yaw = measurement_yaw + M_PI;
    }
    while (M_PI_2 <= measurement_yaw - X_t(IDX::YAW)) {
      measurement_yaw = measurement_yaw - M_PI;
    }
  }
  return measurement_yaw;
}
