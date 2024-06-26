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

#include "radar_object_tracker/tracker/model/linear_motion_tracker.hpp"

#include "radar_object_tracker/utils/utils.hpp"

#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>

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
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

using Label = autoware_perception_msgs::msg::ObjectClassification;

// initialize static parameter
bool LinearMotionTracker::is_initialized_ = false;
LinearMotionTracker::EkfParams LinearMotionTracker::ekf_params_;
double LinearMotionTracker::max_vx_;
double LinearMotionTracker::max_vy_;
double LinearMotionTracker::filter_tau_;
double LinearMotionTracker::filter_dt_;
bool LinearMotionTracker::estimate_acc_;
bool LinearMotionTracker::trust_yaw_input_;
bool LinearMotionTracker::trust_twist_input_;
bool LinearMotionTracker::use_polar_coordinate_in_measurement_noise_;

LinearMotionTracker::LinearMotionTracker(
  const rclcpp::Time & time, const autoware_perception_msgs::msg::DetectedObject & object,
  const std::string & tracker_param_file, const std::uint8_t & /*label*/)
: Tracker(time, object.classification),
  logger_(rclcpp::get_logger("LinearMotionTracker")),
  last_update_time_(time),
  z_(object.kinematics.pose_with_covariance.pose.position.z)
{
  object_ = object;

  // load setting from yaml file
  if (!is_initialized_) {
    loadDefaultModelParameters(tracker_param_file);
    is_initialized_ = true;
  }
  // shape initialization
  bounding_box_ = {0.5, 0.5, 1.7};
  cylinder_ = {0.3, 1.7};

  // initialize X matrix and position
  Eigen::MatrixXd X(ekf_params_.dim_x, 1);
  X(IDX::X) = object.kinematics.pose_with_covariance.pose.position.x;
  X(IDX::Y) = object.kinematics.pose_with_covariance.pose.position.y;
  const auto yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  z_ = object.kinematics.pose_with_covariance.pose.position.z;
  yaw_ = yaw;
  // radar object usually have twist
  if (object.kinematics.has_twist) {
    const auto v = object.kinematics.twist_with_covariance.twist.linear.x;
    X(IDX::VX) = v * std::cos(yaw);
    X(IDX::VY) = v * std::sin(yaw);
  } else {
    X(IDX::VX) = 0.0;
    X(IDX::VY) = 0.0;
  }
  // init ax and ay
  X(IDX::AX) = 0.0;
  X(IDX::AY) = 0.0;

  // initialize P matrix
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);

  // create rotation matrix to rotate covariance matrix
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  // 2d rotation matrix
  Eigen::Matrix2d R;
  R << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;

  // covariance matrix in the target vehicle coordinate system
  Eigen::Matrix2d P_xy_local, P_v_xy_local, P_a_xy_local;
  P_xy_local << ekf_params_.p0_cov_x, 0.0, 0.0, ekf_params_.p0_cov_y;
  P_v_xy_local << ekf_params_.p0_cov_vx, 0.0, 0.0, ekf_params_.p0_cov_vy;
  P_a_xy_local << ekf_params_.p0_cov_ax, 0.0, 0.0, ekf_params_.p0_cov_ay;

  // Rotated covariance matrix
  // covariance is rotated by 2D rotation matrix R
  // P′=R P RT
  Eigen::Matrix2d P_xy, P_v_xy, P_a_xy;

  // Rotate the covariance matrix according to the vehicle yaw
  // because p0_cov_x and y are in the vehicle coordinate system.
  if (object.kinematics.has_position_covariance) {
    P_xy_local << object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_X],
      object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_Y],
      object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_X],
      object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
    P_xy = R * P_xy_local * R.transpose();
  } else {
    // rotate
    P_xy = R * P_xy_local * R.transpose();
  }
  // covariance often written in object frame
  if (object.kinematics.has_twist_covariance) {
    const auto vx_cov = object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::X_X];
    const auto vy_cov = object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
    P_v_xy_local << vx_cov, 0.0, 0.0, vy_cov;
    P_v_xy = R * P_v_xy_local * R.transpose();
  } else {
    P_v_xy = R * P_v_xy_local * R.transpose();
  }

  P_a_xy = R * P_a_xy_local * R.transpose();

  // put value in P matrix
  // use block description. This assume x,y,vx,vy,ax,ay in this order
  P.block<2, 2>(IDX::X, IDX::X) = P_xy;
  P.block<2, 2>(IDX::VX, IDX::VX) = P_v_xy;
  P.block<2, 2>(IDX::AX, IDX::AX) = P_a_xy;

  // init shape
  if (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    bounding_box_ = {
      object.shape.dimensions.x, object.shape.dimensions.y, object.shape.dimensions.z};
  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    cylinder_ = {object.shape.dimensions.x, object.shape.dimensions.z};
  }

  ekf_.init(X, P);
}

void LinearMotionTracker::loadDefaultModelParameters(const std::string & path)
{
  YAML::Node config = YAML::LoadFile(path);
  // initialize ekf params
  const float q_stddev_ax =
    config["default"]["ekf_params"]["process_noise_std"]["ax"].as<float>();  // [m/(s*s)]
  const float q_stddev_ay =
    config["default"]["ekf_params"]["process_noise_std"]["ay"].as<float>();  // [m/(s*s)]
  const float q_stddev_vx =
    config["default"]["ekf_params"]["process_noise_std"]["vx"].as<float>();  // [m/s]
  const float q_stddev_vy =
    config["default"]["ekf_params"]["process_noise_std"]["vy"].as<float>();  // [m/s]
  const float q_stddev_x =
    config["default"]["ekf_params"]["process_noise_std"]["x"].as<float>();  // [m]
  const float q_stddev_y =
    config["default"]["ekf_params"]["process_noise_std"]["y"].as<float>();  // [m]
  const float r_stddev_x =
    config["default"]["ekf_params"]["measurement_noise_std"]["x"].as<float>();  // [m]
  const float r_stddev_y =
    config["default"]["ekf_params"]["measurement_noise_std"]["y"].as<float>();  // [m]
  const float r_stddev_vx =
    config["default"]["ekf_params"]["measurement_noise_std"]["vx"].as<float>();  // [m]
  const float r_stddev_vy =
    config["default"]["ekf_params"]["measurement_noise_std"]["vy"].as<float>();  // [m]
  const float p0_stddev_x =
    config["default"]["ekf_params"]["initial_covariance_std"]["x"].as<float>();  // [m/s]
  const float p0_stddev_y =
    config["default"]["ekf_params"]["initial_covariance_std"]["y"].as<float>();  // [m/s]
  const float p0_stddev_vx =
    config["default"]["ekf_params"]["initial_covariance_std"]["vx"].as<float>();  // [m/(s)]
  const float p0_stddev_vy =
    config["default"]["ekf_params"]["initial_covariance_std"]["vy"].as<float>();  // [m/(s)]
  const float p0_stddev_ax =
    config["default"]["ekf_params"]["initial_covariance_std"]["ax"].as<float>();  // [m/(s*s)]
  const float p0_stddev_ay =
    config["default"]["ekf_params"]["initial_covariance_std"]["ay"].as<float>();  // [m/(s*s)]
  estimate_acc_ = config["default"]["ekf_params"]["estimate_acc"].as<bool>();
  trust_yaw_input_ = config["default"]["trust_yaw_input"].as<bool>(false);      // default false
  trust_twist_input_ = config["default"]["trust_twist_input"].as<bool>(false);  // default false
  use_polar_coordinate_in_measurement_noise_ =
    config["default"]["use_polar_coordinate_in_measurement_noise"].as<bool>(
      false);  // default false
  ekf_params_.q_cov_ax = std::pow(q_stddev_ax, 2.0);
  ekf_params_.q_cov_ay = std::pow(q_stddev_ay, 2.0);
  ekf_params_.q_cov_vx = std::pow(q_stddev_vx, 2.0);
  ekf_params_.q_cov_vy = std::pow(q_stddev_vy, 2.0);
  ekf_params_.q_cov_x = std::pow(q_stddev_x, 2.0);
  ekf_params_.q_cov_y = std::pow(q_stddev_y, 2.0);
  ekf_params_.r_cov_x = std::pow(r_stddev_x, 2.0);
  ekf_params_.r_cov_y = std::pow(r_stddev_y, 2.0);
  ekf_params_.r_cov_vx = std::pow(r_stddev_vx, 2.0);
  ekf_params_.r_cov_vy = std::pow(r_stddev_vy, 2.0);
  ekf_params_.p0_cov_x = std::pow(p0_stddev_x, 2.0);
  ekf_params_.p0_cov_y = std::pow(p0_stddev_y, 2.0);
  ekf_params_.p0_cov_vx = std::pow(p0_stddev_vx, 2.0);
  ekf_params_.p0_cov_vy = std::pow(p0_stddev_vy, 2.0);
  ekf_params_.p0_cov_ay = std::pow(p0_stddev_ax, 2.0);
  ekf_params_.p0_cov_ay = std::pow(p0_stddev_ay, 2.0);

  // lpf filter parameters
  filter_tau_ = config["default"]["low_pass_filter"]["time_constant"].as<float>(1.0);  // [s]
  filter_dt_ = config["default"]["low_pass_filter"]["sampling_time"].as<float>(0.1);   // [s]

  // limitation
  // (TODO): this may be written in another yaml file based on classify result
  const float max_speed_kmph = config["default"]["limit"]["max_speed"].as<float>();  // [km/h]
  max_vx_ = autoware::universe_utils::kmph2mps(max_speed_kmph);                      // [m/s]
  max_vy_ = autoware::universe_utils::kmph2mps(max_speed_kmph);                      // [rad/s]
}

bool LinearMotionTracker::predict(const rclcpp::Time & time)
{
  const double dt = (time - last_update_time_).seconds();
  bool ret = predict(dt, ekf_);
  if (ret) {
    last_update_time_ = time;
  }
  return ret;
}

bool LinearMotionTracker::predict(const double dt, KalmanFilter & ekf) const
{
  /*  == Linear model ==
   *
   *  x_{k+1} = x_k + dt * vx_k + 0.5 * dt^2 * ax_k
   *  y_{k+1} = y_k + dt * vy_k + 0.5 * dt^2 * ay_k
   *  vx_{k+1} = vx_k + dt * ax_k
   *  vy_{k+1} = vy_k + dt * ay_k
   *  ax_{k+1} = ax_k
   *  ay_{k+1} = ay_k
   */

  /*  == state transition matrix ==
   *
   * A = [1, 0, dt, 0,  0.5 * dt^2, 0,
   *      0, 1, 0,  dt, 0, 0.5 * dt^2,
   *      0, 0, 1,  0,  dt,         0,
   *      0, 0, 0,  1,  0,          dt,
   *      0, 0, 0,  0,  1,          0,
   *      0, 0, 0,  0,  0,          1]
   */
  // estimate acc
  const double acc_coeff = estimate_acc_ ? 1.0 : 0.0;

  // X t
  Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);  // predicted state
  ekf.getX(X_t);
  const auto x = X_t(IDX::X);
  const auto y = X_t(IDX::Y);
  const auto vx = X_t(IDX::VX);
  const auto vy = X_t(IDX::VY);
  const auto ax = X_t(IDX::AX);
  const auto ay = X_t(IDX::AY);

  // X t+1
  Eigen::MatrixXd X_next_t(ekf_params_.dim_x, 1);
  X_next_t(IDX::X) = x + vx * dt + 0.5 * ax * dt * dt * acc_coeff;
  X_next_t(IDX::Y) = y + vy * dt + 0.5 * ay * dt * dt * acc_coeff;
  X_next_t(IDX::VX) = vx + ax * dt * acc_coeff;
  X_next_t(IDX::VY) = vy + ay * dt * acc_coeff;
  X_next_t(IDX::AX) = ax;
  X_next_t(IDX::AY) = ay;

  // A: state transition matrix
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(ekf_params_.dim_x, ekf_params_.dim_x);
  A(IDX::X, IDX::VX) = dt;
  A(IDX::Y, IDX::VY) = dt;
  A(IDX::X, IDX::AX) = 0.5 * dt * dt * acc_coeff;
  A(IDX::Y, IDX::AY) = 0.5 * dt * dt * acc_coeff;
  A(IDX::VX, IDX::AX) = dt * acc_coeff;
  A(IDX::VY, IDX::AY) = dt * acc_coeff;

  // Q: system noise
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  Eigen::MatrixXd Q_local = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);

  // system noise in local coordinate
  // we assume acceleration random walk model
  //
  // Q_local = [dt^3/6 0 dt^2/2 0 dt 0] ^ T q_cov_ax [dt^3/6 0 dt^2/2 0 dt 0]
  //           + [0 dt^3/6 0 dt^2/2 0 dt] ^ T q_cov_ay [0 dt^3/6 0 dt^2/2 0 dt]
  // Eigen::MatrixXd qx = Eigen::MatrixXd::Zero(ekf_params_.dim_x, 1);
  // Eigen::MatrixXd qy = Eigen::MatrixXd::Zero(ekf_params_.dim_x, 1);
  // qx << dt * dt * dt / 6, 0, dt * dt / 2, 0, dt, 0;
  // qy << 0, dt * dt * dt / 6, 0, dt * dt / 2, 0, dt;
  // Q_local = qx * ekf_params_.q_cov_ax * qx.transpose() + qy * ekf_params_.q_cov_ay *
  // qy.transpose(); just create diag matrix
  Eigen::VectorXd q_diag_vector = Eigen::VectorXd::Zero(ekf_params_.dim_x);
  q_diag_vector << ekf_params_.q_cov_x, ekf_params_.q_cov_y, ekf_params_.q_cov_vx,
    ekf_params_.q_cov_vy, ekf_params_.q_cov_ax, ekf_params_.q_cov_ay;
  Q_local = q_diag_vector.asDiagonal();

  // Rotate the covariance matrix according to the vehicle yaw
  // because q_cov_x and y are in the vehicle coordinate system.
  // rotate pose, velocity, acceleration covariance in one matrix
  // it is something like
  // [R 0 0]             [R^T 0 0]
  // [0 R 0] * Q_local * [0 R^T 0]
  // [0 0 R]             [0 0 R^T]
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
  R << cos(yaw_), -sin(yaw_), sin(yaw_), cos(yaw_);
  Eigen::MatrixXd RotateCovMatrix = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  RotateCovMatrix.block<2, 2>(IDX::X, IDX::X) = R;
  RotateCovMatrix.block<2, 2>(IDX::VX, IDX::VX) = R;
  RotateCovMatrix.block<2, 2>(IDX::AX, IDX::AX) = R;
  Q = RotateCovMatrix * Q_local * RotateCovMatrix.transpose();

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  Eigen::MatrixXd u = Eigen::MatrixXd::Zero(ekf_params_.dim_x, 1);

  // call kalman filter library
  if (!ekf.predict(X_next_t, A, Q)) {
    RCLCPP_WARN(logger_, "Cannot predict");
  }

  return true;
}

bool LinearMotionTracker::measureWithPose(
  const autoware_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & self_transform)
{
  // Observation pattern will be:
  // 1. x, y, vx, vy
  // 2. x, y
  // 3. vx, vy (Do not consider this right now)
  //
  // We handle this measurements by stacking observation matrix, measurement vector and measurement
  // covariance
  // - observation matrix: C
  // - measurement vector : Y
  // - measurement covariance: R

  // rotation matrix
  Eigen::Matrix2d RotationYaw;
  if (trust_yaw_input_) {
    const auto yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    RotationYaw << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  } else {
    RotationYaw << std::cos(yaw_), -std::sin(yaw_), std::sin(yaw_), std::cos(yaw_);
  }
  const auto base_link_yaw = tf2::getYaw(self_transform.rotation);
  Eigen::Matrix2d RotationBaseLink;
  RotationBaseLink << std::cos(base_link_yaw), -std::sin(base_link_yaw), std::sin(base_link_yaw),
    std::cos(base_link_yaw);

  // depth from base_link to object
  const auto dx =
    object.kinematics.pose_with_covariance.pose.position.x - self_transform.translation.x;
  const auto dy =
    object.kinematics.pose_with_covariance.pose.position.y - self_transform.translation.y;
  Eigen::Vector2d pose_diff_in_map(dx, dy);
  Eigen::Vector2d pose_diff_in_base_link = RotationBaseLink * pose_diff_in_map;
  const auto depth = abs(pose_diff_in_base_link(0));

  // gather matrices as vector
  std::vector<Eigen::MatrixXd> C_list;
  std::vector<Eigen::MatrixXd> Y_list;
  std::vector<Eigen::MatrixXd> R_block_list;

  // 1. add position measurement
  const bool enable_position_measurement = true;  // assume position is always measured
  if (enable_position_measurement) {
    Eigen::MatrixXd Cxy = Eigen::MatrixXd::Zero(2, ekf_params_.dim_x);
    Cxy(0, IDX::X) = 1;
    Cxy(1, IDX::Y) = 1;
    C_list.push_back(Cxy);

    Eigen::MatrixXd Yxy = Eigen::MatrixXd::Zero(2, 1);
    Yxy << object.kinematics.pose_with_covariance.pose.position.x,
      object.kinematics.pose_with_covariance.pose.position.y;
    Y_list.push_back(Yxy);

    // covariance need to be rotated since it is in the vehicle coordinate system
    Eigen::MatrixXd Rxy_local = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd Rxy = Eigen::MatrixXd::Zero(2, 2);
    if (!object.kinematics.has_position_covariance) {
      // switch noise covariance in polar coordinate or cartesian coordinate
      const auto r_cov_y = use_polar_coordinate_in_measurement_noise_
                             ? depth * depth * ekf_params_.r_cov_x
                             : ekf_params_.r_cov_y;
      Rxy_local << ekf_params_.r_cov_x, 0, 0, r_cov_y;  // xy in base_link coordinate
      Rxy = RotationBaseLink * Rxy_local * RotationBaseLink.transpose();
    } else {
      Rxy_local << object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_X],
        object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_Y],
        object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_X],
        object.kinematics.pose_with_covariance
          .covariance[utils::MSG_COV_IDX::Y_Y];  // xy in vehicle coordinate
      Rxy = RotationYaw * Rxy_local * RotationYaw.transpose();
    }
    R_block_list.push_back(Rxy);
  }

  // 2. add linear velocity measurement
  const bool enable_velocity_measurement = object.kinematics.has_twist && trust_twist_input_;
  if (enable_velocity_measurement) {
    Eigen::MatrixXd C_vx_vy = Eigen::MatrixXd::Zero(2, ekf_params_.dim_x);
    C_vx_vy(0, IDX::VX) = 1;
    C_vx_vy(1, IDX::VY) = 1;
    C_list.push_back(C_vx_vy);

    // velocity is in the target vehicle coordinate system
    Eigen::MatrixXd Vxy_local = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd Vxy = Eigen::MatrixXd::Zero(2, 1);
    Vxy_local << object.kinematics.twist_with_covariance.twist.linear.x,
      object.kinematics.twist_with_covariance.twist.linear.y;
    Vxy = RotationYaw * Vxy_local;
    Y_list.push_back(Vxy);

    Eigen::Matrix2d R_v_xy_local = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd R_v_xy = Eigen::MatrixXd::Zero(2, 2);
    if (!object.kinematics.has_twist_covariance) {
      R_v_xy_local << ekf_params_.r_cov_vx, 0, 0, ekf_params_.r_cov_vy;
      R_v_xy = RotationBaseLink * R_v_xy_local * RotationBaseLink.transpose();
    } else {
      R_v_xy_local << object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::X_X],
        0, 0, object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
      R_v_xy = RotationYaw * R_v_xy_local * RotationYaw.transpose();
    }
    R_block_list.push_back(R_v_xy);
  }

  // 3. sum up matrices
  const auto row_number = std::accumulate(
    C_list.begin(), C_list.end(), 0,
    [](const auto & sum, const auto & C) { return sum + C.rows(); });
  if (row_number == 0) {
    RCLCPP_WARN(logger_, "No measurement is available");
    return false;
  }
  // Eigen::MatrixXd C = Eigen::MatrixXd::Zero(row_number, ekf_params_.dim_x);
  // Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(row_number, 1);
  // Eigen::MatrixXd R = Eigen::MatrixXd::Zero(row_number, row_number);

  // stacking matrices vertically or diagonally
  const auto C = utils::stackMatricesVertically(C_list);
  const auto Y = utils::stackMatricesVertically(Y_list);
  const auto R = utils::stackMatricesDiagonally(R_block_list);

  // 4. EKF update
  if (!ekf_.update(Y, C, R)) {
    RCLCPP_WARN(logger_, "Cannot update");
  }

  // 5. normalize: limit vx, vy
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

  // 6. Filter z and yaw
  // first order low pass filter
  const float gain = filter_tau_ / (filter_tau_ + filter_dt_);
  z_ = gain * z_ + (1.0 - gain) * object.kinematics.pose_with_covariance.pose.position.z;
  // get yaw from twist atan
  Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);
  ekf_.getX(X_t);
  const auto twist_yaw =
    std::atan2(X_t(IDX::VY), X_t(IDX::VX));  // calc from lateral and longitudinal velocity
  if (trust_yaw_input_) {
    yaw_ = gain * yaw_ + (1.0 - gain) * twist_yaw;
  } else {
    yaw_ = twist_yaw;
  }
  return true;
}

bool LinearMotionTracker::measureWithShape(
  const autoware_perception_msgs::msg::DetectedObject & object)
{
  // just use first order low pass filter
  const float gain = filter_tau_ / (filter_tau_ + filter_dt_);

  if (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    bounding_box_.length = gain * bounding_box_.length + (1.0 - gain) * object.shape.dimensions.x;
    bounding_box_.width = gain * bounding_box_.width + (1.0 - gain) * object.shape.dimensions.y;
    bounding_box_.height = gain * bounding_box_.height + (1.0 - gain) * object.shape.dimensions.z;
  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    cylinder_.width = gain * cylinder_.width + (1.0 - gain) * object.shape.dimensions.x;
    cylinder_.height = gain * cylinder_.height + (1.0 - gain) * object.shape.dimensions.z;
  } else {
    return false;
  }

  return true;
}

bool LinearMotionTracker::measure(
  const autoware_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
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

  measureWithPose(object, self_transform);
  measureWithShape(object);

  // (void)self_transform;  // currently do not use self vehicle position
  return true;
}

bool LinearMotionTracker::getTrackedObject(
  const rclcpp::Time & time, autoware_perception_msgs::msg::TrackedObject & object) const
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
  auto & acceleration_with_cov = object.kinematics.acceleration_with_covariance;
  // rotation matrix with yaw_
  Eigen::Matrix2d R_yaw = Eigen::Matrix2d::Zero();
  R_yaw << std::cos(yaw_), -std::sin(yaw_), std::sin(yaw_), std::cos(yaw_);
  const Eigen::Matrix2d R_yaw_inv = R_yaw.transpose();

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
    filtered_quaternion.setRPY(roll, pitch, yaw_);
    pose_with_cov.pose.orientation.x = filtered_quaternion.x();
    pose_with_cov.pose.orientation.y = filtered_quaternion.y();
    pose_with_cov.pose.orientation.z = filtered_quaternion.z();
    pose_with_cov.pose.orientation.w = filtered_quaternion.w();
    if (trust_yaw_input_) {
      object.kinematics.orientation_availability =
        autoware_perception_msgs::msg::TrackedObjectKinematics::SIGN_UNKNOWN;
    } else {
      object.kinematics.orientation_availability =
        autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE;
    }
  }
  // twist
  // twist need to converted to local coordinate
  const auto vx = X_t(IDX::VX);
  const auto vy = X_t(IDX::VY);
  // rotate this vector with -yaw
  Eigen::Vector2d v_local = R_yaw_inv * Eigen::Vector2d(vx, vy);
  twist_with_cov.twist.linear.x = v_local(0);
  twist_with_cov.twist.linear.y = v_local(1);

  // acceleration
  // acceleration need to converted to local coordinate
  const auto ax = X_t(IDX::AX);
  const auto ay = X_t(IDX::AY);
  // rotate this vector with -yaw
  Eigen::Vector2d a_local = R_yaw_inv * Eigen::Vector2d(ax, ay);
  acceleration_with_cov.accel.linear.x = a_local(0);
  acceleration_with_cov.accel.linear.y = a_local(1);

  // ===== covariance transformation =====
  // since covariance in EKF is in map coordinate and output should be in object coordinate,
  // we need to transform covariance
  Eigen::Matrix2d P_xy_map, P_v_xy_map, P_a_xy_map;
  P_xy_map << P(IDX::X, IDX::X), P(IDX::X, IDX::Y), P(IDX::Y, IDX::X), P(IDX::Y, IDX::Y);
  P_v_xy_map << P(IDX::VX, IDX::VX), P(IDX::VX, IDX::VY), P(IDX::VY, IDX::VX), P(IDX::VY, IDX::VY);
  P_a_xy_map << P(IDX::AX, IDX::AX), P(IDX::AX, IDX::AY), P(IDX::AY, IDX::AX), P(IDX::AY, IDX::AY);

  // rotate covariance with -yaw
  Eigen::Matrix2d P_xy = R_yaw_inv * P_xy_map * R_yaw_inv.transpose();
  Eigen::Matrix2d P_v_xy = R_yaw_inv * P_v_xy_map * R_yaw_inv.transpose();
  Eigen::Matrix2d P_a_xy = R_yaw_inv * P_a_xy_map * R_yaw_inv.transpose();

  // position covariance
  constexpr double no_info_cov = 1e9;    // no information
  constexpr double z_cov = 1. * 1.;      // TODO(yukkysaito) Currently tentative
  constexpr double yaw_cov = 0.1 * 0.1;  // TODO(yukkysaito) Currently tentative

  pose_with_cov.covariance[utils::MSG_COV_IDX::X_X] = P_xy(IDX::X, IDX::X);
  pose_with_cov.covariance[utils::MSG_COV_IDX::X_Y] = P_xy(IDX::X, IDX::Y);
  pose_with_cov.covariance[utils::MSG_COV_IDX::Y_X] = P_xy(IDX::Y, IDX::X);
  pose_with_cov.covariance[utils::MSG_COV_IDX::Y_Y] = P_xy(IDX::Y, IDX::Y);
  pose_with_cov.covariance[utils::MSG_COV_IDX::Z_Z] = z_cov;
  pose_with_cov.covariance[utils::MSG_COV_IDX::ROLL_ROLL] = no_info_cov;
  pose_with_cov.covariance[utils::MSG_COV_IDX::PITCH_PITCH] = no_info_cov;
  pose_with_cov.covariance[utils::MSG_COV_IDX::YAW_YAW] = yaw_cov;

  // twist covariance
  constexpr double vz_cov = 0.2 * 0.2;  // TODO(Yoshi Ri) Currently tentative
  constexpr double wz_cov = 0.2 * 0.2;  // TODO(yukkysaito) Currently tentative

  twist_with_cov.covariance[utils::MSG_COV_IDX::X_X] = P_v_xy(IDX::X, IDX::X);
  twist_with_cov.covariance[utils::MSG_COV_IDX::X_Y] = P_v_xy(IDX::X, IDX::Y);
  twist_with_cov.covariance[utils::MSG_COV_IDX::Y_X] = P_v_xy(IDX::Y, IDX::X);
  twist_with_cov.covariance[utils::MSG_COV_IDX::Y_Y] = P_v_xy(IDX::Y, IDX::Y);
  twist_with_cov.covariance[utils::MSG_COV_IDX::Z_Z] = vz_cov;
  twist_with_cov.covariance[utils::MSG_COV_IDX::ROLL_ROLL] = no_info_cov;
  twist_with_cov.covariance[utils::MSG_COV_IDX::PITCH_PITCH] = no_info_cov;
  twist_with_cov.covariance[utils::MSG_COV_IDX::YAW_YAW] = wz_cov;

  // acceleration covariance
  acceleration_with_cov.covariance[utils::MSG_COV_IDX::X_X] = P_a_xy(IDX::X, IDX::X);
  acceleration_with_cov.covariance[utils::MSG_COV_IDX::X_Y] = P_a_xy(IDX::X, IDX::Y);
  acceleration_with_cov.covariance[utils::MSG_COV_IDX::Y_X] = P_a_xy(IDX::Y, IDX::X);
  acceleration_with_cov.covariance[utils::MSG_COV_IDX::Y_Y] = P_a_xy(IDX::Y, IDX::Y);
  acceleration_with_cov.covariance[utils::MSG_COV_IDX::Z_Z] = no_info_cov;
  acceleration_with_cov.covariance[utils::MSG_COV_IDX::ROLL_ROLL] = no_info_cov;
  acceleration_with_cov.covariance[utils::MSG_COV_IDX::PITCH_PITCH] = no_info_cov;
  acceleration_with_cov.covariance[utils::MSG_COV_IDX::YAW_YAW] = no_info_cov;

  // set shape
  if (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    object.shape.dimensions.x = bounding_box_.length;
    object.shape.dimensions.y = bounding_box_.width;
    object.shape.dimensions.z = bounding_box_.height;
  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    object.shape.dimensions.x = cylinder_.width;
    object.shape.dimensions.y = cylinder_.width;
    object.shape.dimensions.z = cylinder_.height;
  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    const auto origin_yaw = tf2::getYaw(object_.kinematics.pose_with_covariance.pose.orientation);
    const auto ekf_pose_yaw = tf2::getYaw(pose_with_cov.pose.orientation);
    object.shape.footprint =
      autoware::universe_utils::rotatePolygon(object.shape.footprint, origin_yaw - ekf_pose_yaw);
  }

  return true;
}
