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

#include "radar_object_tracker/tracker/model/constant_turn_rate_motion_tracker.hpp"

#include "radar_object_tracker/utils/utils.hpp"

#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
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

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

// init static member variables
bool ConstantTurnRateMotionTracker::is_initialized_ = false;
ConstantTurnRateMotionTracker::EkfParams ConstantTurnRateMotionTracker::ekf_params_;
double ConstantTurnRateMotionTracker::max_vx_;
double ConstantTurnRateMotionTracker::filter_tau_;
double ConstantTurnRateMotionTracker::filter_dt_;
bool ConstantTurnRateMotionTracker::assume_zero_yaw_rate_;
bool ConstantTurnRateMotionTracker::trust_yaw_input_;
bool ConstantTurnRateMotionTracker::trust_twist_input_;
bool ConstantTurnRateMotionTracker::use_polar_coordinate_in_measurement_noise_;

ConstantTurnRateMotionTracker::ConstantTurnRateMotionTracker(
  const rclcpp::Time & time, const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const std::string & tracker_param_file, const std::uint8_t & /*label*/)
: Tracker(time, object.classification),
  logger_(rclcpp::get_logger("ConstantTurnRateMotionTracker")),
  last_update_time_(time),
  z_(object.kinematics.pose_with_covariance.pose.position.z)
{
  object_ = object;

  // load setting from yaml file
  if (!is_initialized_) {
    loadDefaultModelParameters(tracker_param_file);  // currently not using label
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
  X(IDX::YAW) = yaw;
  // radar object usually have twist
  if (object.kinematics.has_twist) {
    const auto v = object.kinematics.twist_with_covariance.twist.linear.x;
    X(IDX::VX) = v;
  } else {
    X(IDX::VX) = 0.0;
  }
  // init turn rate
  X(IDX::WZ) = 0.0;

  // initialize P matrix
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);

  // create rotation matrix to rotate covariance matrix
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  // 2d rotation matrix
  Eigen::Matrix2d R;
  R << cos_yaw, -sin_yaw, sin_yaw, cos_yaw;

  // covariance matrix in the target vehicle coordinate system
  Eigen::Matrix2d P_xy_local;
  P_xy_local << ekf_params_.p0_cov_x, 0.0, 0.0, ekf_params_.p0_cov_y;

  // Rotated covariance matrix
  // covariance is rotated by 2D rotation matrix R
  // P′=R P RT
  Eigen::Matrix2d P_xy, P_v_xy;

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
  // put value in P matrix
  P.block<2, 2>(IDX::X, IDX::X) = P_xy;

  // covariance often written in object frame
  if (object.kinematics.has_twist_covariance) {
    const auto vx_cov = object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::X_X];
    // const auto vy_cov =
    // object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::Y_Y];
    P(IDX::VX, IDX::VX) = vx_cov;
  } else {
    P(IDX::VX, IDX::VX) = ekf_params_.p0_cov_vx;
  }

  P(IDX::YAW, IDX::YAW) = ekf_params_.p0_cov_yaw;
  P(IDX::WZ, IDX::WZ) = ekf_params_.p0_cov_wz;

  // init shape
  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    bounding_box_ = {
      object.shape.dimensions.x, object.shape.dimensions.y, object.shape.dimensions.z};
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    cylinder_ = {object.shape.dimensions.x, object.shape.dimensions.z};
  }

  ekf_.init(X, P);
}

void ConstantTurnRateMotionTracker::loadDefaultModelParameters(const std::string & path)
{
  YAML::Node config = YAML::LoadFile(path);
  // initialize ekf params
  const float q_stddev_x =
    config["default"]["ekf_params"]["process_noise_std"]["x"].as<float>();  // [m]
  const float q_stddev_y =
    config["default"]["ekf_params"]["process_noise_std"]["y"].as<float>();  // [m]
  const float q_stddev_yaw =
    config["default"]["ekf_params"]["process_noise_std"]["yaw"].as<float>();  // [rad]
  const float q_stddev_vx =
    config["default"]["ekf_params"]["process_noise_std"]["vx"].as<float>();  // [m/s]
  const float q_stddev_wz =
    config["default"]["ekf_params"]["process_noise_std"]["wz"].as<float>();  // [m/s]
  const float r_stddev_x =
    config["default"]["ekf_params"]["measurement_noise_std"]["x"].as<float>();  // [m]
  const float r_stddev_y =
    config["default"]["ekf_params"]["measurement_noise_std"]["y"].as<float>();  // [m]
  const float r_stddev_yaw =
    config["default"]["ekf_params"]["measurement_noise_std"]["yaw"].as<float>();  // [rad]
  const float r_stddev_vx =
    config["default"]["ekf_params"]["measurement_noise_std"]["vx"].as<float>();  // [m/s]
  const float p0_stddev_x =
    config["default"]["ekf_params"]["initial_covariance_std"]["x"].as<float>();  // [m/s]
  const float p0_stddev_y =
    config["default"]["ekf_params"]["initial_covariance_std"]["y"].as<float>();  // [m/s]
  const float p0_stddev_yaw =
    config["default"]["ekf_params"]["initial_covariance_std"]["yaw"].as<float>();  // [rad]
  const float p0_stddev_vx =
    config["default"]["ekf_params"]["initial_covariance_std"]["vx"].as<float>();  // [m/(s)]
  const float p0_stddev_wz =
    config["default"]["ekf_params"]["initial_covariance_std"]["wz"].as<float>();  // [rad/s]
  assume_zero_yaw_rate_ = config["default"]["assume_zero_yaw_rate"].as<bool>();   // default false
  trust_yaw_input_ = config["default"]["trust_yaw_input"].as<bool>();             // default false
  trust_twist_input_ = config["default"]["trust_twist_input"].as<bool>();         // default false
  use_polar_coordinate_in_measurement_noise_ =
    config["default"]["use_polar_coordinate_in_measurement_noise"].as<bool>();  // default false
  ekf_params_.q_cov_x = std::pow(q_stddev_x, 2.0);
  ekf_params_.q_cov_y = std::pow(q_stddev_y, 2.0);
  ekf_params_.q_cov_yaw = std::pow(q_stddev_yaw, 2.0);
  ekf_params_.q_cov_vx = std::pow(q_stddev_vx, 2.0);
  ekf_params_.q_cov_wz = std::pow(q_stddev_wz, 2.0);
  ekf_params_.r_cov_x = std::pow(r_stddev_x, 2.0);
  ekf_params_.r_cov_y = std::pow(r_stddev_y, 2.0);
  ekf_params_.r_cov_yaw = std::pow(r_stddev_yaw, 2.0);
  ekf_params_.r_cov_vx = std::pow(r_stddev_vx, 2.0);
  ekf_params_.p0_cov_x = std::pow(p0_stddev_x, 2.0);
  ekf_params_.p0_cov_y = std::pow(p0_stddev_y, 2.0);
  ekf_params_.p0_cov_yaw = std::pow(p0_stddev_yaw, 2.0);
  ekf_params_.p0_cov_vx = std::pow(p0_stddev_vx, 2.0);
  ekf_params_.p0_cov_wz = std::pow(p0_stddev_wz, 2.0);

  // lpf filter parameters
  filter_tau_ = config["default"]["low_pass_filter"]["time_constant"].as<float>();  // [s]
  filter_dt_ = config["default"]["low_pass_filter"]["sampling_time"].as<float>();   // [s]

  // limitation
  // (TODO): this may be written in another yaml file based on classify result
  const float max_speed_kmph = config["default"]["limit"]["max_speed"].as<float>();  // [km/h]
  max_vx_ = tier4_autoware_utils::kmph2mps(max_speed_kmph);                          // [m/s]
}

bool ConstantTurnRateMotionTracker::predict(const rclcpp::Time & time)
{
  const double dt = (time - last_update_time_).seconds();
  bool ret = predict(dt, ekf_);
  if (ret) {
    last_update_time_ = time;
  }
  return ret;
}

bool ConstantTurnRateMotionTracker::predict(const double dt, KalmanFilter & ekf) const
{
  /*  == Nonlinear model ==
   *
   * x_{k+1}   = x_k + vx_k * cos(yaw_k) * dt
   * y_{k+1}   = y_k + vx_k * sin(yaw_k) * dt
   * yaw_{k+1} = yaw_k + (wz_k) * dt
   * vx_{k+1}  = vx_k
   * wz_{k+1}  = wz_k
   *
   */

  /*  == Linearized model ==
   *
   * A = [ 1, 0, -vx*sin(yaw)*dt, cos(yaw)*dt,  0]
   *     [ 0, 1,  vx*cos(yaw)*dt, sin(yaw)*dt,  0]
   *     [ 0, 0,               1,           0, dt]
   *     [ 0, 0,               0,           1,  0]
   *     [ 0, 0,               0,           0,  1]
   */

  const double yaw_rate_coeff = assume_zero_yaw_rate_ ? 0.0 : 1.0;

  // X t
  Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);  // predicted state
  ekf.getX(X_t);
  const auto x = X_t(IDX::X);
  const auto y = X_t(IDX::Y);
  const auto yaw = X_t(IDX::YAW);
  const auto vx = X_t(IDX::VX);
  const auto wz = X_t(IDX::WZ);

  // X t+1
  Eigen::MatrixXd X_next_t(ekf_params_.dim_x, 1);
  X_next_t(IDX::X) = x + vx * std::cos(yaw) * dt;
  X_next_t(IDX::Y) = y + vx * std::sin(yaw) * dt;
  X_next_t(IDX::YAW) = yaw + wz * dt * yaw_rate_coeff;
  X_next_t(IDX::VX) = vx;
  X_next_t(IDX::WZ) = wz * yaw_rate_coeff;

  // A: state transition matrix
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(ekf_params_.dim_x, ekf_params_.dim_x);
  A(IDX::X, IDX::YAW) = -vx * std::sin(yaw) * dt;
  A(IDX::Y, IDX::YAW) = vx * std::cos(yaw) * dt;
  A(IDX::X, IDX::VX) = std::cos(yaw) * dt;
  A(IDX::Y, IDX::VX) = std::sin(yaw) * dt;
  A(IDX::YAW, IDX::WZ) = dt * yaw_rate_coeff;

  // Q: system noise
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);

  // Rotate the covariance matrix according to the vehicle yaw
  // because q_cov_x and y are in the vehicle coordinate system.
  Eigen::MatrixXd Q_xy_local = Eigen::MatrixXd::Zero(2, 2);
  Eigen::MatrixXd Q_xy_global = Eigen::MatrixXd::Zero(2, 2);
  Q_xy_local << ekf_params_.q_cov_x, 0.0, 0.0, ekf_params_.q_cov_y;
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
  R << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
  Q_xy_global = R * Q_xy_local * R.transpose();
  Q.block<2, 2>(IDX::X, IDX::X) = Q_xy_global;

  Q(IDX::YAW, IDX::YAW) = ekf_params_.q_cov_yaw;
  Q(IDX::VX, IDX::VX) = ekf_params_.q_cov_vx;
  Q(IDX::WZ, IDX::WZ) = ekf_params_.q_cov_wz;

  // may be unused
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(ekf_params_.dim_x, ekf_params_.dim_x);
  Eigen::MatrixXd u = Eigen::MatrixXd::Zero(ekf_params_.dim_x, 1);

  // call kalman filter library
  if (!ekf.predict(X_next_t, A, Q)) {
    RCLCPP_WARN(logger_, "Cannot predict");
  }

  return true;
}

bool ConstantTurnRateMotionTracker::measureWithPose(
  const autoware_auto_perception_msgs::msg::DetectedObject & object,
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

  // get current state
  Eigen::MatrixXd X(ekf_params_.dim_x, 1);
  ekf_.getX(X);
  const auto yaw_state = X(IDX::YAW);

  // rotation matrix
  Eigen::Matrix2d RotationYaw;
  if (trust_yaw_input_) {
    const auto yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    RotationYaw << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  } else {
    RotationYaw << std::cos(yaw_state), -std::sin(yaw_state), std::sin(yaw_state),
      std::cos(yaw_state);
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

  // 2. add yaw measurement
  const bool object_has_orientation = object.kinematics.orientation_availability >
                                      0;  // 0: not available, 1: sign unknown, 2: available
  const bool enable_yaw_measurement = trust_yaw_input_ && object_has_orientation;

  if (enable_yaw_measurement) {
    Eigen::MatrixXd Cyaw = Eigen::MatrixXd::Zero(1, ekf_params_.dim_x);
    Cyaw(0, IDX::YAW) = 1;
    C_list.push_back(Cyaw);

    Eigen::MatrixXd Yyaw = Eigen::MatrixXd::Zero(1, 1);
    const auto yaw = [&] {
      auto obj_yaw = tier4_autoware_utils::normalizeRadian(
        tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation));
      while (M_PI_2 <= yaw_state - obj_yaw) {
        obj_yaw = obj_yaw + M_PI;
      }
      while (M_PI_2 <= obj_yaw - yaw_state) {
        obj_yaw = obj_yaw - M_PI;
      }
      return obj_yaw;
    }();

    Yyaw << yaw;
    Y_list.push_back(Yyaw);

    Eigen::MatrixXd Ryaw = Eigen::MatrixXd::Zero(1, 1);
    Ryaw << ekf_params_.r_cov_yaw;
    R_block_list.push_back(Ryaw);
  }

  // 3. add linear velocity measurement
  const bool enable_velocity_measurement = object.kinematics.has_twist && trust_twist_input_;
  if (enable_velocity_measurement) {
    Eigen::MatrixXd C_vx = Eigen::MatrixXd::Zero(1, ekf_params_.dim_x);
    C_vx(0, IDX::VX) = 1;
    C_list.push_back(C_vx);

    // measure absolute velocity
    Eigen::MatrixXd Vx = Eigen::MatrixXd::Zero(1, 1);
    Vx << object.kinematics.twist_with_covariance.twist.linear.x;

    Eigen::Matrix2d R_vx = Eigen::MatrixXd::Zero(1, 1);
    if (!object.kinematics.has_twist_covariance) {
      R_vx << ekf_params_.r_cov_vx;
    } else {
      R_vx << object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::X_X];
    }
    R_block_list.push_back(R_vx);
  }

  // 4. sum up matrices
  const auto row_number = std::accumulate(
    C_list.begin(), C_list.end(), 0,
    [](const auto & sum, const auto & C) { return sum + C.rows(); });
  if (row_number == 0) {
    RCLCPP_WARN(logger_, "No measurement is available");
    return false;
  }

  // stacking matrices vertically or diagonally
  const auto C = utils::stackMatricesVertically(C_list);
  const auto Y = utils::stackMatricesVertically(Y_list);
  const auto R = utils::stackMatricesDiagonally(R_block_list);

  // 4. EKF update
  if (!ekf_.update(Y, C, R)) {
    RCLCPP_WARN(logger_, "Cannot update");
  }

  // 5. normalize: limit vx
  {
    Eigen::MatrixXd X_t(ekf_params_.dim_x, 1);
    Eigen::MatrixXd P_t(ekf_params_.dim_x, ekf_params_.dim_x);
    ekf_.getX(X_t);
    ekf_.getP(P_t);
    if (!(-max_vx_ <= X_t(IDX::VX) && X_t(IDX::VX) <= max_vx_)) {
      X_t(IDX::VX) = X_t(IDX::VX) < 0 ? -max_vx_ : max_vx_;
    }
    ekf_.init(X_t, P_t);
  }

  // 6. Filter z
  // first order low pass filter
  const float gain = filter_tau_ / (filter_tau_ + filter_dt_);
  z_ = gain * z_ + (1.0 - gain) * object.kinematics.pose_with_covariance.pose.position.z;
  return true;
}

bool ConstantTurnRateMotionTracker::measureWithShape(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  // just use first order low pass filter
  const float gain = filter_tau_ / (filter_tau_ + filter_dt_);

  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    bounding_box_.length = gain * bounding_box_.length + (1.0 - gain) * object.shape.dimensions.x;
    bounding_box_.width = gain * bounding_box_.width + (1.0 - gain) * object.shape.dimensions.y;
    bounding_box_.height = gain * bounding_box_.height + (1.0 - gain) * object.shape.dimensions.z;
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    cylinder_.width = gain * cylinder_.width + (1.0 - gain) * object.shape.dimensions.x;
    cylinder_.height = gain * cylinder_.height + (1.0 - gain) * object.shape.dimensions.z;
  } else {
    return false;
  }

  return true;
}

bool ConstantTurnRateMotionTracker::measure(
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

  measureWithPose(object, self_transform);
  measureWithShape(object);

  // (void)self_transform;  // currently do not use self vehicle position
  return true;
}

bool ConstantTurnRateMotionTracker::getTrackedObject(
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
  // rotation matrix with yaw_
  Eigen::Matrix2d R_yaw = Eigen::Matrix2d::Zero();
  R_yaw << std::cos(X_t(IDX::YAW)), -std::sin(X_t(IDX::YAW)), std::sin(X_t(IDX::YAW)),
    std::cos(X_t(IDX::YAW));
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
    filtered_quaternion.setRPY(roll, pitch, X_t(IDX::YAW));
    pose_with_cov.pose.orientation.x = filtered_quaternion.x();
    pose_with_cov.pose.orientation.y = filtered_quaternion.y();
    pose_with_cov.pose.orientation.z = filtered_quaternion.z();
    pose_with_cov.pose.orientation.w = filtered_quaternion.w();
    if (trust_yaw_input_) {
      object.kinematics.orientation_availability =
        autoware_auto_perception_msgs::msg::TrackedObjectKinematics::SIGN_UNKNOWN;
    } else {
      object.kinematics.orientation_availability =
        autoware_auto_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE;
    }
  }
  // twist
  // twist need to converted to local coordinate
  const auto vx = X_t(IDX::VX);
  twist_with_cov.twist.linear.x = vx;

  // ===== covariance transformation =====
  // since covariance in EKF is in map coordinate and output should be in object coordinate,
  // we need to transform covariance
  Eigen::Matrix2d P_xy_map;
  P_xy_map << P(IDX::X, IDX::X), P(IDX::X, IDX::Y), P(IDX::Y, IDX::X), P(IDX::Y, IDX::Y);

  // rotate covariance with -yaw
  Eigen::Matrix2d P_xy = R_yaw_inv * P_xy_map * R_yaw_inv.transpose();

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

  twist_with_cov.covariance[utils::MSG_COV_IDX::X_X] = P(IDX::VX, IDX::VX);
  twist_with_cov.covariance[utils::MSG_COV_IDX::X_Y] = 0.0;
  twist_with_cov.covariance[utils::MSG_COV_IDX::Y_X] = 0.0;
  twist_with_cov.covariance[utils::MSG_COV_IDX::Y_Y] = no_info_cov;
  twist_with_cov.covariance[utils::MSG_COV_IDX::Z_Z] = vz_cov;
  twist_with_cov.covariance[utils::MSG_COV_IDX::ROLL_ROLL] = no_info_cov;
  twist_with_cov.covariance[utils::MSG_COV_IDX::PITCH_PITCH] = no_info_cov;
  twist_with_cov.covariance[utils::MSG_COV_IDX::YAW_YAW] = wz_cov;

  // set shape
  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    object.shape.dimensions.x = bounding_box_.length;
    object.shape.dimensions.y = bounding_box_.width;
    object.shape.dimensions.z = bounding_box_.height;
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    object.shape.dimensions.x = cylinder_.width;
    object.shape.dimensions.y = cylinder_.width;
    object.shape.dimensions.z = cylinder_.height;
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    const auto origin_yaw = tf2::getYaw(object_.kinematics.pose_with_covariance.pose.orientation);
    const auto ekf_pose_yaw = tf2::getYaw(pose_with_cov.pose.orientation);
    object.shape.footprint =
      tier4_autoware_utils::rotatePolygon(object.shape.footprint, origin_yaw - ekf_pose_yaw);
  }

  return true;
}
