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

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_

#include "kalman_filter/kalman_filter.hpp"
#include "multi_object_tracker/tracker/motion_model/motion_model_base.hpp"

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <geometry_msgs/msg/twist.hpp>

class BicycleMotionModel : public MotionModel
{
private:
  // attributes
  rclcpp::Logger logger_;

  // extended state
  double lf_;
  double lr_;

  // motion parameters
  struct MotionParams
  {
    double q_stddev_acc_long;
    double q_stddev_acc_lat;
    double q_cov_acc_long;
    double q_cov_acc_lat;
    double q_stddev_yaw_rate_min;
    double q_stddev_yaw_rate_max;
    double q_cov_slip_rate_min;
    double q_cov_slip_rate_max;
    double q_max_slip_angle;
    double lf_ratio;
    double lr_ratio;
    double lf_min;
    double lr_min;
    double max_vel;
    double max_slip;
  } motion_params_;

public:
  BicycleMotionModel();

  enum IDX { X = 0, Y = 1, YAW = 2, VEL = 3, SLIP = 4 };
  const char DIM = 5;

  bool initialize(
    const rclcpp::Time & time, const double & x, const double & y, const double & yaw,
    const std::array<double, 36> & pose_cov, const double & vel, const double & vel_cov,
    const double & slip, const double & slip_cov, const double & length);

  void setDefaultParams();

  void setMotionParams(
    const double & q_stddev_acc_long, const double & q_stddev_acc_lat,
    const double & q_stddev_yaw_rate_min, const double & q_stddev_yaw_rate_max,
    const double & q_stddev_slip_rate_min, const double & q_stddev_slip_rate_max,
    const double & q_max_slip_angle, const double & lf_ratio, const double & lf_min,
    const double & lr_ratio, const double & lr_min);

  void setMotionLimits(const double & max_vel, const double & max_slip);

  bool updateStatePose(const double & x, const double & y, const std::array<double, 36> & pose_cov);

  bool updateStatePoseHead(
    const double & x, const double & y, const double & yaw,
    const std::array<double, 36> & pose_cov);

  bool updateStatePoseHeadVel(
    const double & x, const double & y, const double & yaw, const std::array<double, 36> & pose_cov,
    const double & vel, const std::array<double, 36> & twist_cov);

  bool adjustPosition(const double & x, const double & y);

  bool limitStates();

  bool updateExtendedState(const double & length);

  bool predictStateStep(const double dt, KalmanFilter & ekf) const override;

  bool getPredictedState(
    const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
    geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const override;
};

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__BICYCLE_MOTION_MODEL_HPP_
