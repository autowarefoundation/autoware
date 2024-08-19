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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__MOTION_MODEL_BASE_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__MOTION_MODEL_BASE_HPP_

#include "autoware/kalman_filter/kalman_filter.hpp"

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <geometry_msgs/msg/twist.hpp>

namespace autoware::multi_object_tracker
{
using autoware::kalman_filter::KalmanFilter;

class MotionModel
{
private:
  bool is_initialized_{false};
  double dt_max_{0.11};  // [s] maximum time interval for prediction

protected:
  rclcpp::Time last_update_time_;
  KalmanFilter ekf_;

public:
  MotionModel();
  virtual ~MotionModel() = default;

  bool checkInitialized() const { return is_initialized_; }
  double getDeltaTime(const rclcpp::Time & time) const
  {
    return (time - last_update_time_).seconds();
  }
  void setMaxDeltaTime(const double dt_max) { dt_max_ = dt_max; }
  double getStateElement(unsigned int idx) const { return ekf_.getXelement(idx); }
  void getStateVector(Eigen::MatrixXd & X) const { ekf_.getX(X); }

  bool initialize(const rclcpp::Time & time, const Eigen::MatrixXd & X, const Eigen::MatrixXd & P);

  bool predictState(const rclcpp::Time & time);
  bool getPredictedState(const rclcpp::Time & time, Eigen::MatrixXd & X, Eigen::MatrixXd & P) const;

  virtual bool predictStateStep(const double dt, KalmanFilter & ekf) const = 0;
  virtual bool getPredictedState(
    const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
    geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const = 0;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MOTION_MODEL__MOTION_MODEL_BASE_HPP_
