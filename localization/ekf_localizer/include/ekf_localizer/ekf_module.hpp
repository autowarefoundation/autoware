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

#ifndef EKF_LOCALIZER__EKF_MODULE_HPP_
#define EKF_LOCALIZER__EKF_MODULE_HPP_

#include "ekf_localizer/hyper_parameters.hpp"
#include "ekf_localizer/state_index.hpp"
#include "ekf_localizer/warning.hpp"

#include <kalman_filter/kalman_filter.hpp>
#include <kalman_filter/time_delay_kalman_filter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <memory>
#include <vector>

struct EKFDiagnosticInfo
{
  EKFDiagnosticInfo()
  : no_update_count(0),
    queue_size(0),
    is_passed_delay_gate(true),
    delay_time(0),
    delay_time_threshold(0),
    is_passed_mahalanobis_gate(true),
    mahalanobis_distance(0)
  {
  }

  size_t no_update_count;
  size_t queue_size;
  bool is_passed_delay_gate;
  double delay_time;
  double delay_time_threshold;
  bool is_passed_mahalanobis_gate;
  double mahalanobis_distance;
};

class EKFModule
{
private:
  using PoseWithCovariance = geometry_msgs::msg::PoseWithCovarianceStamped;
  using TwistWithCovariance = geometry_msgs::msg::TwistWithCovarianceStamped;
  using Pose = geometry_msgs::msg::PoseStamped;
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  EKFModule(std::shared_ptr<Warning> warning, const HyperParameters params);

  void initialize(
    const PoseWithCovariance & initial_pose,
    const geometry_msgs::msg::TransformStamped & transform);

  geometry_msgs::msg::PoseStamped getCurrentPose(
    const rclcpp::Time & current_time, const double z, const double roll, const double pitch,
    bool get_biased_yaw) const;
  geometry_msgs::msg::TwistStamped getCurrentTwist(const rclcpp::Time & current_time) const;
  double getYawBias() const;
  std::array<double, 36> getCurrentPoseCovariance() const;
  std::array<double, 36> getCurrentTwistCovariance() const;

  size_t find_closest_delay_time_index(double target_value) const;
  void accumulate_delay_time(const double dt);

  void predictWithDelay(const double dt);
  bool measurementUpdatePose(
    const PoseWithCovariance & pose, const rclcpp::Time & t_curr,
    EKFDiagnosticInfo & pose_diag_info);
  bool measurementUpdateTwist(
    const TwistWithCovariance & twist, const rclcpp::Time & t_curr,
    EKFDiagnosticInfo & twist_diag_info);
  geometry_msgs::msg::PoseWithCovarianceStamped compensatePoseWithZDelay(
    const PoseWithCovariance & pose, const double delay_time);

private:
  TimeDelayKalmanFilter kalman_filter_;

  std::shared_ptr<Warning> warning_;
  const int dim_x_;
  std::vector<double> accumulated_delay_times_;
  const HyperParameters params_;
};

#endif  // EKF_LOCALIZER__EKF_MODULE_HPP_
