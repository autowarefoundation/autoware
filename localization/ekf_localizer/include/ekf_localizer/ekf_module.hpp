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

#include <autoware/kalman_filter/kalman_filter.hpp>
#include <autoware/kalman_filter/time_delay_kalman_filter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <tf2/utils.h>

#include <memory>
#include <vector>

using autoware::kalman_filter::TimeDelayKalmanFilter;

struct EKFDiagnosticInfo
{
  size_t no_update_count{0};
  size_t queue_size{0};
  bool is_passed_delay_gate{true};
  double delay_time{0.0};
  double delay_time_threshold{0.0};
  bool is_passed_mahalanobis_gate{true};
  double mahalanobis_distance{0.0};
};

class EKFModule
{
private:
  using PoseWithCovariance = geometry_msgs::msg::PoseWithCovarianceStamped;
  using TwistWithCovariance = geometry_msgs::msg::TwistWithCovarianceStamped;
  using Pose = geometry_msgs::msg::PoseStamped;
  using Twist = geometry_msgs::msg::TwistStamped;

public:
  EKFModule(std::shared_ptr<Warning> warning, const HyperParameters & params);

  void initialize(
    const PoseWithCovariance & initial_pose,
    const geometry_msgs::msg::TransformStamped & transform);

  [[nodiscard]] geometry_msgs::msg::PoseStamped get_current_pose(
    const rclcpp::Time & current_time, const double z, const double roll, const double pitch,
    bool get_biased_yaw) const;
  [[nodiscard]] geometry_msgs::msg::TwistStamped get_current_twist(
    const rclcpp::Time & current_time) const;
  [[nodiscard]] double get_yaw_bias() const;
  [[nodiscard]] std::array<double, 36> get_current_pose_covariance() const;
  [[nodiscard]] std::array<double, 36> get_current_twist_covariance() const;

  [[nodiscard]] size_t find_closest_delay_time_index(double target_value) const;
  void accumulate_delay_time(const double dt);

  void predict_with_delay(const double dt);
  bool measurement_update_pose(
    const PoseWithCovariance & pose, const rclcpp::Time & t_curr,
    EKFDiagnosticInfo & pose_diag_info);
  bool measurement_update_twist(
    const TwistWithCovariance & twist, const rclcpp::Time & t_curr,
    EKFDiagnosticInfo & twist_diag_info);
  geometry_msgs::msg::PoseWithCovarianceStamped compensate_rph_with_delay(
    const PoseWithCovariance & pose, tf2::Vector3 last_angular_velocity, const double delay_time);

private:
  TimeDelayKalmanFilter kalman_filter_;

  std::shared_ptr<Warning> warning_;
  const int dim_x_;
  std::vector<double> accumulated_delay_times_;
  const HyperParameters params_;
};

#endif  // EKF_LOCALIZER__EKF_MODULE_HPP_
