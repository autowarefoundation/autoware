// Copyright 2015-2019 Autoware Foundation
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

#ifndef LOCALIZATION_UTIL__SMART_POSE_BUFFER_HPP_
#define LOCALIZATION_UTIL__SMART_POSE_BUFFER_HPP_

#include "localization_util/util_func.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <deque>

class SmartPoseBuffer
{
private:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

public:
  struct InterpolateResult
  {
    PoseWithCovarianceStamped old_pose;
    PoseWithCovarianceStamped new_pose;
    PoseWithCovarianceStamped interpolated_pose;
  };

  SmartPoseBuffer() = delete;
  SmartPoseBuffer(
    const rclcpp::Logger & logger, const double & pose_timeout_sec,
    const double & pose_distance_tolerance_meters);

  std::optional<InterpolateResult> interpolate(const rclcpp::Time & target_ros_time);

  void push_back(const PoseWithCovarianceStamped::ConstSharedPtr & pose_msg_ptr);

  void pop_old(const rclcpp::Time & target_ros_time);

  void clear();

private:
  rclcpp::Logger logger_;
  std::deque<PoseWithCovarianceStamped::ConstSharedPtr> pose_buffer_;
  std::mutex mutex_;  // This mutex is for pose_buffer_

  const double pose_timeout_sec_;
  const double pose_distance_tolerance_meters_;

  [[nodiscard]] bool validate_time_stamp_difference(
    const rclcpp::Time & target_time, const rclcpp::Time & reference_time,
    const double time_tolerance_sec) const;
  [[nodiscard]] bool validate_position_difference(
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & reference_point, const double distance_tolerance_m_) const;
};

#endif  // LOCALIZATION_UTIL__SMART_POSE_BUFFER_HPP_
