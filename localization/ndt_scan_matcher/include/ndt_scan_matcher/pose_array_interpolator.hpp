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

#ifndef NDT_SCAN_MATCHER__POSE_ARRAY_INTERPOLATOR_HPP_
#define NDT_SCAN_MATCHER__POSE_ARRAY_INTERPOLATOR_HPP_

#include "ndt_scan_matcher/util_func.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <deque>

class PoseArrayInterpolator
{
private:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

public:
  PoseArrayInterpolator(
    rclcpp::Node * node, const rclcpp::Time target_ros_time,
    const std::deque<PoseWithCovarianceStamped::ConstSharedPtr> & pose_msg_ptr_array,
    const double & pose_timeout_sec, const double & pose_distance_tolerance_meters);

  PoseArrayInterpolator(
    rclcpp::Node * node, const rclcpp::Time target_ros_time,
    const std::deque<PoseWithCovarianceStamped::ConstSharedPtr> & pose_msg_ptr_array);

  PoseWithCovarianceStamped get_current_pose();
  PoseWithCovarianceStamped get_old_pose();
  PoseWithCovarianceStamped get_new_pose();
  bool is_success();

private:
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  const PoseWithCovarianceStamped::SharedPtr current_pose_ptr_;
  PoseWithCovarianceStamped::SharedPtr old_pose_ptr_;
  PoseWithCovarianceStamped::SharedPtr new_pose_ptr_;
  bool success_;

  bool validate_time_stamp_difference(
    const rclcpp::Time & target_time, const rclcpp::Time & reference_time,
    const double time_tolerance_sec) const;
  bool validate_position_difference(
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & reference_point, const double distance_tolerance_m_) const;
};

#endif  // NDT_SCAN_MATCHER__POSE_ARRAY_INTERPOLATOR_HPP_
