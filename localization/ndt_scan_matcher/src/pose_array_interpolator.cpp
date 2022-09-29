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

#include "ndt_scan_matcher/pose_array_interpolator.hpp"

PoseArrayInterpolator::PoseArrayInterpolator(
  rclcpp::Node * node, const rclcpp::Time target_ros_time,
  const std::deque<PoseWithCovarianceStamped::ConstSharedPtr> & pose_msg_ptr_array)
: logger_(node->get_logger()),
  clock_(*node->get_clock()),
  current_pose_ptr_(new PoseWithCovarianceStamped),
  old_pose_ptr_(new PoseWithCovarianceStamped),
  new_pose_ptr_(new PoseWithCovarianceStamped),
  success_(true)
{
  get_nearest_timestamp_pose(pose_msg_ptr_array, target_ros_time, old_pose_ptr_, new_pose_ptr_);
  const geometry_msgs::msg::PoseStamped interpolated_pose_msg =
    interpolate_pose(*old_pose_ptr_, *new_pose_ptr_, target_ros_time);
  current_pose_ptr_->header = interpolated_pose_msg.header;
  current_pose_ptr_->pose.pose = interpolated_pose_msg.pose;
  current_pose_ptr_->pose.covariance = old_pose_ptr_->pose.covariance;
}

PoseArrayInterpolator::PoseArrayInterpolator(
  rclcpp::Node * node, const rclcpp::Time target_ros_time,
  const std::deque<PoseWithCovarianceStamped::ConstSharedPtr> & pose_msg_ptr_array,
  const double & pose_timeout_sec, const double & pose_distance_tolerance_meters)
: PoseArrayInterpolator(node, target_ros_time, pose_msg_ptr_array)
{
  // check the time stamp
  bool is_old_pose_valid =
    validate_time_stamp_difference(old_pose_ptr_->header.stamp, target_ros_time, pose_timeout_sec);
  bool is_new_pose_valid =
    validate_time_stamp_difference(new_pose_ptr_->header.stamp, target_ros_time, pose_timeout_sec);

  // check the position jumping (ex. immediately after the initial pose estimation)
  bool is_pose_diff_valid = validate_position_difference(
    old_pose_ptr_->pose.pose.position, new_pose_ptr_->pose.pose.position,
    pose_distance_tolerance_meters);

  // all validations must be true
  if (!(is_old_pose_valid & is_new_pose_valid & is_pose_diff_valid)) {
    RCLCPP_WARN(logger_, "Validation error.");
  }
}

geometry_msgs::msg::PoseWithCovarianceStamped PoseArrayInterpolator::get_current_pose()
{
  return *current_pose_ptr_;
}

geometry_msgs::msg::PoseWithCovarianceStamped PoseArrayInterpolator::get_old_pose()
{
  return *old_pose_ptr_;
}

geometry_msgs::msg::PoseWithCovarianceStamped PoseArrayInterpolator::get_new_pose()
{
  return *new_pose_ptr_;
}

bool PoseArrayInterpolator::is_success() { return success_; }

bool PoseArrayInterpolator::validate_time_stamp_difference(
  const rclcpp::Time & target_time, const rclcpp::Time & reference_time,
  const double time_tolerance_sec) const
{
  const double dt = std::abs((target_time - reference_time).seconds());
  bool success = dt < time_tolerance_sec;
  if (!success) {
    RCLCPP_WARN(
      logger_,
      "Validation error. The reference time is %lf[sec], but the target time is %lf[sec]. The "
      "difference is %lf[sec] (the tolerance is %lf[sec]).",
      reference_time.seconds(), target_time.seconds(), dt, time_tolerance_sec);
  }
  return success;
}

bool PoseArrayInterpolator::validate_position_difference(
  const geometry_msgs::msg::Point & target_point, const geometry_msgs::msg::Point & reference_point,
  const double distance_tolerance_m_) const
{
  double distance = norm(target_point, reference_point);
  bool success = distance < distance_tolerance_m_;
  if (!success) {
    RCLCPP_WARN(
      logger_,
      "Validation error. The distance from reference position to target position is %lf[m] (the "
      "tolerance is %lf[m]).",
      distance, distance_tolerance_m_);
  }
  return success;
}
