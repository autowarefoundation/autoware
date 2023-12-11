// Copyright 2023- Autoware Foundation
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

#include "localization_util/smart_pose_buffer.hpp"

SmartPoseBuffer::SmartPoseBuffer(
  const rclcpp::Logger & logger, const double & pose_timeout_sec,
  const double & pose_distance_tolerance_meters)
: logger_(logger),
  pose_timeout_sec_(pose_timeout_sec),
  pose_distance_tolerance_meters_(pose_distance_tolerance_meters)
{
}

std::optional<SmartPoseBuffer::InterpolateResult> SmartPoseBuffer::interpolate(
  const rclcpp::Time & target_ros_time)
{
  InterpolateResult result;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (pose_buffer_.size() < 2) {
      RCLCPP_INFO(logger_, "pose_buffer_.size() < 2");
      return std::nullopt;
    }

    const rclcpp::Time time_first = pose_buffer_.front()->header.stamp;
    const rclcpp::Time time_last = pose_buffer_.back()->header.stamp;

    if (target_ros_time < time_first) {
      return std::nullopt;
    }

    // [time_last < target_ros_time] is acceptable here.
    // It is possible that the target_ros_time (often sensor timestamp) is newer than the latest
    // timestamp of buffered pose (often EKF).
    // However, if the timestamp difference is too large,
    // it will later be rejected by validate_time_stamp_difference.

    // get the nearest poses
    result.old_pose = *pose_buffer_.front();
    for (const PoseWithCovarianceStamped::ConstSharedPtr & pose_cov_msg_ptr : pose_buffer_) {
      result.new_pose = *pose_cov_msg_ptr;
      const rclcpp::Time pose_time_stamp = result.new_pose.header.stamp;
      if (pose_time_stamp > target_ros_time) {
        break;
      }
      result.old_pose = *pose_cov_msg_ptr;
    }
  }

  // check the time stamp
  const bool is_old_pose_valid = validate_time_stamp_difference(
    result.old_pose.header.stamp, target_ros_time, pose_timeout_sec_);
  const bool is_new_pose_valid = validate_time_stamp_difference(
    result.new_pose.header.stamp, target_ros_time, pose_timeout_sec_);

  // check the position jumping (ex. immediately after the initial pose estimation)
  const bool is_pose_diff_valid = validate_position_difference(
    result.old_pose.pose.pose.position, result.new_pose.pose.pose.position,
    pose_distance_tolerance_meters_);

  // all validations must be true
  if (!(is_old_pose_valid && is_new_pose_valid && is_pose_diff_valid)) {
    return std::nullopt;
  }

  // interpolate the pose
  const geometry_msgs::msg::PoseStamped interpolated_pose_msg =
    interpolate_pose(result.old_pose, result.new_pose, target_ros_time);
  result.interpolated_pose.header = interpolated_pose_msg.header;
  result.interpolated_pose.pose.pose = interpolated_pose_msg.pose;
  // This does not interpolate the covariance.
  // interpolated_pose.pose.covariance is always set to old_pose.covariance
  result.interpolated_pose.pose.covariance = result.old_pose.pose.covariance;
  return result;
}

void SmartPoseBuffer::push_back(const PoseWithCovarianceStamped::ConstSharedPtr & pose_msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pose_buffer_.empty()) {
    // Check for non-chronological timestamp order
    // This situation can arise when replaying a rosbag multiple times
    const rclcpp::Time end_time = pose_buffer_.back()->header.stamp;
    const rclcpp::Time msg_time = pose_msg_ptr->header.stamp;
    if (msg_time < end_time) {
      // Clear the buffer if timestamps are reversed to maintain chronological order
      pose_buffer_.clear();
    }
  }
  pose_buffer_.push_back(pose_msg_ptr);
}

void SmartPoseBuffer::pop_old(const rclcpp::Time & target_ros_time)
{
  std::lock_guard<std::mutex> lock(mutex_);
  while (!pose_buffer_.empty()) {
    if (rclcpp::Time(pose_buffer_.front()->header.stamp) >= target_ros_time) {
      break;
    }
    pose_buffer_.pop_front();
  }
}

void SmartPoseBuffer::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pose_buffer_.clear();
}

bool SmartPoseBuffer::validate_time_stamp_difference(
  const rclcpp::Time & target_time, const rclcpp::Time & reference_time,
  const double time_tolerance_sec) const
{
  const double dt = std::abs((target_time - reference_time).seconds());
  const bool success = dt < time_tolerance_sec;
  if (!success) {
    RCLCPP_WARN(
      logger_,
      "Validation error. The reference time is %lf[sec], but the target time is %lf[sec]. The "
      "difference is %lf[sec] (the tolerance is %lf[sec]).",
      reference_time.seconds(), target_time.seconds(), dt, time_tolerance_sec);
  }
  return success;
}

bool SmartPoseBuffer::validate_position_difference(
  const geometry_msgs::msg::Point & target_point, const geometry_msgs::msg::Point & reference_point,
  const double distance_tolerance_m_) const
{
  const double distance = norm(target_point, reference_point);
  const bool success = distance < distance_tolerance_m_;
  if (!success) {
    RCLCPP_WARN(
      logger_,
      "Validation error. The distance from reference position to target position is %lf[m] (the "
      "tolerance is %lf[m]).",
      distance, distance_tolerance_m_);
  }
  return success;
}
