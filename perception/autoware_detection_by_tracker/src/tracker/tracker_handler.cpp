// Copyright 2021 Tier IV, Inc.
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

#include "tracker_handler.hpp"

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/math/normalization.hpp"

#include <tf2/utils.h>

namespace autoware::detection_by_tracker
{

void TrackerHandler::onTrackedObjects(
  const autoware_perception_msgs::msg::TrackedObjects::ConstSharedPtr input_objects_msg)
{
  constexpr size_t max_buffer_size = 10;

  // Add tracked objects to buffer
  objects_buffer_.push_front(*input_objects_msg);

  // Remove old data
  while (max_buffer_size < objects_buffer_.size()) {
    objects_buffer_.pop_back();
  }
}

bool TrackerHandler::estimateTrackedObjects(
  const rclcpp::Time & time, autoware_perception_msgs::msg::TrackedObjects & output)
{
  if (objects_buffer_.empty()) {
    return false;
  }

  // Get the objects closest to the target time.
  const auto target_objects_iter = std::min_element(
    objects_buffer_.cbegin(), objects_buffer_.cend(),
    [&time](
      autoware_perception_msgs::msg::TrackedObjects first,
      autoware_perception_msgs::msg::TrackedObjects second) {
      return std::fabs((time - first.header.stamp).seconds()) <
             std::fabs((time - second.header.stamp).seconds());
    });

  // Estimate the pose of the object at the target time
  const auto dt = time - target_objects_iter->header.stamp;
  output.header.frame_id = target_objects_iter->header.frame_id;
  output.header.stamp = time;
  for (const auto & object : target_objects_iter->objects) {
    const auto & pose_with_covariance = object.kinematics.pose_with_covariance;
    const auto & x = pose_with_covariance.pose.position.x;
    const auto & y = pose_with_covariance.pose.position.y;
    const auto & z = pose_with_covariance.pose.position.z;
    const float yaw = tf2::getYaw(pose_with_covariance.pose.orientation);
    const auto & twist = object.kinematics.twist_with_covariance.twist;
    const float & vx = twist.linear.x;
    const float & wz = twist.angular.z;

    // build output
    autoware_perception_msgs::msg::TrackedObject estimated_object;
    estimated_object.object_id = object.object_id;
    estimated_object.existence_probability = object.existence_probability;
    estimated_object.classification = object.classification;
    estimated_object.shape = object.shape;
    estimated_object.kinematics.pose_with_covariance.pose.position.x =
      x + vx * std::cos(yaw) * dt.seconds();
    estimated_object.kinematics.pose_with_covariance.pose.position.y =
      y + vx * std::sin(yaw) * dt.seconds();
    estimated_object.kinematics.pose_with_covariance.pose.position.z = z;
    const float yaw_hat = autoware::universe_utils::normalizeRadian(yaw + wz * dt.seconds());
    estimated_object.kinematics.pose_with_covariance.pose.orientation =
      autoware::universe_utils::createQuaternionFromYaw(yaw_hat);
    output.objects.push_back(estimated_object);
  }
  return true;
}
}  // namespace autoware::detection_by_tracker
