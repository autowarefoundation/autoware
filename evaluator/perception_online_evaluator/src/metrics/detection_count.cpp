// Copyright 2024 TIER IV, Inc.
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

#include "perception_online_evaluator/metrics/detection_count.hpp"

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "object_recognition_utils/object_recognition_utils.hpp"
#include "perception_online_evaluator/utils/objects_filtering.hpp"

#include <autoware/universe_utils/ros/uuid_helper.hpp>

namespace perception_diagnostics
{
namespace metrics
{
using autoware::universe_utils::toHexString;

bool isCountObject(
  const std::uint8_t classification, const std::unordered_map<uint8_t, ObjectParameter> & params)
{
  const auto & p = params;

  if (p.find(classification) == p.end()) {
    return false;
  }

  return p.at(classification).check_total_objects_count ||
         p.at(classification).check_average_objects_count ||
         p.at(classification).check_interval_average_objects_count;
}

DetectionRanges DetectionCounter::getRanges() const
{
  DetectionRanges ranges;
  for (const auto & radius : parameters_->detection_radius_list) {
    for (const auto & height : parameters_->detection_height_list) {
      ranges.push_back(DetectionRange(radius, height));
    }
  }

  return ranges;
}

void DetectionCounter::addObjects(
  const PredictedObjects & objects, const tf2_ros::Buffer & tf_buffer)
{
  // initialize the data structures if new class or new range is detected
  initializeDetectionMap();

  const auto objects_frame_id = objects.header.frame_id;
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer.lookupTransform(
      "base_link", objects_frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    return;
  }

  const auto timestamp = objects.header.stamp;
  unique_timestamps_.insert(timestamp);

  for (const auto & object : objects.objects) {
    const auto uuid = toHexString(object.object_id);
    const auto label = object_recognition_utils::getHighestProbLabel(object.classification);
    if (!isCountObject(label, parameters_->object_parameters)) {
      continue;
    }
    for (const auto & range : getRanges()) {
      geometry_msgs::msg::PoseStamped pose_in, pose_out;
      pose_in.header.frame_id = objects_frame_id;
      pose_in.pose = object.kinematics.initial_pose_with_covariance.pose;

      // Transform the object's pose into the 'base_link' coordinate frame
      tf2::doTransform(pose_in, pose_out, transform_stamped);

      const double distance_to_base_link =
        std::hypot(pose_out.pose.position.x, pose_out.pose.position.y);

      // If the pose is within the detection_radius and below a detection_height, increment the
      // count
      const bool is_within_detection_radius = distance_to_base_link < range.radius;
      const bool is_below_detection_height = pose_out.pose.position.z < range.height;
      if (is_within_detection_radius && is_below_detection_height) {
        updateDetectionMap(uuid, label, range.toString(), timestamp);
      }
    }
  }
}

void DetectionCounter::initializeDetectionMap()
{
  for (std::uint8_t i = ObjectClassification::UNKNOWN; i < ObjectClassification::PEDESTRIAN + 1;
       ++i) {
    for (const auto & range : getRanges()) {
      std::string range_str = range.toString();
      if (time_series_counts_.find(i) == time_series_counts_.end()) {
        time_series_counts_[i][range_str] = std::vector<rclcpp::Time>();
        seen_uuids_[i][range_str] = std::set<std::string>();
      } else if (time_series_counts_[i].find(range_str) == time_series_counts_[i].end()) {
        time_series_counts_[i][range_str] = std::vector<rclcpp::Time>();
        seen_uuids_[i][range_str] = std::set<std::string>();
      }
    }
  }
}

void DetectionCounter::updateDetectionMap(
  const std::string uuid, const std::uint8_t classification, const std::string & range,
  const rclcpp::Time & timestamp)
{
  seen_uuids_[classification][range].insert(uuid);

  // Record the detection time for averaging
  time_series_counts_[classification][range].push_back(timestamp);

  // Purge old records if necessary
  purgeOldRecords(timestamp);
}

std::unordered_map<std::uint8_t, std::unordered_map<std::string, double>>
DetectionCounter::getAverageCount(double seconds) const
{
  std::unordered_map<std::uint8_t, std::unordered_map<std::string, double>> averages;
  if (unique_timestamps_.size() == 0) {
    return averages;
  }
  const rclcpp::Time cutoff_time =
    *unique_timestamps_.rbegin() - rclcpp::Duration::from_seconds(seconds);

  for (auto & [classification, range_map] : time_series_counts_) {
    if (
      parameters_->object_parameters.find(classification) == parameters_->object_parameters.end() ||
      !parameters_->object_parameters.at(classification).check_interval_average_objects_count) {
      continue;
    }
    for (auto & [range, timestamps] : range_map) {
      // Calculate the number of detections within the given time window
      const size_t count = std::count_if(
        timestamps.begin(), timestamps.end(),
        [&](const rclcpp::Time & time) { return time >= cutoff_time; });

      // Calculate the number of unique frames within the given time window
      const size_t frame_count = std::count_if(
        unique_timestamps_.begin(), unique_timestamps_.end(),
        [&](const rclcpp::Time & time) { return time >= cutoff_time; });

      if (frame_count > 0) {
        averages[classification][range] = static_cast<double>(count) / frame_count;
      } else {
        averages[classification][range] = 0;
      }
    }
  }

  return averages;
}

std::unordered_map<std::uint8_t, std::unordered_map<std::string, double>>
DetectionCounter::getOverallAverageCount() const
{
  std::unordered_map<std::uint8_t, std::unordered_map<std::string, double>> averages;

  for (auto & [classification, range_map] : time_series_counts_) {
    if (
      parameters_->object_parameters.find(classification) == parameters_->object_parameters.end() ||
      !parameters_->object_parameters.at(classification).check_average_objects_count) {
      continue;
    }
    for (auto & [range, timestamps] : range_map) {
      const size_t count = timestamps.size();
      const size_t frame_count = unique_timestamps_.size();
      if (frame_count > 0) {
        averages[classification][range] = static_cast<double>(count) / frame_count;
      } else {
        averages[classification][range] = 0;
      }
    }
  }

  return averages;
}

std::unordered_map<std::uint8_t, std::unordered_map<std::string, size_t>>
DetectionCounter::getTotalCount() const
{
  std::unordered_map<std::uint8_t, std::unordered_map<std::string, size_t>> total_counts;
  for (auto & [classification, range_map] : seen_uuids_) {
    if (
      parameters_->object_parameters.find(classification) == parameters_->object_parameters.end() ||
      !parameters_->object_parameters.at(classification).check_total_objects_count) {
      continue;
    }
    for (auto & [range, uuids] : range_map) {
      total_counts[classification][range] = uuids.size();
    }
  }

  return total_counts;
}

void DetectionCounter::purgeOldRecords(rclcpp::Time current_time)
{
  const rclcpp::Time cutoff_time =
    current_time - rclcpp::Duration::from_seconds(parameters_->detection_count_purge_seconds);

  // Remove timestamps older than purge_seconds_ from time_series_counts_
  for (auto & [classification, range_map] : time_series_counts_) {
    for (auto & [range, timestamps] : range_map) {
      timestamps.erase(
        std::remove_if(
          timestamps.begin(), timestamps.end(),
          [&](const rclcpp::Time & time) { return time < cutoff_time; }),
        timestamps.end());
    }
  }

  // Remove timestamps older than purge_seconds_ from unique_timestamps_
  while (!unique_timestamps_.empty() && *unique_timestamps_.begin() < cutoff_time) {
    unique_timestamps_.erase(unique_timestamps_.begin());
  }
}
}  // namespace metrics
}  // namespace perception_diagnostics
