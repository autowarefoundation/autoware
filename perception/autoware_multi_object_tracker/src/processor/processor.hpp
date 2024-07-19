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

#ifndef PROCESSOR__PROCESSOR_HPP_
#define PROCESSOR__PROCESSOR_HPP_

#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"

#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::multi_object_tracker
{
class TrackerProcessor
{
public:
  explicit TrackerProcessor(
    const std::map<std::uint8_t, std::string> & tracker_map, const size_t & channel_size);

  const std::list<std::shared_ptr<Tracker>> & getListTracker() const { return list_tracker_; }
  // tracker processes
  void predict(const rclcpp::Time & time);
  void update(
    const autoware_perception_msgs::msg::DetectedObjects & detected_objects,
    const geometry_msgs::msg::Transform & self_transform,
    const std::unordered_map<int, int> & direct_assignment, const uint & channel_index);
  void spawn(
    const autoware_perception_msgs::msg::DetectedObjects & detected_objects,
    const geometry_msgs::msg::Transform & self_transform,
    const std::unordered_map<int, int> & reverse_assignment, const uint & channel_index);
  void prune(const rclcpp::Time & time);

  // output
  bool isConfidentTracker(const std::shared_ptr<Tracker> & tracker) const;
  void getTrackedObjects(
    const rclcpp::Time & time,
    autoware_perception_msgs::msg::TrackedObjects & tracked_objects) const;
  void getTentativeObjects(
    const rclcpp::Time & time,
    autoware_perception_msgs::msg::TrackedObjects & tentative_objects) const;

  void getExistenceProbabilities(std::vector<std::vector<float>> & existence_vectors) const;

private:
  std::map<std::uint8_t, std::string> tracker_map_;
  std::list<std::shared_ptr<Tracker>> list_tracker_;
  const size_t channel_size_;

  // parameters
  float max_elapsed_time_;            // [s]
  float min_iou_;                     // [ratio]
  float min_iou_for_unknown_object_;  // [ratio]
  double distance_threshold_;         // [m]
  int confident_count_threshold_;     // [count]

  void removeOldTracker(const rclcpp::Time & time);
  void removeOverlappedTracker(const rclcpp::Time & time);
  std::shared_ptr<Tracker> createNewTracker(
    const autoware_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
    const geometry_msgs::msg::Transform & self_transform, const uint & channel_index) const;
};

}  // namespace autoware::multi_object_tracker

#endif  // PROCESSOR__PROCESSOR_HPP_
