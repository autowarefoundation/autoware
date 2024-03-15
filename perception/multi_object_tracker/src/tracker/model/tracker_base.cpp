// Copyright 2020 Tier IV, Inc.
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

#include "multi_object_tracker/tracker/model/tracker_base.hpp"

#include "multi_object_tracker/utils/utils.hpp"

#include <algorithm>
#include <random>

Tracker::Tracker(
  const rclcpp::Time & time,
  const std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> & classification)
: classification_(classification),
  no_measurement_count_(0),
  total_no_measurement_count_(0),
  total_measurement_count_(1),
  last_update_with_measurement_time_(time)
{
  // Generate random number
  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid_.uuid.begin(), uuid_.uuid.end(), bit_eng);
}

bool Tracker::updateWithMeasurement(
  const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const rclcpp::Time & measurement_time, const geometry_msgs::msg::Transform & self_transform)
{
  no_measurement_count_ = 0;
  ++total_measurement_count_;
  last_update_with_measurement_time_ = measurement_time;
  measure(object, measurement_time, self_transform);
  return true;
}

bool Tracker::updateWithoutMeasurement()
{
  ++no_measurement_count_;
  ++total_no_measurement_count_;
  return true;
}

void Tracker::updateClassification(
  const std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> & classification)
{
  // classification algorithm:
  // 0. Normalize the input classification
  // 1-1. Update the matched classification probability with a gain (ratio of 0.05)
  // 1-2. If the label is not found, add it to the classification list
  // 2. Remove the class with probability < remove_threshold (0.001)
  // 3. Normalize tracking classification

  // Parameters
  // if the remove_threshold is too high (compare to the gain), the classification will be removed
  // immediately
  const double gain = 0.05;
  constexpr double remove_threshold = 0.001;

  // Normalization function
  auto normalizeProbabilities =
    [](std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> & classification) {
      double sum = 0.0;
      for (const auto & class_ : classification) {
        sum += class_.probability;
      }
      for (auto & class_ : classification) {
        class_.probability /= sum;
      }
    };

  // Normalize the input
  auto classification_input = classification;
  normalizeProbabilities(classification_input);

  // Update the matched classification probability with a gain
  for (const auto & new_class : classification_input) {
    bool found = false;
    for (auto & old_class : classification_) {
      if (new_class.label == old_class.label) {
        old_class.probability += new_class.probability * gain;
        found = true;
        break;
      }
    }
    // If the label is not found, add it to the classification list
    if (!found) {
      auto adding_class = new_class;
      adding_class.probability *= gain;
      classification_.push_back(adding_class);
    }
  }

  // If the probability is less than the threshold, remove the class
  classification_.erase(
    std::remove_if(
      classification_.begin(), classification_.end(),
      [remove_threshold](const auto & class_) { return class_.probability < remove_threshold; }),
    classification_.end());

  // Normalize tracking classification
  normalizeProbabilities(classification_);
}

geometry_msgs::msg::PoseWithCovariance Tracker::getPoseWithCovariance(
  const rclcpp::Time & time) const
{
  autoware_auto_perception_msgs::msg::TrackedObject object;
  getTrackedObject(time, object);
  return object.kinematics.pose_with_covariance;
}
