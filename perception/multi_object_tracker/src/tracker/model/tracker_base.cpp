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

namespace
{
float updateProbability(float prior, float true_positive, float false_positive)
{
  return (prior * true_positive) / (prior * true_positive + (1 - prior) * false_positive);
}
}  // namespace

Tracker::Tracker(
  const rclcpp::Time & time,
  const std::vector<autoware_auto_perception_msgs::msg::ObjectClassification> & classification,
  const size_t & channel_size)
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

  // Initialize existence probabilities
  existence_probabilities_.resize(channel_size, 0.0);
}

void Tracker::initializeExistenceProbabilities(
  const uint & channel_index, const float & existence_probability)
{
  // The initial existence probability is modeled
  // since the incoming object's existence probability is not reliable
  // existence probability on each channel
  constexpr float initial_existence_probability = 0.1;
  existence_probabilities_[channel_index] = initial_existence_probability;

  // total existence probability
  total_existence_probability_ = existence_probability;
}

bool Tracker::updateWithMeasurement(
  const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const rclcpp::Time & measurement_time, const geometry_msgs::msg::Transform & self_transform,
  const uint & channel_index)
{
  // Update existence probability
  {
    no_measurement_count_ = 0;
    ++total_measurement_count_;

    // existence probability on each channel
    const double delta_time = (measurement_time - last_update_with_measurement_time_).seconds();
    const double decay_rate = -log(0.5) / 0.3;  // 50% decay in 0.3s
    constexpr float probability_true_detection = 0.9;
    constexpr float probability_false_detection = 0.2;

    // update measured channel probability
    existence_probabilities_[channel_index] = updateProbability(
      existence_probabilities_[channel_index], probability_true_detection,
      probability_false_detection);
    // decay other channel probabilities
    for (size_t i = 0; i < existence_probabilities_.size(); ++i) {
      if (i == channel_index) {
        continue;
      }
      existence_probabilities_[i] *= std::exp(decay_rate * delta_time);
    }

    // update total existence probability
    const float & existence_probability_from_object = object.existence_probability;
    total_existence_probability_ = updateProbability(
      total_existence_probability_, existence_probability_from_object, probability_false_detection);
  }

  last_update_with_measurement_time_ = measurement_time;

  // Update object
  measure(object, measurement_time, self_transform);

  return true;
}

bool Tracker::updateWithoutMeasurement(const rclcpp::Time & now)
{
  // Update existence probability
  ++no_measurement_count_;
  ++total_no_measurement_count_;
  {
    // decay existence probability
    double const delta_time = (now - last_update_with_measurement_time_).seconds();
    const double decay_rate = -log(0.5) / 0.3;  // 50% decay in 0.3s
    for (size_t i = 0; i < existence_probabilities_.size(); ++i) {
      existence_probabilities_[i] *= std::exp(-decay_rate * delta_time);
    }
    total_existence_probability_ *= std::exp(-decay_rate * delta_time);
  }

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
