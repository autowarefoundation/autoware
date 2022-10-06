// Copyright 2022 TIER IV, Inc.
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

#ifndef PERCEPTION_UTILS__OBJECT_CLASSIFICATION_HPP_
#define PERCEPTION_UTILS__OBJECT_CLASSIFICATION_HPP_

#include "autoware_auto_perception_msgs/msg/object_classification.hpp"

#include <vector>

namespace perception_utils
{
using autoware_auto_perception_msgs::msg::ObjectClassification;

inline std::uint8_t getHighestProbLabel(
  const std::vector<ObjectClassification> & object_classifications)
{
  std::uint8_t label = ObjectClassification::UNKNOWN;
  float highest_prob = 0.0;
  // TODO(Satoshi Tanaka): It might be simple if you use STL or range-v3.
  for (const auto & object_classification : object_classifications) {
    if (highest_prob < object_classification.probability) {
      highest_prob = object_classification.probability;
      label = object_classification.label;
    }
  }
  return label;
}

inline bool isVehicle(const uint8_t object_classification)
{
  return object_classification == ObjectClassification::BICYCLE ||
         object_classification == ObjectClassification::BUS ||
         object_classification == ObjectClassification::CAR ||
         object_classification == ObjectClassification::MOTORCYCLE ||
         object_classification == ObjectClassification::TRAILER ||
         object_classification == ObjectClassification::TRUCK;
}

inline bool isVehicle(const std::vector<ObjectClassification> & object_classifications)
{
  auto highest_prob_classification = getHighestProbLabel(object_classifications);
  return highest_prob_classification == ObjectClassification::BICYCLE ||
         highest_prob_classification == ObjectClassification::BUS ||
         highest_prob_classification == ObjectClassification::CAR ||
         highest_prob_classification == ObjectClassification::MOTORCYCLE ||
         highest_prob_classification == ObjectClassification::TRAILER ||
         highest_prob_classification == ObjectClassification::TRUCK;
}

inline bool isCarLikeVehicle(const uint8_t object_classification)
{
  return object_classification == ObjectClassification::BUS ||
         object_classification == ObjectClassification::CAR ||
         object_classification == ObjectClassification::TRAILER ||
         object_classification == ObjectClassification::TRUCK;
}

inline bool isCarLikeVehicle(const std::vector<ObjectClassification> & object_classifications)
{
  auto highest_prob_classification = getHighestProbLabel(object_classifications);
  return highest_prob_classification == ObjectClassification::BUS ||
         highest_prob_classification == ObjectClassification::CAR ||
         highest_prob_classification == ObjectClassification::TRAILER ||
         highest_prob_classification == ObjectClassification::TRUCK;
}

inline bool isLargeVehicle(const uint8_t object_classification)
{
  return object_classification == ObjectClassification::BUS ||
         object_classification == ObjectClassification::TRAILER ||
         object_classification == ObjectClassification::TRUCK;
}

inline bool isLargeVehicle(const std::vector<ObjectClassification> & object_classifications)
{
  auto highest_prob_classification = getHighestProbLabel(object_classifications);
  return highest_prob_classification == ObjectClassification::BUS ||
         highest_prob_classification == ObjectClassification::TRAILER ||
         highest_prob_classification == ObjectClassification::TRUCK;
}
}  // namespace perception_utils

#endif  // PERCEPTION_UTILS__OBJECT_CLASSIFICATION_HPP_
