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

inline ObjectClassification getHighestProbClassification(
  const std::vector<ObjectClassification> & object_classifications)
{
  if (object_classifications.empty()) {
    return ObjectClassification{};
  }
  return *std::max_element(
    std::begin(object_classifications), std::end(object_classifications),
    [](const auto & a, const auto & b) { return a.probability < b.probability; });
}

inline std::uint8_t getHighestProbLabel(
  const std::vector<ObjectClassification> & object_classifications)
{
  auto classification = getHighestProbClassification(object_classifications);
  return classification.label;
}

inline bool isVehicle(const uint8_t label)
{
  return label == ObjectClassification::BICYCLE || label == ObjectClassification::BUS ||
         label == ObjectClassification::CAR || label == ObjectClassification::MOTORCYCLE ||
         label == ObjectClassification::TRAILER || label == ObjectClassification::TRUCK;
}

inline bool isVehicle(const ObjectClassification & object_classification)
{
  return isVehicle(object_classification.label);
}

inline bool isVehicle(const std::vector<ObjectClassification> & object_classifications)
{
  auto highest_prob_label = getHighestProbLabel(object_classifications);
  return isVehicle(highest_prob_label);
}

inline bool isCarLikeVehicle(const uint8_t label)
{
  return label == ObjectClassification::BUS || label == ObjectClassification::CAR ||
         label == ObjectClassification::TRAILER || label == ObjectClassification::TRUCK;
}

inline bool isCarLikeVehicle(const ObjectClassification & object_classification)
{
  return isCarLikeVehicle(object_classification.label);
}

inline bool isCarLikeVehicle(const std::vector<ObjectClassification> & object_classifications)
{
  auto highest_prob_label = getHighestProbLabel(object_classifications);
  return isCarLikeVehicle(highest_prob_label);
}

inline bool isLargeVehicle(const uint8_t label)
{
  return label == ObjectClassification::BUS || label == ObjectClassification::TRAILER ||
         label == ObjectClassification::TRUCK;
}

inline bool isLargeVehicle(const ObjectClassification & object_classification)
{
  return isLargeVehicle(object_classification.label);
}

inline bool isLargeVehicle(const std::vector<ObjectClassification> & object_classifications)
{
  auto highest_prob_label = getHighestProbLabel(object_classifications);
  return isLargeVehicle(highest_prob_label);
}
}  // namespace perception_utils

#endif  // PERCEPTION_UTILS__OBJECT_CLASSIFICATION_HPP_
