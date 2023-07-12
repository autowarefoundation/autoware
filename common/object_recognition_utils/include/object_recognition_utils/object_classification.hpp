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

#ifndef OBJECT_RECOGNITION_UTILS__OBJECT_CLASSIFICATION_HPP_
#define OBJECT_RECOGNITION_UTILS__OBJECT_CLASSIFICATION_HPP_

#include "autoware_auto_perception_msgs/msg/object_classification.hpp"

#include <string>
#include <vector>

namespace object_recognition_utils
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

inline uint8_t toLabel(const std::string & class_name)
{
  if (class_name == "UNKNOWN") {
    return ObjectClassification::UNKNOWN;
  } else if (class_name == "CAR") {
    return ObjectClassification::CAR;
  } else if (class_name == "TRUCK") {
    return ObjectClassification::TRUCK;
  } else if (class_name == "BUS") {
    return ObjectClassification::BUS;
  } else if (class_name == "TRAILER") {
    return ObjectClassification::TRAILER;
  } else if (class_name == "MOTORCYCLE") {
    return ObjectClassification::MOTORCYCLE;
  } else if (class_name == "BICYCLE") {
    return ObjectClassification::BICYCLE;
  } else if (class_name == "PEDESTRIAN") {
    return ObjectClassification::PEDESTRIAN;
  } else {
    throw std::runtime_error("Invalid Classification label.");
  }
}

inline ObjectClassification toObjectClassification(
  const std::string & class_name, float probability)
{
  ObjectClassification classification;
  classification.label = toLabel(class_name);
  classification.probability = probability;
  return classification;
}

inline std::vector<ObjectClassification> toObjectClassifications(
  const std::string & class_name, float probability)
{
  std::vector<ObjectClassification> classifications;
  classifications.push_back(toObjectClassification(class_name, probability));
  return classifications;
}

inline std::string convertLabelToString(const uint8_t label)
{
  if (label == ObjectClassification::UNKNOWN) {
    return "UNKNOWN";
  } else if (label == ObjectClassification::CAR) {
    return "CAR";
  } else if (label == ObjectClassification::TRUCK) {
    return "TRUCK";
  } else if (label == ObjectClassification::BUS) {
    return "BUS";
  } else if (label == ObjectClassification::TRAILER) {
    return "TRAILER";
  } else if (label == ObjectClassification::MOTORCYCLE) {
    return "MOTORCYCLE";
  } else if (label == ObjectClassification::BICYCLE) {
    return "BICYCLE";
  } else if (label == ObjectClassification::PEDESTRIAN) {
    return "PEDESTRIAN";
  } else {
    return "UNKNOWN";
  }
}

inline std::string convertLabelToString(const ObjectClassification object_classification)
{
  return convertLabelToString(object_classification.label);
}

inline std::string convertLabelToString(
  const std::vector<ObjectClassification> object_classifications)
{
  auto highest_prob_label = getHighestProbLabel(object_classifications);
  return convertLabelToString(highest_prob_label);
}

}  // namespace object_recognition_utils

#endif  // OBJECT_RECOGNITION_UTILS__OBJECT_CLASSIFICATION_HPP_
