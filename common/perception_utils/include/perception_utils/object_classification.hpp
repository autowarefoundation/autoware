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

namespace perception_utils
{
using autoware_auto_perception_msgs::msg::ObjectClassification;

bool isVehicle(const uint8_t object_classification)
{
  return object_classification == ObjectClassification::BICYCLE ||
         object_classification == ObjectClassification::BUS ||
         object_classification == ObjectClassification::CAR ||
         object_classification == ObjectClassification::MOTORCYCLE ||
         object_classification == ObjectClassification::TRAILER ||
         object_classification == ObjectClassification::TRUCK;
}

bool isCarLikeVehicle(const uint8_t object_classification)
{
  return object_classification == ObjectClassification::BUS ||
         object_classification == ObjectClassification::CAR ||
         object_classification == ObjectClassification::TRAILER ||
         object_classification == ObjectClassification::TRUCK;
}

bool isLargeVehicle(const uint8_t object_classification)
{
  return object_classification == ObjectClassification::BUS ||
         object_classification == ObjectClassification::TRAILER ||
         object_classification == ObjectClassification::TRUCK;
}
}  // namespace perception_utils

#endif  // PERCEPTION_UTILS__OBJECT_CLASSIFICATION_HPP_
