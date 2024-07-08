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

#ifndef AUTOWARE__DETECTED_OBJECT_VALIDATION__UTILS__UTILS_HPP_
#define AUTOWARE__DETECTED_OBJECT_VALIDATION__UTILS__UTILS_HPP_

#include <autoware_perception_msgs/msg/detected_objects.hpp>

#include <cstdint>

namespace autoware::detected_object_validation
{
namespace utils
{
struct FilterTargetLabel
{
  bool UNKNOWN;
  bool CAR;
  bool TRUCK;
  bool BUS;
  bool TRAILER;
  bool MOTORCYCLE;
  bool BICYCLE;
  bool PEDESTRIAN;
  bool isTarget(const uint8_t label) const;
};  // struct FilterTargetLabel

inline bool hasBoundingBox(const autoware_perception_msgs::msg::DetectedObject & object)
{
  if (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return true;
  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    return true;
  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    return false;
  } else {
    // unknown shape type.
    return false;
  }
}

}  // namespace utils
}  // namespace autoware::detected_object_validation

#endif  // AUTOWARE__DETECTED_OBJECT_VALIDATION__UTILS__UTILS_HPP_
