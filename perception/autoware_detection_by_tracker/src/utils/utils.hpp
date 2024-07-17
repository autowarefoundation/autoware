// Copyright 2023 TIER IV, Inc.
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

#ifndef UTILS__UTILS_HPP_
#define UTILS__UTILS_HPP_

#include "autoware_perception_msgs/msg/object_classification.hpp"

#include <cstdint>

namespace autoware::detection_by_tracker
{
namespace utils
{
using Label = autoware_perception_msgs::msg::ObjectClassification;

struct TrackerIgnoreLabel
{
  bool UNKNOWN;
  bool CAR;
  bool TRUCK;
  bool BUS;
  bool TRAILER;
  bool MOTORCYCLE;
  bool BICYCLE;
  bool PEDESTRIAN;
  bool isIgnore(const uint8_t label) const
  {
    return (label == Label::UNKNOWN && UNKNOWN) || (label == Label::CAR && CAR) ||
           (label == Label::TRUCK && TRUCK) || (label == Label::BUS && BUS) ||
           (label == Label::TRAILER && TRAILER) || (label == Label::MOTORCYCLE && MOTORCYCLE) ||
           (label == Label::BICYCLE && BICYCLE) || (label == Label::PEDESTRIAN && PEDESTRIAN);
  }
};

}  // namespace utils
}  // namespace autoware::detection_by_tracker

#endif  // UTILS__UTILS_HPP_
