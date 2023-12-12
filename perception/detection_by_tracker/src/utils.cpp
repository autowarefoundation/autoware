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

#include "detection_by_tracker/utils.hpp"

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>

namespace detection_by_tracker
{
namespace utils
{
using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

bool TrackerIgnoreLabel::isIgnore(const uint8_t label) const
{
  return (label == Label::UNKNOWN && UNKNOWN) || (label == Label::CAR && CAR) ||
         (label == Label::TRUCK && TRUCK) || (label == Label::BUS && BUS) ||
         (label == Label::TRAILER && TRAILER) || (label == Label::MOTORCYCLE && MOTORCYCLE) ||
         (label == Label::BICYCLE && BICYCLE) || (label == Label::PEDESTRIAN && PEDESTRIAN);
}
}  // namespace utils
}  // namespace detection_by_tracker
