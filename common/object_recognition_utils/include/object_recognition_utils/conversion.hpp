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

#ifndef OBJECT_RECOGNITION_UTILS__CONVERSION_HPP_
#define OBJECT_RECOGNITION_UTILS__CONVERSION_HPP_

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>

namespace object_recognition_utils
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;

DetectedObject toDetectedObject(const TrackedObject & tracked_object);
DetectedObjects toDetectedObjects(const TrackedObjects & tracked_objects);
TrackedObject toTrackedObject(const DetectedObject & detected_object);
TrackedObjects toTrackedObjects(const DetectedObjects & detected_objects);
}  // namespace object_recognition_utils

#endif  // OBJECT_RECOGNITION_UTILS__CONVERSION_HPP_
