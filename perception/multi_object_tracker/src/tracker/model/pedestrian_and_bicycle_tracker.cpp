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
// Author: v1.0 Yukihiro Saito
//

#include "multi_object_tracker/tracker/model/pedestrian_and_bicycle_tracker.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

PedestrianAndBicycleTracker::PedestrianAndBicycleTracker(
  const rclcpp::Time & time, const autoware_auto_perception_msgs::msg::DetectedObject & object)
: Tracker(time, object.classification),
  pedestrian_tracker_(time, object),
  bicycle_tracker_(time, object)
{
}

bool PedestrianAndBicycleTracker::predict(const rclcpp::Time & time)
{
  pedestrian_tracker_.predict(time);
  bicycle_tracker_.predict(time);
  return true;
}

bool PedestrianAndBicycleTracker::measure(
  const autoware_auto_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time)
{
  pedestrian_tracker_.measure(object, time);
  bicycle_tracker_.measure(object, time);
  if (perception_utils::getHighestProbLabel(object.classification) != Label::UNKNOWN)
    setClassification(object.classification);
  return true;
}

bool PedestrianAndBicycleTracker::getTrackedObject(
  const rclcpp::Time & time, autoware_auto_perception_msgs::msg::TrackedObject & object) const
{
  using Label = autoware_auto_perception_msgs::msg::ObjectClassification;
  const uint8_t label = getHighestProbLabel();

  if (label == Label::PEDESTRIAN) {
    pedestrian_tracker_.getTrackedObject(time, object);
  } else if (label == Label::BICYCLE || label == Label::MOTORCYCLE) {
    bicycle_tracker_.getTrackedObject(time, object);
  }
  object.object_id = getUUID();
  object.classification = getClassification();
  return true;
}
