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

#include "object_recognition_utils/conversion.hpp"

namespace object_recognition_utils
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;

DetectedObject toDetectedObject(const TrackedObject & tracked_object)
{
  DetectedObject detected_object;
  detected_object.existence_probability = tracked_object.existence_probability;

  detected_object.classification = tracked_object.classification;

  detected_object.kinematics.pose_with_covariance = tracked_object.kinematics.pose_with_covariance;
  detected_object.kinematics.has_position_covariance = true;
  detected_object.kinematics.orientation_availability =
    tracked_object.kinematics.orientation_availability;
  detected_object.kinematics.twist_with_covariance =
    tracked_object.kinematics.twist_with_covariance;
  detected_object.kinematics.has_twist = true;
  detected_object.kinematics.has_twist_covariance = true;

  detected_object.shape = tracked_object.shape;
  return detected_object;
}

DetectedObjects toDetectedObjects(const TrackedObjects & tracked_objects)
{
  autoware_auto_perception_msgs::msg::DetectedObjects detected_objects;
  detected_objects.header = tracked_objects.header;

  for (auto & tracked_object : tracked_objects.objects) {
    detected_objects.objects.push_back(toDetectedObject(tracked_object));
  }
  return detected_objects;
}

TrackedObject toTrackedObject(const DetectedObject & detected_object)
{
  TrackedObject tracked_object;
  tracked_object.existence_probability = detected_object.existence_probability;

  tracked_object.classification = detected_object.classification;

  tracked_object.kinematics.pose_with_covariance = detected_object.kinematics.pose_with_covariance;
  tracked_object.kinematics.twist_with_covariance =
    detected_object.kinematics.twist_with_covariance;
  tracked_object.kinematics.orientation_availability =
    detected_object.kinematics.orientation_availability;

  tracked_object.shape = detected_object.shape;
  return tracked_object;
}

TrackedObjects toTrackedObjects(const DetectedObjects & detected_objects)
{
  TrackedObjects tracked_objects;
  tracked_objects.header = detected_objects.header;

  for (auto & detected_object : detected_objects.objects) {
    tracked_objects.objects.push_back(toTrackedObject(detected_object));
  }
  return tracked_objects;
}
}  // namespace object_recognition_utils
