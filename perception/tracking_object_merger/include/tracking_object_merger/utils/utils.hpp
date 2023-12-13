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
//
//

#ifndef TRACKING_OBJECT_MERGER__UTILS__UTILS_HPP_
#define TRACKING_OBJECT_MERGER__UTILS__UTILS_HPP_

// #include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object_kinematics.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>

#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;
namespace utils
{
enum MSG_COV_IDX {
  X_X = 0,
  X_Y = 1,
  X_Z = 2,
  X_ROLL = 3,
  X_PITCH = 4,
  X_YAW = 5,
  Y_X = 6,
  Y_Y = 7,
  Y_Z = 8,
  Y_ROLL = 9,
  Y_PITCH = 10,
  Y_YAW = 11,
  Z_X = 12,
  Z_Y = 13,
  Z_Z = 14,
  Z_ROLL = 15,
  Z_PITCH = 16,
  Z_YAW = 17,
  ROLL_X = 18,
  ROLL_Y = 19,
  ROLL_Z = 20,
  ROLL_ROLL = 21,
  ROLL_PITCH = 22,
  ROLL_YAW = 23,
  PITCH_X = 24,
  PITCH_Y = 25,
  PITCH_Z = 26,
  PITCH_ROLL = 27,
  PITCH_PITCH = 28,
  PITCH_YAW = 29,
  YAW_X = 30,
  YAW_Y = 31,
  YAW_Z = 32,
  YAW_ROLL = 33,
  YAW_PITCH = 34,
  YAW_YAW = 35
};

// linear interpolation for tracked objects
TrackedObject linearInterpolationForTrackedObject(
  const TrackedObject & obj1, const TrackedObject & obj2);

// predict tracked objects
TrackedObject predictPastOrFutureTrackedObject(const TrackedObject & obj, const double dt);

TrackedObjects predictPastOrFutureTrackedObjects(
  const TrackedObjects & obj, const std_msgs::msg::Header & header);

// predict tracked objects
TrackedObjects interpolateTrackedObjects(
  const TrackedObjects & objects1, const TrackedObjects & objects2, std_msgs::msg::Header header);

}  // namespace utils

namespace merger_utils
{
// merge policy
enum MergePolicy : int { SKIP = 0, OVERWRITE = 1, FUSION = 2 };

// object kinematics velocity merger
autoware_auto_perception_msgs::msg::TrackedObjectKinematics objectKinematicsVXMerger(
  const TrackedObject & main_obj, const TrackedObject & sub_obj, const MergePolicy policy);

// object classification merger
TrackedObject objectClassificationMerger(
  const TrackedObject & main_obj, const TrackedObject & sub_obj, const MergePolicy policy);

// probability merger
float probabilityMerger(const float main_prob, const float sub_prob, const MergePolicy policy);

// shape merger
autoware_auto_perception_msgs::msg::Shape shapeMerger(
  const TrackedObject & main_obj, const TrackedObject & sub_obj, const MergePolicy policy);

// update tracked object
void updateExceptVelocity(TrackedObject & main_obj, const TrackedObject & sub_obj);

void updateOnlyObjectVelocity(TrackedObject & main_obj, const TrackedObject & sub_obj);

void updateOnlyClassification(TrackedObject & main_obj, const TrackedObject & sub_obj);

void updateWholeTrackedObject(TrackedObject & main_obj, const TrackedObject & sub_obj);

}  // namespace merger_utils

#endif  // TRACKING_OBJECT_MERGER__UTILS__UTILS_HPP_
