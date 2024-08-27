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

#ifndef AUTOWARE__TRACKING_OBJECT_MERGER__UTILS__UTILS_HPP_
#define AUTOWARE__TRACKING_OBJECT_MERGER__UTILS__UTILS_HPP_

#include "autoware/universe_utils/geometry/geometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/object_classification.hpp"
#include "autoware_perception_msgs/msg/shape.hpp"
#include "autoware_perception_msgs/msg/tracked_object.hpp"
#include "autoware_perception_msgs/msg/tracked_object_kinematics.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"
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

namespace autoware::tracking_object_merger
{
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

namespace utils
{
// linear interpolation for tracked objects
TrackedObject linearInterpolationForTrackedObject(
  const TrackedObject & obj1, const TrackedObject & obj2);

// predict tracked objects
TrackedObject predictPastOrFutureTrackedObject(const TrackedObject & obj, const double dt);

TrackedObjects predictPastOrFutureTrackedObjects(
  const TrackedObjects & input_objects, const std_msgs::msg::Header & header);

// predict tracked objects
TrackedObjects interpolateTrackedObjects(
  const TrackedObjects & objects1, const TrackedObjects & objects2, std_msgs::msg::Header header);

}  // namespace utils

namespace merger_utils
{
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

// merge policy
enum MergePolicy : int { SKIP = 0, OVERWRITE = 1, FUSION = 2 };

// object kinematics velocity merger
autoware_perception_msgs::msg::TrackedObjectKinematics objectKinematicsVXMerger(
  const TrackedObject & main_obj, const TrackedObject & sub_obj, const MergePolicy policy);

// object classification merger
TrackedObject objectClassificationMerger(
  const TrackedObject & main_obj, const TrackedObject & sub_obj, const MergePolicy policy);

// update tracked object
void updateExceptVelocity(TrackedObject & main_obj, const TrackedObject & sub_obj);

void updateOnlyObjectVelocity(TrackedObject & main_obj, const TrackedObject & sub_obj);

void updateOnlyClassification(TrackedObject & main_obj, const TrackedObject & sub_obj);

void updateWholeTrackedObject(TrackedObject & main_obj, const TrackedObject & sub_obj);

}  // namespace merger_utils

}  // namespace autoware::tracking_object_merger

#endif  // AUTOWARE__TRACKING_OBJECT_MERGER__UTILS__UTILS_HPP_
