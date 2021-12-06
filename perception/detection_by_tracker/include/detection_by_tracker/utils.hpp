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

#ifndef DETECTION_BY_TRACKER__UTILS_HPP_
#define DETECTION_BY_TRACKER__UTILS_HPP_

#include <autoware_utils/autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <cmath>

namespace utils
{
double getPolygonArea(const geometry_msgs::msg::Polygon & footprint);
double getRectangleArea(const geometry_msgs::msg::Vector3 & dimensions);
double getCircleArea(const geometry_msgs::msg::Vector3 & dimensions);
double getArea(const autoware_auto_perception_msgs::msg::Shape & shape);
double get2dIoU(
  const autoware_auto_perception_msgs::msg::DetectedObject & object1,
  const autoware_auto_perception_msgs::msg::DetectedObject & object2);
double get2dPrecision(
  const autoware_auto_perception_msgs::msg::DetectedObject & source_object,
  const autoware_auto_perception_msgs::msg::DetectedObject & target_object);
double get2dRecall(
  const autoware_auto_perception_msgs::msg::DetectedObject & source_object,
  const autoware_auto_perception_msgs::msg::DetectedObject & target_object);
}  // namespace utils

#endif  // DETECTION_BY_TRACKER__UTILS_HPP_
