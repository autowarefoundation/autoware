// Copyright 2024 TIER IV, Inc.
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

#ifndef LIDAR_TRANSFUSION__ROS_UTILS_HPP_
#define LIDAR_TRANSFUSION__ROS_UTILS_HPP_

#include "lidar_transfusion/utils.hpp"

#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <string>
#include <vector>

namespace lidar_transfusion
{

void box3DToDetectedObject(
  const Box3D & box3d, const std::vector<std::string> & class_names,
  autoware_perception_msgs::msg::DetectedObject & obj);

uint8_t getSemanticType(const std::string & class_name);

}  // namespace lidar_transfusion

#endif  // LIDAR_TRANSFUSION__ROS_UTILS_HPP_
