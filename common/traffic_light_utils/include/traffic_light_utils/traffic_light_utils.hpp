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

#ifndef TRAFFIC_LIGHT_UTILS__TRAFFIC_LIGHT_UTILS_HPP_
#define TRAFFIC_LIGHT_UTILS__TRAFFIC_LIGHT_UTILS_HPP_

#include "tier4_perception_msgs/msg/traffic_light_element.hpp"
#include "tier4_perception_msgs/msg/traffic_light_roi.hpp"
#include "tier4_perception_msgs/msg/traffic_signal.hpp"

#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Primitive.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

namespace traffic_light_utils
{

bool isRoiValid(
  const tier4_perception_msgs::msg::TrafficLightRoi & roi, uint32_t width, uint32_t height);

void setRoiInvalid(tier4_perception_msgs::msg::TrafficLightRoi & roi);

bool isSignalUnknown(const tier4_perception_msgs::msg::TrafficSignal & signal);

void setSignalUnknown(tier4_perception_msgs::msg::TrafficSignal & signal, float confidence = -1);

tf2::Vector3 getTrafficLightTopLeft(const lanelet::ConstLineString3d & traffic_light);

tf2::Vector3 getTrafficLightBottomRight(const lanelet::ConstLineString3d & traffic_light);

tf2::Vector3 getTrafficLightCenter(const lanelet::ConstLineString3d & traffic_light);

}  // namespace traffic_light_utils

#endif  // TRAFFIC_LIGHT_UTILS__TRAFFIC_LIGHT_UTILS_HPP_
