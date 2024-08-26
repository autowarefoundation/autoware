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

#include "autoware_perception_msgs/msg/traffic_light_element.hpp"
#include "autoware_perception_msgs/msg/traffic_light_group.hpp"
#include "tier4_perception_msgs/msg/traffic_light.hpp"
#include "tier4_perception_msgs/msg/traffic_light_element.hpp"
#include "tier4_perception_msgs/msg/traffic_light_roi.hpp"

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Primitive.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

namespace traffic_light_utils
{

void setSignalUnknown(tier4_perception_msgs::msg::TrafficLight & signal, float confidence = -1);

/**
 * @brief Checks if a traffic light state includes a circle-shaped light with the specified color.
 *
 * Iterates through the traffic light elements to find a circle-shaped light that matches the given
 * color.
 *
 * @param tl_state The traffic light state to check.
 * @param lamp_color The color to look for in the traffic light's circle-shaped lamps.
 * @return True if a circle-shaped light with the specified color is found, false otherwise.
 */
bool hasTrafficLightCircleColor(
  const autoware_perception_msgs::msg::TrafficLightGroup & tl_state, const uint8_t & lamp_color);

/**
 * @brief Checks if a traffic light state includes a light with the specified shape.
 *
 * Searches through the traffic light elements to find a light that matches the given shape.
 *
 * @param tl_state The traffic light state to check.
 * @param shape The shape to look for in the traffic light's lights.
 * @return True if a light with the specified shape is found, false otherwise.
 */
bool hasTrafficLightShape(
  const autoware_perception_msgs::msg::TrafficLightGroup & tl_state, const uint8_t & lamp_shape);

/**
 * @brief Determines if a traffic signal indicates a stop for the given lanelet.
 *
 * Evaluates the current state of the traffic light, considering if it's green or unknown,
 * which would not necessitate a stop. Then, it checks the turn direction attribute of the lanelet
 * against the traffic light's arrow shapes to determine whether a vehicle must stop or if it can
 * proceed based on allowed turn directions.
 *
 * @param lanelet The lanelet to check for a stop signal at its traffic light.
 * @param tl_state The current state of the traffic light associated with the lanelet.
 * @return True if the traffic signal indicates a stop is required, false otherwise.
 */
bool isTrafficSignalStop(
  const lanelet::ConstLanelet & lanelet,
  const autoware_perception_msgs::msg::TrafficLightGroup & tl_state);

tf2::Vector3 getTrafficLightTopLeft(const lanelet::ConstLineString3d & traffic_light);

tf2::Vector3 getTrafficLightBottomRight(const lanelet::ConstLineString3d & traffic_light);

tf2::Vector3 getTrafficLightCenter(const lanelet::ConstLineString3d & traffic_light);

}  // namespace traffic_light_utils

#endif  // TRAFFIC_LIGHT_UTILS__TRAFFIC_LIGHT_UTILS_HPP_
