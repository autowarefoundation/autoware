// Copyright 2019 Autoware Foundation
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

#ifndef LANELET2_PLUGINS__UTILITY_FUNCTIONS_HPP_
#define LANELET2_PLUGINS__UTILITY_FUNCTIONS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LaneletSequence.h>

#include <string>
#include <unordered_set>
#include <vector>

template <typename T>
bool exists(const std::vector<T> & vectors, const T & item)
{
  for (const auto & i : vectors) {
    if (i == item) {
      return true;
    }
  }
  return false;
}

tier4_autoware_utils::Polygon2d convert_linear_ring_to_polygon(
  tier4_autoware_utils::LinearRing2d footprint);
void insert_marker_array(
  visualization_msgs::msg::MarkerArray * a1, const visualization_msgs::msg::MarkerArray & a2);

/**
 * @brief create a single merged lanelet whose left/right bounds consist of the leftmost/rightmost
 * bound of the original lanelets or the left/right bound of its adjacent road_shoulder respectively
 * @param lanelets topologically sorted sequence of lanelets
 * @param shoulder_lanelets list of possible road_shoulder lanelets to combine with lanelets
 */
lanelet::ConstLanelet combine_lanelets_with_shoulder(
  const lanelet::ConstLanelets & lanelets, const lanelet::ConstLanelets & shoulder_lanelets);

std::vector<geometry_msgs::msg::Point> convertCenterlineToPoints(const lanelet::Lanelet & lanelet);
geometry_msgs::msg::Pose convertBasicPoint3dToPose(
  const lanelet::BasicPoint3d & point, const double lane_yaw);
#endif  // LANELET2_PLUGINS__UTILITY_FUNCTIONS_HPP_
