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

#ifndef PERCEPTION_ONLINE_EVALUATOR__UTILS__MARKER_UTILS_HPP_
#define PERCEPTION_ONLINE_EVALUATOR__UTILS__MARKER_UTILS_HPP_

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/Forward.h>

#include <cstdint>
#include <string>
#include <vector>

namespace marker_utils
{

using autoware::universe_utils::Polygon2d;
using autoware_perception_msgs::msg::PredictedObject;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

inline int64_t bitShift(int64_t original_id)
{
  return original_id << (sizeof(int32_t) * 8 / 2);
}

void addFootprintMarker(
  visualization_msgs::msg::Marker & marker, const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

MarkerArray createFootprintMarkerArray(
  const Pose & base_link_pose, const autoware::vehicle_info_utils::VehicleInfo vehicle_info,
  const std::string && ns, const int32_t & id, const float & r, const float & g, const float & b);

MarkerArray createPointsMarkerArray(
  const std::vector<Point> & points, const std::string & ns, const int32_t id, const float r,
  const float g, const float b);
MarkerArray createPointsMarkerArray(
  const std::vector<Pose> & poses, const std::string & ns, const int32_t id, const float r,
  const float g, const float b);

MarkerArray createPoseMarkerArray(
  const Pose & pose, std::string && ns, const int32_t & id, const float & r, const float & g,
  const float & b);

MarkerArray createPosesMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const int32_t & first_id, const float & r,
  const float & g, const float & b, const float & x_scale = 0.5, const float & y_scale = 0.2,
  const float & z_scale = 0.2);

std_msgs::msg::ColorRGBA createColorFromString(const std::string & str);

MarkerArray createObjectPolygonMarkerArray(
  const PredictedObject & object, std::string && ns, const int32_t & id, const float & r,
  const float & g, const float & b);

MarkerArray createDeviationLines(
  const std::vector<Pose> & poses1, const std::vector<Pose> & poses2, const std::string & ns,
  const int32_t & first_id, const float r, const float g, const float b);

}  // namespace marker_utils

#endif  // PERCEPTION_ONLINE_EVALUATOR__UTILS__MARKER_UTILS_HPP_
