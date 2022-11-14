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
#ifndef BEHAVIOR_PATH_PLANNER__DEBUG_UTILITIES_HPP_
#define BEHAVIOR_PATH_PLANNER__DEBUG_UTILITIES_HPP_

#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <string>
#include <vector>

namespace marker_utils
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using behavior_path_planner::ShiftLineArray;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Polygon;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::ColorRGBA;
using tier4_autoware_utils::Polygon2d;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

struct CollisionCheckDebug
{
  std::string failed_reason;
  std::size_t lane_id{0};
  Pose current_pose{};
  Twist current_twist{};
  Twist object_twist{};
  Pose expected_ego_pose{};
  Pose expected_obj_pose{};
  Pose relative_to_ego{};
  bool is_front{false};
  bool allow_lane_change{false};
  std::vector<Pose> lerped_path;
  std::vector<PredictedPath> ego_predicted_path{};
  Polygon2d ego_polygon{};
  Polygon2d obj_polygon{};
};

constexpr std::array<std::array<float, 3>, 10> colorsList()
{
  constexpr std::array<float, 3> red = {1., 0., 0.};
  constexpr std::array<float, 3> green = {0., 1., 0.};
  constexpr std::array<float, 3> blue = {0., 0., 1.};
  constexpr std::array<float, 3> yellow = {1., 1., 0.};
  constexpr std::array<float, 3> aqua = {0., 1., 1.};
  constexpr std::array<float, 3> magenta = {1., 0., 1.};
  constexpr std::array<float, 3> medium_orchid = {0.729, 0.333, 0.827};
  constexpr std::array<float, 3> light_pink = {1, 0.713, 0.756};
  constexpr std::array<float, 3> light_yellow = {1, 1, 0.878};
  constexpr std::array<float, 3> light_steel_blue = {0.690, 0.768, 0.870};
  return {red,     green,         blue,       yellow,       aqua,
          magenta, medium_orchid, light_pink, light_yellow, light_steel_blue};
}

inline int64_t bitShift(int64_t original_id) { return original_id << (sizeof(int32_t) * 8 / 2); }

MarkerArray createPoseMarkerArray(
  const Pose & pose, std::string && ns, const int32_t & id, const float & r, const float & g,
  const float & b);

MarkerArray createPathMarkerArray(
  const PathWithLaneId & path, std::string && ns, const int64_t & lane_id, const float & r,
  const float & g, const float & b);

MarkerArray createShiftLineMarkerArray(
  const ShiftLineArray & shift_lines, const double & base_shift, std::string && ns, const float & r,
  const float & g, const float & b, const float & w);

MarkerArray createShiftLengthMarkerArray(
  const std::vector<double> & shift_distance,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & reference, std::string && ns,
  const float & r, const float & g, const float & b);

MarkerArray createLaneletsAreaMarkerArray(
  const std::vector<lanelet::ConstLanelet> & lanelets, std::string && ns, const float & r,
  const float & g, const float & b);

MarkerArray createFurthestLineStringMarkerArray(const lanelet::ConstLineStrings3d & linestrings);

MarkerArray createPolygonMarkerArray(
  const Polygon & polygon, std::string && ns, const int64_t & lane_id, const float & r,
  const float & g, const float & b);

MarkerArray createObjectsMarkerArray(
  const PredictedObjects & objects, std::string && ns, const int64_t & lane_id, const float & r,
  const float & g, const float & b);

}  // namespace marker_utils

#endif  // BEHAVIOR_PATH_PLANNER__DEBUG_UTILITIES_HPP_
