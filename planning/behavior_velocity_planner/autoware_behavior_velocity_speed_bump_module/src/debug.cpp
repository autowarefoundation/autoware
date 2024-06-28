// Copyright 2020 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/speed_bump.hpp>

#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware::motion_utils::createSlowDownVirtualWallMarker;
using autoware::universe_utils::appendMarkerArray;
using autoware::universe_utils::calcOffsetPose;
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerScale;
using autoware::universe_utils::createPoint;
using visualization_msgs::msg::Marker;

namespace
{
visualization_msgs::msg::MarkerArray createSpeedBumpMarkers(
  const SpeedBumpModule::DebugData & debug_data, const rclcpp::Time & now, const int64_t module_id)
{
  visualization_msgs::msg::MarkerArray msg;
  const int32_t uid = planning_utils::bitShift(module_id);

  // Speed bump polygon
  if (!debug_data.speed_bump_polygon.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "speed_bump polygon", uid, Marker::LINE_STRIP, createMarkerScale(0.1, 0.0, 0.0),
      createMarkerColor(0.0, 0.0, 1.0, 0.999));
    for (const auto & p : debug_data.speed_bump_polygon) {
      marker.points.push_back(createPoint(p.x, p.y, p.z));
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Slow start point
  if (!debug_data.slow_start_poses.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "slow start point", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(1.0, 1.0, 0.0, 0.999));
    for (const auto & p : debug_data.slow_start_poses) {
      marker.points.push_back(createPoint(p.position.x, p.position.y, p.position.z));
    }
    msg.markers.push_back(marker);
  }

  // Slow end point
  if (!debug_data.slow_end_points.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "slow end point", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(0.2, 0.8, 1.0, 0.999));
    for (const auto & p : debug_data.slow_end_points) {
      marker.points.push_back(createPoint(p.x, p.y, p.z));
    }
    msg.markers.push_back(marker);
  }

  // Path - polygon intersection points
  {
    auto marker = createDefaultMarker(
      "map", now, "path_polygon intersection points", uid, Marker::POINTS,
      createMarkerScale(0.25, 0.25, 0.0), createMarkerColor(1.0, 0.0, 0.0, 0.999));
    const auto & p_first = debug_data.path_polygon_intersection_status.first_intersection_point;
    if (p_first) {
      marker.points.push_back(createPoint(p_first->x, p_first->y, p_first->z));
    }
    const auto & p_second = debug_data.path_polygon_intersection_status.second_intersection_point;
    if (p_second) {
      marker.points.push_back(createPoint(p_second->x, p_second->y, p_second->z));
    }
    if (!marker.points.empty()) msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace

autoware::motion_utils::VirtualWalls SpeedBumpModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.text = "speed_bump";
  wall.ns = std::to_string(module_id_) + "_";
  wall.style = autoware::motion_utils::VirtualWallType::slowdown;
  for (const auto & p : debug_data_.slow_start_poses) {
    wall.pose = autoware::universe_utils::calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

visualization_msgs::msg::MarkerArray SpeedBumpModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  appendMarkerArray(
    createSpeedBumpMarkers(debug_data_, this->clock_->now(), module_id_), &debug_marker_array);

  return debug_marker_array;
}

}  // namespace autoware::behavior_velocity_planner
