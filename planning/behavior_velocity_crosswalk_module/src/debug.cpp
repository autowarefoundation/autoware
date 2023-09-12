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

#include "scene_crosswalk.hpp"

#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_utils/marker/marker_helper.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <vector>

namespace behavior_velocity_planner
{

using motion_utils::createSlowDownVirtualWallMarker;
using motion_utils::createStopVirtualWallMarker;
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;
using visualization_msgs::msg::Marker;

namespace
{
visualization_msgs::msg::MarkerArray createCrosswalkMarkers(
  const DebugData & debug_data, const rclcpp::Time & now, const int64_t module_id)
{
  visualization_msgs::msg::MarkerArray msg;
  int32_t uid = planning_utils::bitShift(module_id);

  {
    for (size_t i = 0; i < debug_data.collision_points.size(); ++i) {
      const auto & collision_point = debug_data.collision_points.at(i);
      const auto & point = std::get<1>(collision_point);
      const auto & state = std::get<2>(collision_point);

      auto marker = createDefaultMarker(
        "map", now, "collision point state", uid + i++, Marker::TEXT_VIEW_FACING,
        createMarkerScale(0.0, 0.0, 1.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));
      std::ostringstream string_stream;
      string_stream << std::fixed << std::setprecision(2);
      string_stream << "(module, ttc, ttv, state)=(" << module_id << " , "
                    << point.time_to_collision << " , " << point.time_to_vehicle;
      switch (state) {
        case CollisionState::YIELD:
          string_stream << " , YIELD)";
          break;
        case CollisionState::EGO_PASS_FIRST:
          string_stream << " , EGO_PASS_FIRST)";
          break;
        case CollisionState::EGO_PASS_LATER:
          string_stream << " , EGO_PASS_LATER)";
          break;
        case CollisionState::IGNORE:
          string_stream << " , IGNORE)";
          break;
        default:
          string_stream << " , NONE)";
          break;
      }
      marker.text = string_stream.str();
      marker.pose.position = point.collision_point;
      marker.pose.position.z += 2.0 + i * 0.5;  // NOTE: so that the texts will not overlap
      msg.markers.push_back(marker);
    }
  }

  if (debug_data.range_near_point) {
    auto marker = createDefaultMarker(
      "map", now, "attention range near", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(0.0, 0.0, 1.0, 0.999));
    marker.points.push_back(*debug_data.range_near_point);
    msg.markers.push_back(marker);
  }

  if (debug_data.range_far_point) {
    auto marker = createDefaultMarker(
      "map", now, "attention range far", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker.points.push_back(*debug_data.range_far_point);
    msg.markers.push_back(marker);
  }

  for (size_t i = 0; i < debug_data.obj_polygons.size(); ++i) {
    const auto & points = debug_data.obj_polygons.at(i);
    auto marker = createDefaultMarker(
      "map", now, "object polygon", uid + i, Marker::LINE_STRIP, createMarkerScale(0.1, 0.0, 0.0),
      createMarkerColor(1.0, 0.0, 1.0, 0.999));
    marker.points = points;
    // marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  for (size_t i = 0; i < debug_data.ego_polygons.size(); ++i) {
    const auto & points = debug_data.ego_polygons.at(i);
    auto marker = createDefaultMarker(
      "map", now, "vehicle polygon", uid + i, Marker::LINE_STRIP, createMarkerScale(0.1, 0.0, 0.0),
      createMarkerColor(1.0, 1.0, 0.0, 0.999));
    marker.points = points;
    // marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Crosswalk polygon
  if (!debug_data.crosswalk_polygon.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "crosswalk polygon", uid, Marker::LINE_STRIP, createMarkerScale(0.1, 0.0, 0.0),
      createMarkerColor(1.0, 1.0, 1.0, 0.999));
    for (const auto & p : debug_data.crosswalk_polygon) {
      marker.points.push_back(createPoint(p.x, p.y, p.z));
    }
    marker.points.push_back(marker.points.front());
    marker.color = debug_data.ignore_crosswalk ? createMarkerColor(1.0, 1.0, 1.0, 0.999)
                                               : createMarkerColor(1.0, 0.0, 0.0, 0.999);
    msg.markers.push_back(marker);
  }

  // Collision point
  if (!debug_data.collision_points.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "collision point", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(1.0, 0.0, 1.0, 0.999));
    for (const auto & collision_point : debug_data.collision_points) {
      marker.points.push_back(std::get<1>(collision_point).collision_point);
    }
    msg.markers.push_back(marker);
  }

  // Slow point
  if (!debug_data.slow_poses.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "slow point", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(1.0, 1.0, 0.0, 0.999));
    for (const auto & p : debug_data.slow_poses) {
      marker.points.push_back(createPoint(p.position.x, p.position.y, p.position.z));
    }
    msg.markers.push_back(marker);
  }

  // Stop factor point
  if (!debug_data.stop_factor_points.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "stop factor point", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(0.0, 0.0, 1.0, 0.999));
    for (const auto & p : debug_data.stop_factor_points) {
      marker.points.push_back(p);
    }
    msg.markers.push_back(marker);
  }

  // Stop point
  if (!debug_data.stop_poses.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "stop point", uid, Marker::POINTS, createMarkerScale(0.25, 0.25, 0.0),
      createMarkerColor(1.0, 0.0, 0.0, 0.999));
    for (const auto & p : debug_data.stop_poses) {
      marker.points.push_back(createPoint(p.position.x, p.position.y, p.position.z));
    }
    msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace

motion_utils::VirtualWalls CrosswalkModule::createVirtualWalls()
{
  motion_utils::VirtualWalls virtual_walls;
  motion_utils::VirtualWall wall;
  wall.text = "crosswalk";
  wall.ns = std::to_string(module_id_) + "_";

  wall.style = motion_utils::VirtualWallType::stop;
  for (const auto & p : debug_data_.stop_poses) {
    wall.pose = calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  wall.style = motion_utils::VirtualWallType::slowdown;
  for (const auto & p : debug_data_.slow_poses) {
    wall.pose = calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

visualization_msgs::msg::MarkerArray CrosswalkModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  appendMarkerArray(
    createCrosswalkMarkers(debug_data_, this->clock_->now(), module_id_), &debug_marker_array);

  return debug_marker_array;
}
}  // namespace behavior_velocity_planner
