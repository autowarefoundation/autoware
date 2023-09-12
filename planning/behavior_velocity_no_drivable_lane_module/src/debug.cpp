// Copyright 2023 TIER IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <vector>

namespace behavior_velocity_planner
{
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;
using visualization_msgs::msg::Marker;

namespace
{
visualization_msgs::msg::MarkerArray createNoDrivableLaneMarkers(
  const NoDrivableLaneModule::DebugData & debug_data, const rclcpp::Time & now,
  const int64_t module_id)
{
  visualization_msgs::msg::MarkerArray msg;
  const int32_t uid = planning_utils::bitShift(module_id);

  // No Drivable Lane Polygon
  if (!debug_data.no_drivable_lane_polygon.empty()) {
    auto marker = createDefaultMarker(
      "map", now, "no_drivable_lane polygon", uid, Marker::LINE_STRIP,
      createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));
    for (const auto & p : debug_data.no_drivable_lane_polygon) {
      marker.points.push_back(createPoint(p.x, p.y, p.z));
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Path - polygon intersection points
  {
    auto marker = createDefaultMarker(
      "map", now, "path_polygon intersection points", uid, Marker::POINTS,
      createMarkerScale(0.25, 0.25, 0.0), createMarkerColor(1.0, 0.0, 0.0, 0.999));
    const auto & p_first = debug_data.path_polygon_intersection.first_intersection_point;
    if (p_first) {
      marker.points.push_back(createPoint(p_first->x, p_first->y, p_first->z));
    }
    const auto & p_second = debug_data.path_polygon_intersection.second_intersection_point;
    if (p_second) {
      marker.points.push_back(createPoint(p_second->x, p_second->y, p_second->z));
    }
    if (!marker.points.empty()) msg.markers.push_back(marker);
  }

  return msg;
}
}  // namespace

motion_utils::VirtualWalls NoDrivableLaneModule::createVirtualWalls()
{
  motion_utils::VirtualWalls virtual_walls;

  const auto now = this->clock_->now();

  if (
    (state_ == State::APPROACHING) || (state_ == State::INSIDE_NO_DRIVABLE_LANE) ||
    (state_ == State::STOPPED)) {
    motion_utils::VirtualWall wall;
    wall.text = "no_drivable_lane";
    wall.style = motion_utils::VirtualWallType::stop;
    wall.ns = std::to_string(module_id_) + "_";
    wall.pose = debug_data_.stop_pose;
    virtual_walls.push_back(wall);
  }

  return virtual_walls;
}

visualization_msgs::msg::MarkerArray NoDrivableLaneModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;
  const auto now = this->clock_->now();

  appendMarkerArray(
    createNoDrivableLaneMarkers(debug_data_, this->clock_->now(), module_id_), &debug_marker_array);

  return debug_marker_array;
}

}  // namespace behavior_velocity_planner
