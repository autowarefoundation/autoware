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

#include "scene_walkway.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>

#include <vector>

namespace autoware::behavior_velocity_planner
{

using autoware::motion_utils::createSlowDownVirtualWallMarker;
using autoware::motion_utils::createStopVirtualWallMarker;
using autoware::universe_utils::appendMarkerArray;
using autoware::universe_utils::calcOffsetPose;
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerScale;
using autoware::universe_utils::createPoint;
using visualization_msgs::msg::Marker;

namespace
{
visualization_msgs::msg::MarkerArray createWalkwayMarkers(
  const DebugData & debug_data, const rclcpp::Time & now, const int64_t module_id)
{
  int32_t uid = planning_utils::bitShift(module_id);
  visualization_msgs::msg::MarkerArray msg;

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

  {
    size_t i = 0;
    for (const auto & p : debug_data.stop_poses) {
      auto marker = createDefaultMarker(
        "map", now, "walkway stop judge range", uid + i++, Marker::LINE_STRIP,
        createMarkerScale(0.1, 0.1, 0.0), createMarkerColor(1.0, 0.0, 0.0, 0.5));
      for (size_t j = 0; j < 50; ++j) {
        const auto x = p.position.x + debug_data.stop_judge_range * std::cos(M_PI * 2 / 50 * j);
        const auto y = p.position.y + debug_data.stop_judge_range * std::sin(M_PI * 2 / 50 * j);
        marker.points.push_back(createPoint(x, y, p.position.z));
      }
      marker.points.push_back(marker.points.front());
      msg.markers.push_back(marker);
    }
  }

  return msg;
}
}  // namespace

autoware::motion_utils::VirtualWalls WalkwayModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.text = "walkway";
  wall.ns = std::to_string(module_id_) + "_";

  wall.style = autoware::motion_utils::VirtualWallType::stop;
  for (const auto & p : debug_data_.stop_poses) {
    wall.pose = calcOffsetPose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

visualization_msgs::msg::MarkerArray WalkwayModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  appendMarkerArray(
    createWalkwayMarkers(debug_data_, this->clock_->now(), module_id_), &debug_marker_array);

  return debug_marker_array;
}
}  // namespace autoware::behavior_velocity_planner
