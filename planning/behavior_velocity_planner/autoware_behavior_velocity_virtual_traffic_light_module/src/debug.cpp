// Copyright 2021 Tier IV, Inc.
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

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/universe_utils/math/constants.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>
using autoware::motion_utils::createStopVirtualWallMarker;
using autoware::universe_utils::appendMarkerArray;
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerOrientation;
using autoware::universe_utils::createMarkerPosition;
using autoware::universe_utils::createMarkerScale;
using autoware::universe_utils::toMsg;
using namespace std::literals::string_literals;

namespace autoware::behavior_velocity_planner
{

autoware::motion_utils::VirtualWalls VirtualTrafficLightModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.text = "virtual_traffic_light";
  wall.ns = std::to_string(module_id_) + "_";
  wall.style = autoware::motion_utils::VirtualWallType::stop;
  const auto & d = module_data_;
  // virtual_wall_stop_line
  std::vector<geometry_msgs::msg::Pose> wall_poses;
  if (d.stop_head_pose_at_stop_line) wall_poses.push_back(*d.stop_head_pose_at_stop_line);
  // virtual_wall_end_line
  if (d.stop_head_pose_at_end_line) wall_poses.push_back(*d.stop_head_pose_at_end_line);
  for (const auto & p : wall_poses) {
    wall.pose = p;
    virtual_walls.push_back(wall);
  }

  return virtual_walls;
}

visualization_msgs::msg::MarkerArray VirtualTrafficLightModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  // Common
  const auto & m = map_data_;
  const auto now = clock_->now();

  // instrument_id
  {
    auto marker = createDefaultMarker(
      "map", now, "instrument_id", module_id_, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.0, 0.0, 1.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));

    marker.pose.position = toMsg(m.instrument_center);
    marker.text = m.instrument_id;

    debug_marker_array.markers.push_back(marker);
  }

  // instrument_center
  {
    auto marker = createDefaultMarker(
      "map", now, "instrument_center", module_id_, visualization_msgs::msg::Marker::SPHERE,
      createMarkerScale(0.3, 0.3, 0.3), createMarkerColor(1.0, 0.0, 0.0, 0.999));

    marker.pose.position = toMsg(m.instrument_center);

    debug_marker_array.markers.push_back(marker);
  }

  // stop_line
  if (m.stop_line) {
    auto marker = createDefaultMarker(
      "map", now, "stop_line", module_id_, visualization_msgs::msg::Marker::LINE_STRIP,
      createMarkerScale(0.3, 0.0, 0.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));

    for (const auto & p : *m.stop_line) {
      marker.points.push_back(toMsg(p));
    }

    debug_marker_array.markers.push_back(marker);
  }

  // start_line
  {
    auto marker = createDefaultMarker(
      "map", now, "start_line", module_id_, visualization_msgs::msg::Marker::LINE_STRIP,
      createMarkerScale(0.3, 0.0, 0.0), createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (const auto & p : m.start_line) {
      marker.points.push_back(toMsg(p));
    }

    debug_marker_array.markers.push_back(marker);
  }

  // end_lines
  {
    auto marker = createDefaultMarker(
      "map", now, "end_lines", module_id_, visualization_msgs::msg::Marker::LINE_LIST,
      createMarkerScale(0.3, 0.0, 0.0), createMarkerColor(0.0, 1.0, 1.0, 0.999));

    for (const auto & line : m.end_lines) {
      for (size_t i = 1; i < line.size(); ++i) {
        marker.points.push_back(toMsg(line.at(i - 1)));
        marker.points.push_back(toMsg(line.at(i)));
      }
    }

    debug_marker_array.markers.push_back(marker);
  }

  return debug_marker_array;
}
}  // namespace autoware::behavior_velocity_planner
