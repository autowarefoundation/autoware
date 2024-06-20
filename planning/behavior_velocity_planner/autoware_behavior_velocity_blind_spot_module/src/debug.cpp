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

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <string>

namespace autoware::behavior_velocity_planner
{
using autoware::universe_utils::appendMarkerArray;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerOrientation;
using autoware::universe_utils::createMarkerScale;

namespace
{

visualization_msgs::msg::MarkerArray createLaneletPolygonsMarkerArray(
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  int32_t i = 0;
  int32_t uid = planning_utils::bitShift(lane_id);
  for (const auto & polygon : polygons) {
    visualization_msgs::msg::Marker marker{};
    marker.header.frame_id = "map";

    marker.ns = ns;
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = createMarkerScale(0.1, 0.0, 0.0);
    marker.color = createMarkerColor(r, g, b, 0.999);
    for (const auto & p : polygon) {
      geometry_msgs::msg::Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

}  // namespace

autoware::motion_utils::VirtualWalls BlindSpotModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;

  if (debug_data_.virtual_wall_pose) {
    autoware::motion_utils::VirtualWall wall;
    wall.text = "blind_spot";
    wall.pose = debug_data_.virtual_wall_pose.value();
    wall.ns = std::to_string(module_id_) + "_";
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

visualization_msgs::msg::MarkerArray BlindSpotModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  const auto now = this->clock_->now();

  if (debug_data_.detection_area) {
    appendMarkerArray(
      createLaneletPolygonsMarkerArray(
        {debug_data_.detection_area.value()}, "detection_area", module_id_, 0.5, 0.0, 0.0),
      &debug_marker_array, now);
  }

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.conflicting_targets, "conflicting_targets", module_id_, now, 0.99, 0.4, 0.0),
    &debug_marker_array, now);

  return debug_marker_array;
}
}  // namespace autoware::behavior_velocity_planner
