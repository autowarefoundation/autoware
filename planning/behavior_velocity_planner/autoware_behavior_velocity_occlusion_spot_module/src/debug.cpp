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

#include "occlusion_spot_utils.hpp"
#include "scene_occlusion_spot.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace
{
using builtin_interfaces::msg::Time;
using BasicPolygons = std::vector<lanelet::BasicPolygon2d>;
using autoware::universe_utils::appendMarkerArray;
using autoware::universe_utils::calcOffsetPose;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerOrientation;
using autoware::universe_utils::createMarkerPosition;
using autoware::universe_utils::createMarkerScale;
using occlusion_spot_utils::PossibleCollisionInfo;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

std::vector<Marker> makeDebugInfoMarker(
  const PossibleCollisionInfo & possible_collision, const int id, const bool show_text)
{
  std::vector<Marker> debug_markers;
  Marker debug_marker;
  debug_marker.header.frame_id = "map";
  debug_marker.id = id;
  debug_marker.action = Marker::ADD;
  debug_marker.pose.position = createMarkerPosition(0.0, 0.0, 0.0);
  debug_marker.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
  debug_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  const auto & pc = possible_collision;
  // for collision point with margin
  {
    debug_marker.ns = "collision_point";
    debug_marker.type = Marker::CYLINDER;
    debug_marker.pose = pc.collision_with_margin.pose;
    debug_marker.scale = createMarkerScale(0.5, 0.5, 0.5);
    debug_marker.color = createMarkerColor(1.0, 0.0, 0.0, 0.5);
    debug_markers.push_back(debug_marker);
  }
  // cylinder at collision_point point
  {
    debug_marker.ns = "collision_point_with_margin";
    debug_marker.type = Marker::CYLINDER;
    debug_marker.pose = pc.collision_pose;
    debug_marker.scale = createMarkerScale(0.5, 0.5, 0.5);
    debug_marker.color = createMarkerColor(1.0, 0.5, 0.0, 0.5);
    debug_markers.push_back(debug_marker);
  }

  // cylinder at obstacle point
  {
    debug_marker.ns = "obstacle";
    debug_marker.type = Marker::CYLINDER;
    debug_marker.pose.position = pc.obstacle_info.position;
    debug_marker.color = createMarkerColor(0.5, 0.5, 0.5, 0.5);
    debug_marker.scale = createMarkerScale(0.8, 0.8, 1.5);
    debug_markers.push_back(debug_marker);
  }

  // arrow marker
  {
    debug_marker.ns = "from_obj_to_collision";
    debug_marker.type = Marker::ARROW;
    debug_marker.scale = createMarkerScale(0.05, 0.2, 0.5);
    debug_marker.color = createMarkerColor(0.1, 0.1, 0.1, 0.5);
    debug_marker.points = {pc.obstacle_info.position, pc.intersection_pose.position};
    debug_markers.push_back(debug_marker);
  }

  if (show_text) {
    // info text at obstacle point
    debug_marker.ns = "info";
    debug_marker.type = Marker::TEXT_VIEW_FACING;
    debug_marker.pose = pc.collision_with_margin.pose;
    debug_marker.scale.z = 1.0;
    debug_marker.color = createMarkerColor(1.0, 1.0, 0.0, 1.0);
    std::ostringstream string_stream;
    auto r = [](const double v) { return std::round(v * 100.0) / 100.0; };
    const double len = r(pc.arc_lane_dist_at_collision.length);
    const double dist = r(pc.arc_lane_dist_at_collision.distance);
    const double vel = r(pc.obstacle_info.safe_motion.safe_velocity);
    const double margin = r(pc.obstacle_info.safe_motion.stop_dist);
    string_stream << "(s,d,v,m)=(" << len << " , " << dist << " , " << vel << " , " << margin
                  << " )";
    debug_marker.text = string_stream.str();
    debug_markers.push_back(debug_marker);
  }
  return debug_markers;
}

template <class T>
MarkerArray makeDebugInfoMarkers(T & debug_data)
{
  // add slow down markers for occlusion spot
  MarkerArray debug_markers;
  const auto & possible_collisions = debug_data.possible_collisions;
  size_t id = 0;
  // draw obstacle collision
  for (const auto & pc : possible_collisions) {
    // debug marker
    std::vector<Marker> collision_markers = makeDebugInfoMarker(pc, id, true);
    debug_markers.markers.insert(
      debug_markers.markers.end(), collision_markers.begin(), collision_markers.end());
    id++;
  }
  return debug_markers;
}

MarkerArray makePolygonMarker(
  const BasicPolygons & polygons, const std::string & ns, const int id, const double z)
{
  MarkerArray debug_markers;
  Marker debug_marker;
  debug_marker.header.frame_id = "map";
  debug_marker.header.stamp = rclcpp::Time(0);
  debug_marker.id = planning_utils::bitShift(id);
  debug_marker.type = Marker::LINE_STRIP;
  debug_marker.action = Marker::ADD;
  debug_marker.pose.position = createMarkerPosition(0.0, 0.0, 0);
  debug_marker.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
  debug_marker.scale = createMarkerScale(0.1, 0.1, 0.1);
  debug_marker.color = createMarkerColor(1.0, 1.0, 1.0, 0.5);
  debug_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  debug_marker.ns = ns;
  for (const auto & poly : polygons) {
    for (const auto & p : poly) {
      geometry_msgs::msg::Point point = createMarkerPosition(p.x(), p.y(), z + 0.5);
      debug_marker.points.push_back(point);
    }
    debug_markers.markers.push_back(debug_marker);
    debug_marker.id++;
    debug_marker.points.clear();
  }
  return debug_markers;
}

MarkerArray makeSlicePolygonMarker(
  const Polygons2d & slices, const std::string & ns, const int id, const double z)
{
  MarkerArray debug_markers;
  Marker debug_marker;
  debug_marker.header.frame_id = "map";
  debug_marker.header.stamp = rclcpp::Time(0);
  debug_marker.id = planning_utils::bitShift(id);
  debug_marker.type = Marker::LINE_STRIP;
  debug_marker.action = Marker::ADD;
  debug_marker.pose.position = createMarkerPosition(0.0, 0.0, 0);
  debug_marker.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
  debug_marker.scale = createMarkerScale(0.1, 0.1, 0.1);
  debug_marker.color = createMarkerColor(1.0, 0.0, 1.0, 0.3);
  debug_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
  debug_marker.ns = ns;
  for (const auto & slice : slices) {
    for (const auto & p : slice.outer()) {
      geometry_msgs::msg::Point point = createMarkerPosition(p.x(), p.y(), z);
      debug_marker.points.push_back(point);
    }
    debug_markers.markers.push_back(debug_marker);
    debug_marker.id++;
    debug_marker.points.clear();
  }
  return debug_markers;
}
}  // namespace

MarkerArray OcclusionSpotModule::createDebugMarkerArray()
{
  const auto now = this->clock_->now();
  MarkerArray debug_marker_array;
  if (!debug_data_.possible_collisions.empty()) {
    appendMarkerArray(makeDebugInfoMarkers(debug_data_), &debug_marker_array, now);
  }
  if (!debug_data_.detection_area_polygons.empty()) {
    appendMarkerArray(
      makeSlicePolygonMarker(
        debug_data_.detection_area_polygons, "detection_area", module_id_, debug_data_.z),
      &debug_marker_array, now);
  }
  if (!debug_data_.close_partition.empty() && param_.is_show_occlusion) {
    appendMarkerArray(
      makePolygonMarker(debug_data_.close_partition, "close_partition", module_id_, debug_data_.z),
      &debug_marker_array, now);
  }
  if (!debug_data_.occlusion_points.empty()) {
    appendMarkerArray(
      debug::createPointsMarkerArray(
        debug_data_.occlusion_points, "occlusion", module_id_, now, 0.5, 0.5, 0.5, 1.0, 0.0, 0.0),
      &debug_marker_array, now);
  }
  return debug_marker_array;
}

autoware::motion_utils::VirtualWalls OcclusionSpotModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.text = "occlusion_spot";
  wall.style = autoware::motion_utils::VirtualWallType::slowdown;
  for (size_t id = 0; id < debug_data_.debug_poses.size(); id++) {
    wall.pose =
      calcOffsetPose(debug_data_.debug_poses.at(id), debug_data_.baselink_to_front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}
}  // namespace autoware::behavior_velocity_planner
