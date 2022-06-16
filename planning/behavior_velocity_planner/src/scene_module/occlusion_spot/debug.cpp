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

#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>
#include <scene_module/occlusion_spot/scene_occlusion_spot.hpp>
#include <tier4_autoware_utils/planning/planning_marker_helper.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <utilization/marker_helper.hpp>
#include <utilization/util.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
namespace
{
using builtin_interfaces::msg::Time;
using BasicPolygons = std::vector<lanelet::BasicPolygon2d>;
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
  debug_marker.pose.position = tier4_autoware_utils::createMarkerPosition(0.0, 0.0, 0.0);
  debug_marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  debug_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  const auto & pc = possible_collision;
  // for collision point with margin
  {
    debug_marker.ns = "collision_point";
    debug_marker.type = Marker::CYLINDER;
    debug_marker.pose = pc.collision_with_margin.pose;
    debug_marker.scale = tier4_autoware_utils::createMarkerScale(0.5, 0.5, 0.5);
    debug_marker.color = tier4_autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.5);
    debug_markers.push_back(debug_marker);
  }
  // cylinder at collision_point point
  {
    debug_marker.ns = "collision_point_with_margin";
    debug_marker.type = Marker::CYLINDER;
    debug_marker.pose = pc.collision_pose;
    debug_marker.scale = tier4_autoware_utils::createMarkerScale(0.5, 0.5, 0.5);
    debug_marker.color = tier4_autoware_utils::createMarkerColor(1.0, 0.5, 0.0, 0.5);
    debug_markers.push_back(debug_marker);
  }

  // cylinder at obstacle point
  {
    debug_marker.ns = "obstacle";
    debug_marker.type = Marker::CYLINDER;
    debug_marker.pose.position = pc.obstacle_info.position;
    debug_marker.color = tier4_autoware_utils::createMarkerColor(0.5, 0.5, 0.5, 0.5);
    debug_marker.scale = tier4_autoware_utils::createMarkerScale(0.8, 0.8, 1.5);
    debug_markers.push_back(debug_marker);
  }

  // arrow marker
  {
    debug_marker.ns = "from_obj_to_collision";
    debug_marker.type = Marker::ARROW;
    debug_marker.scale = tier4_autoware_utils::createMarkerScale(0.05, 0.2, 0.5);
    debug_marker.color = tier4_autoware_utils::createMarkerColor(0.1, 0.1, 0.1, 0.5);
    debug_marker.points = {pc.obstacle_info.position, pc.intersection_pose.position};
    debug_markers.push_back(debug_marker);
  }

  if (show_text) {
    // info text at obstacle point
    debug_marker.ns = "info";
    debug_marker.type = Marker::TEXT_VIEW_FACING;
    debug_marker.pose = pc.collision_with_margin.pose;
    debug_marker.scale.z = 1.0;
    debug_marker.color = tier4_autoware_utils::createMarkerColor(1.0, 1.0, 0.0, 1.0);
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
  auto & possible_collisions = debug_data.possible_collisions;
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
  const BasicPolygons & polygons, const std::string ns, const int id, const double z)
{
  MarkerArray debug_markers;
  Marker debug_marker;
  debug_marker.header.frame_id = "map";
  debug_marker.header.stamp = rclcpp::Time(0);
  debug_marker.id = planning_utils::bitShift(id);
  debug_marker.type = Marker::LINE_STRIP;
  debug_marker.action = Marker::ADD;
  debug_marker.pose.position = tier4_autoware_utils::createMarkerPosition(0.0, 0.0, 0);
  debug_marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  debug_marker.scale = tier4_autoware_utils::createMarkerScale(0.1, 0.1, 0.1);
  debug_marker.color = tier4_autoware_utils::createMarkerColor(1.0, 1.0, 1.0, 0.5);
  debug_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
  debug_marker.ns = ns;
  for (const auto & poly : polygons) {
    for (const auto & p : poly) {
      geometry_msgs::msg::Point point =
        tier4_autoware_utils::createMarkerPosition(p.x(), p.y(), z + 0.5);
      debug_marker.points.push_back(point);
    }
    debug_markers.markers.push_back(debug_marker);
    debug_marker.id++;
    debug_marker.points.clear();
  }
  return debug_markers;
}

MarkerArray makeSlicePolygonMarker(
  const Polygons2d & slices, const std::string ns, const int id, const double z)
{
  MarkerArray debug_markers;
  Marker debug_marker;
  debug_marker.header.frame_id = "map";
  debug_marker.header.stamp = rclcpp::Time(0);
  debug_marker.id = planning_utils::bitShift(id);
  debug_marker.type = Marker::LINE_STRIP;
  debug_marker.action = Marker::ADD;
  debug_marker.pose.position = tier4_autoware_utils::createMarkerPosition(0.0, 0.0, 0);
  debug_marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  debug_marker.scale = tier4_autoware_utils::createMarkerScale(0.1, 0.1, 0.1);
  debug_marker.color = tier4_autoware_utils::createMarkerColor(1.0, 0.0, 1.0, 0.3);
  debug_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
  debug_marker.ns = ns;
  for (const auto & slice : slices) {
    for (const auto & p : slice.outer()) {
      geometry_msgs::msg::Point point = tier4_autoware_utils::createMarkerPosition(p.x(), p.y(), z);
      debug_marker.points.push_back(point);
    }
    debug_markers.markers.push_back(debug_marker);
    debug_marker.id++;
    debug_marker.points.clear();
  }
  return debug_markers;
}

MarkerArray createPathMarkerArray(
  const PathWithLaneId & path, const std::string & ns, const int64_t lane_id, const double r,
  const double g, const double b)
{
  MarkerArray msg;
  int32_t uid = planning_utils::bitShift(lane_id);
  int32_t i = 0;
  for (const auto & p : path.points) {
    Marker marker{};
    marker.header.frame_id = "map";
    marker.ns = ns;
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;
    marker.pose = p.point.pose;
    marker.scale = createMarkerScale(0.6, 0.3, 0.3);
    if (std::find(p.lane_ids.begin(), p.lane_ids.end(), lane_id) != p.lane_ids.end()) {
      // if p.lane_ids has lane_id
      marker.color = createMarkerColor(r, g, b, 0.999);
    } else {
      marker.color = createMarkerColor(0.5, 0.5, 0.5, 0.999);
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

MarkerArray createOcclusionMarkerArray(
  const std::vector<geometry_msgs::msg::Point> & occlusion_points, const int64_t module_id)
{
  MarkerArray msg;
  {
    const Time now = rclcpp::Time(0);
    auto marker = createDefaultMarker(
      "map", now, "occlusion", 0, Marker::SPHERE, createMarkerColor(1.0, 0.0, 0.0, 0.999));
    marker.scale = createMarkerScale(0.5, 0.5, 0.5);
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    for (size_t i = 0; i < occlusion_points.size(); ++i) {
      marker.id = i + planning_utils::bitShift(module_id);
      marker.pose.position = occlusion_points.at(i);
      msg.markers.push_back(marker);
    }
  }
  return msg;
}
}  // namespace

MarkerArray OcclusionSpotModule::createDebugMarkerArray()
{
  const auto current_time = this->clock_->now();
  MarkerArray debug_marker_array;
  if (!debug_data_.possible_collisions.empty()) {
    appendMarkerArray(makeDebugInfoMarkers(debug_data_), current_time, &debug_marker_array);
  }
  if (!debug_data_.detection_area_polygons.empty()) {
    appendMarkerArray(
      makeSlicePolygonMarker(
        debug_data_.detection_area_polygons, "detection_area", module_id_, debug_data_.z),
      current_time, &debug_marker_array);
  }
  if (!debug_data_.close_partition.empty() && param_.is_show_occlusion) {
    appendMarkerArray(
      makePolygonMarker(debug_data_.close_partition, "close_partition", module_id_, debug_data_.z),
      current_time, &debug_marker_array);
  }
  if (!debug_data_.path_interpolated.points.empty()) {
    appendMarkerArray(
      createPathMarkerArray(debug_data_.path_raw, "path_raw", 0, 0.0, 1.0, 1.0), current_time,
      &debug_marker_array);
    appendMarkerArray(
      createPathMarkerArray(debug_data_.path_interpolated, "path_interpolated", 0, 0.0, 1.0, 1.0),
      current_time, &debug_marker_array);
  }
  if (!debug_data_.occlusion_points.empty()) {
    appendMarkerArray(
      createOcclusionMarkerArray(debug_data_.occlusion_points, module_id_), current_time,
      &debug_marker_array);
  }
  return debug_marker_array;
}

MarkerArray OcclusionSpotModule::createVirtualWallMarkerArray()
{
  const auto current_time = this->clock_->now();

  MarkerArray wall_marker;
  std::string module_name = "occlusion_spot";
  if (!debug_data_.possible_collisions.empty()) {
    for (size_t id = 0; id < debug_data_.possible_collisions.size(); id++) {
      const auto & pose = debug_data_.possible_collisions.at(id).intersection_pose;
      appendMarkerArray(
        tier4_autoware_utils::createSlowDownVirtualWallMarker(pose, module_name, current_time, id),
        current_time, &wall_marker);
    }
  }
  return wall_marker;
}
}  // namespace behavior_velocity_planner
