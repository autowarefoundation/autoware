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

#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"

#include <behavior_path_planner/scene_module/lane_change/debug.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <cstdlib>
#include <iomanip>
#include <string>
#include <vector>

namespace marker_utils::lane_change_markers
{
using geometry_msgs::msg::Point;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;

MarkerArray showObjectInfo(
  const std::unordered_map<std::string, CollisionCheckDebug> & obj_debug_vec, std::string && ns)
{
  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, 0L, Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.5, 0.5, 0.5), createMarkerColor(0.0, 1.0, 1.0, 0.999));

  MarkerArray marker_array;
  int32_t id{0};

  marker_array.markers.reserve(obj_debug_vec.size());

  for (const auto & [uuid, info] : obj_debug_vec) {
    marker.id = ++id;
    marker.pose = info.current_pose;

    std::ostringstream ss;

    ss << info.failed_reason << "\nLon: " << std::setprecision(4) << info.relative_to_ego.position.x
       << "\nLat: " << info.relative_to_ego.position.y
       << "\nPosition: " << (info.is_front ? "front" : "back");

    marker.text = ss.str();

    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

MarkerArray showAllValidLaneChangePath(const std::vector<LaneChangePath> & lanes, std::string && ns)
{
  if (lanes.empty()) {
    return MarkerArray{};
  }

  MarkerArray marker_array;
  int32_t id{0};
  const auto current_time{rclcpp::Clock{RCL_ROS_TIME}.now()};

  constexpr auto colors = colorsList();
  const auto loop_size = std::min(lanes.size(), colors.size());
  marker_array.markers.reserve(loop_size);

  for (std::size_t idx = 0; idx < loop_size; ++idx) {
    if (lanes.at(idx).path.points.empty()) {
      continue;
    }

    const auto & color = colors.at(idx);
    Marker marker = createDefaultMarker(
      "map", current_time, ns, ++id, Marker::LINE_STRIP, createMarkerScale(0.1, 0.1, 0.0),
      createMarkerColor(color[0], color[1], color[2], 0.9));
    marker.points.reserve(lanes.at(idx).path.points.size());

    for (const auto & point : lanes.at(idx).path.points) {
      marker.points.push_back(point.point.pose.position);
    }

    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

MarkerArray showLerpedPose(
  const std::unordered_map<std::string, CollisionCheckDebug> & obj_debug_vec, std::string && ns)
{
  MarkerArray marker_array;
  int32_t id{0};
  const auto current_time{rclcpp::Clock{RCL_ROS_TIME}.now()};
  marker_array.markers.reserve(obj_debug_vec.size());

  for (const auto & [uuid, info] : obj_debug_vec) {
    Marker marker = createDefaultMarker(
      "map", current_time, ns, ++id, Marker::POINTS, createMarkerScale(0.3, 0.3, 0.3),
      createMarkerColor(1.0, 0.3, 1.0, 0.9));
    marker.points.reserve(info.lerped_path.size());

    for (const auto & point : info.lerped_path) {
      marker.points.push_back(point.position);
    }

    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

MarkerArray showEgoPredictedPaths(
  const std::unordered_map<std::string, CollisionCheckDebug> & obj_debug_vec, std::string && ns)
{
  if (obj_debug_vec.empty()) {
    return MarkerArray{};
  }

  MarkerArray marker_array;
  constexpr auto colors = colorsList();
  constexpr float scale_val = 0.2;
  const auto current_time{rclcpp::Clock{RCL_ROS_TIME}.now()};
  marker_array.markers.reserve(obj_debug_vec.size());

  int32_t id{0};
  for (const auto & [uuid, info] : obj_debug_vec) {
    const auto loop_size = std::min(info.ego_predicted_path.size(), colors.size());

    for (std::size_t idx = 0; idx < loop_size; ++idx) {
      const auto & path = info.ego_predicted_path.at(idx).path;
      const auto & color = colors.at(idx);

      Marker marker = createDefaultMarker(
        "map", current_time, ns, ++id, Marker::LINE_STRIP,
        createMarkerScale(scale_val, scale_val, scale_val),
        createMarkerColor(color[0], color[1], color[2], 0.9));

      marker.points.reserve(path.size());

      for (const auto & point : path) {
        marker.points.push_back(point.position);
      }

      marker_array.markers.push_back(marker);
    }
  }
  return marker_array;
}

MarkerArray showPolygon(
  const std::unordered_map<std::string, CollisionCheckDebug> & obj_debug_vec, std::string && ns)
{
  if (obj_debug_vec.empty()) {
    return MarkerArray{};
  }

  constexpr float scale_val{0.2};
  int32_t id{0};
  Marker ego_marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, id, Marker::LINE_STRIP,
    createMarkerScale(scale_val, scale_val, scale_val), createMarkerColor(1.0, 1.0, 1.0, 0.9));
  Marker obj_marker = ego_marker;
  MarkerArray marker_array;

  constexpr auto colors = colorsList();
  const auto loop_size = std::min(obj_debug_vec.size(), colors.size());

  marker_array.markers.reserve(loop_size * 2);
  size_t idx{0};

  for (const auto & [uuid, info] : obj_debug_vec) {
    if (idx >= loop_size) {
      break;
    }
    const auto & color = colors.at(idx);
    const auto & ego_polygon = info.ego_polygon.outer();
    ego_marker.id = ++id;
    ego_marker.color = createMarkerColor(color[0], color[1], color[2], 0.8);
    ego_marker.points.reserve(ego_polygon.size());
    for (const auto & p : ego_polygon) {
      ego_marker.points.push_back(tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0));
    }
    marker_array.markers.push_back(ego_marker);

    const auto & obj_polygon = info.obj_polygon.outer();
    obj_marker.id = ++id;
    obj_marker.color = createMarkerColor(color[0], color[1], color[2], 0.8);
    obj_marker.points.reserve(obj_polygon.size());
    for (const auto & p : obj_polygon) {
      obj_marker.points.push_back(tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0));
    }
    marker_array.markers.push_back(obj_marker);
    ++idx;
    ego_marker.points.clear();
    obj_marker.points.clear();
  }

  return marker_array;
}

MarkerArray showPolygonPose(
  const std::unordered_map<std::string, CollisionCheckDebug> & obj_debug_vec, std::string && ns)
{
  constexpr auto colors = colorsList();
  const auto loop_size = std::min(colors.size(), obj_debug_vec.size());
  MarkerArray marker_array;
  int32_t id{0};
  size_t idx{0};
  const auto current_time{rclcpp::Clock{RCL_ROS_TIME}.now()};
  marker_array.markers.reserve(obj_debug_vec.size());

  for (const auto & [uuid, info] : obj_debug_vec) {
    const auto & color = colors.at(idx);
    Marker marker = createDefaultMarker(
      "map", current_time, ns, ++id, Marker::POINTS, createMarkerScale(0.2, 0.2, 0.2),
      createMarkerColor(color[0], color[1], color[2], 0.999));
    marker.points.reserve(2);
    marker.points.push_back(info.expected_ego_pose.position);
    marker.points.push_back(info.expected_obj_pose.position);
    marker_array.markers.push_back(marker);
    ++idx;
    if (idx >= loop_size) {
      break;
    }
  }

  return marker_array;
}
}  // namespace marker_utils::lane_change_markers
