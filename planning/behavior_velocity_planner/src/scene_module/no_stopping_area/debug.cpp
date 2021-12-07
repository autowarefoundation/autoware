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

#include "scene_module/no_stopping_area/scene_no_stopping_area.hpp"
#include "utilization/marker_helper.hpp"
#include "utilization/util.hpp"

#include <autoware_utils/planning/planning_marker_helper.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>
#include <vector>

namespace behavior_velocity_planner
{
namespace
{
const double marker_lifetime = 0.2;
using DebugData = NoStoppingAreaModule::DebugData;

lanelet::BasicPoint3d getCentroidPoint(const lanelet::BasicPolygon3d & poly)
{
  lanelet::BasicPoint3d p_sum{0.0, 0.0, 0.0};
  for (const auto & p : poly) {
    p_sum += p;
  }
  return p_sum / poly.size();
}

geometry_msgs::msg::Point toMsg(const lanelet::BasicPoint3d & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

geometry_msgs::msg::Point toPoint2d(const geometry_msgs::msg::Point32 & poly)
{
  geometry_msgs::msg::Point msg;
  msg.x = poly.x;
  msg.y = poly.y;
  msg.z = 0;
  return msg;
}

visualization_msgs::msg::MarkerArray createMarkerArray(
  const DebugData & debug_data, const rclcpp::Time & now, const int64_t module_id)
{
  visualization_msgs::msg::MarkerArray msg;
  const tf2::Transform tf_base_link2front(
    tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(debug_data.base_link2front, 0.0, 0.0));

  // Stop VirtualWall
  const int32_t uid = planning_utils::bitShift(module_id);
  for (size_t j = 0; j < debug_data.stop_poses.size(); ++j) {
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(debug_data.stop_poses.at(j), tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    geometry_msgs::msg::Pose pose;
    tf2::toMsg(tf_map2front, pose);
    msg = autoware_utils::createStopVirtualWallMarker(pose, "no_stopping_area", now, uid + j);
  }
  return msg;
}

visualization_msgs::msg::MarkerArray createLaneletInfoMarkerArray(
  const lanelet::autoware::NoStoppingArea & no_stopping_area_reg_elem, const rclcpp::Time & now)
{
  visualization_msgs::msg::MarkerArray msg;

  // ID
  {
    auto marker = createDefaultMarker(
      "map", now, "no_stopping_area_id", no_stopping_area_reg_elem.id(),
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker.scale = createMarkerScale(0.0, 0.0, 1.0);
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);

    for (const auto & detection_area : no_stopping_area_reg_elem.noStoppingAreas()) {
      const auto poly = detection_area.basicPolygon();

      marker.pose.position = toMsg(poly.front());
      marker.pose.position.z += 2.0;
      marker.text = std::to_string(no_stopping_area_reg_elem.id());

      msg.markers.push_back(marker);
    }
  }

  // Polygon
  {
    auto marker = createDefaultMarker(
      "map", now, "no_stopping_area_polygon", no_stopping_area_reg_elem.id(),
      visualization_msgs::msg::Marker::LINE_LIST, createMarkerColor(0.1, 0.1, 1.0, 0.500));
    marker.scale = createMarkerScale(0.1, 0.0, 0.0);
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);

    for (const auto & no_stopping_area : no_stopping_area_reg_elem.noStoppingAreas()) {
      const auto poly = no_stopping_area.basicPolygon();

      for (size_t i = 0; i < poly.size(); ++i) {
        const auto idx_front = i;
        const auto idx_back = (i == poly.size() - 1) ? 0 : i + 1;

        const auto & p_front = poly.at(idx_front);
        const auto & p_back = poly.at(idx_back);

        marker.points.push_back(toMsg(p_front));
        marker.points.push_back(toMsg(p_back));
      }
    }
    msg.markers.push_back(marker);
  }

  const auto & stop_line = no_stopping_area_reg_elem.stopLine();
  // Polygon to StopLine
  if (stop_line) {
    const auto stop_line_center_point =
      (stop_line.value().front().basicPoint() + stop_line.value().back().basicPoint()) / 2;
    auto marker = createDefaultMarker(
      "map", now, "no_stopping_area_correspondence", no_stopping_area_reg_elem.id(),
      visualization_msgs::msg::Marker::LINE_STRIP, createMarkerColor(0.1, 0.1, 1.0, 0.500));
    marker.scale = createMarkerScale(0.1, 0.0, 0.0);
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);
    for (const auto & detection_area : no_stopping_area_reg_elem.noStoppingAreas()) {
      const auto poly = detection_area.basicPolygon();
      const auto centroid_point = getCentroidPoint(poly);
      for (size_t i = 0; i < poly.size(); ++i) {
        marker.points.push_back(toMsg(centroid_point));
        marker.points.push_back(toMsg(stop_line_center_point));
      }
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray createStuckPointsMarkerArray(
  const std::vector<geometry_msgs::msg::Point> & stuck_points, const rclcpp::Time & now)
{
  visualization_msgs::msg::MarkerArray msg;
  {
    auto marker = createDefaultMarker(
      "map", now, "stuck_points", 0, visualization_msgs::msg::Marker::SPHERE,
      createMarkerColor(1.0, 1.0, 0.0, 0.999));
    marker.scale = createMarkerScale(0.3, 0.3, 0.3);
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);
    for (size_t i = 0; i < stuck_points.size(); ++i) {
      marker.id = i;
      marker.pose.position = stuck_points.at(i);
      msg.markers.push_back(marker);
    }
  }
  return msg;
}

visualization_msgs::msg::MarkerArray createNoStoppingAreaMarkerArray(
  const geometry_msgs::msg::Polygon & stuck_vehicle_detect_area, const std::string & ns,
  const rclcpp::Time & now)
{
  visualization_msgs::msg::MarkerArray msg;
  {
    auto marker = createDefaultMarker(
      "map", now, ns.c_str(), 0, visualization_msgs::msg::Marker::LINE_STRIP,
      createMarkerColor(1.0, 1.0, 0.0, 0.999));
    marker.scale = createMarkerScale(0.1, 0.1, 0.1);
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);

    for (size_t i = 0; i < stuck_vehicle_detect_area.points.size(); ++i) {
      marker.id = i;
      marker.points.emplace_back(toPoint2d(stuck_vehicle_detect_area.points[i]));
    }
    marker.points.emplace_back(toPoint2d(stuck_vehicle_detect_area.points.at(0)));
    msg.markers.push_back(marker);
  }
  return msg;
}

}  // namespace

visualization_msgs::msg::MarkerArray NoStoppingAreaModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;
  const rclcpp::Time current_time = clock_->now();
  appendMarkerArray(
    createMarkerArray(debug_data_, current_time, getModuleId()), current_time, &debug_marker_array);

  appendMarkerArray(
    createLaneletInfoMarkerArray(no_stopping_area_reg_elem_, current_time), current_time,
    &debug_marker_array);

  if (!debug_data_.stuck_points.empty()) {
    appendMarkerArray(
      createStuckPointsMarkerArray(debug_data_.stuck_points, current_time), current_time,
      &debug_marker_array);
  }
  if (!debug_data_.stuck_vehicle_detect_area.points.empty()) {
    appendMarkerArray(
      createNoStoppingAreaMarkerArray(
        debug_data_.stuck_vehicle_detect_area, "stuck_vehicle_detect_area", current_time),
      current_time, &debug_marker_array);
  }
  if (!debug_data_.stop_line_detect_area.points.empty()) {
    appendMarkerArray(
      createNoStoppingAreaMarkerArray(
        debug_data_.stop_line_detect_area, "stop_line_detect_area", current_time),
      current_time, &debug_marker_array);
  }
  return debug_marker_array;
}
}  // namespace behavior_velocity_planner
