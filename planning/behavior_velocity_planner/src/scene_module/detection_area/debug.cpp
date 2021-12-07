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

#include <scene_module/detection_area/scene.hpp>
#include <utilization/marker_helper.hpp>
#include <utilization/util.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>

namespace behavior_velocity_planner
{
namespace
{
using DebugData = DetectionAreaModule::DebugData;

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

visualization_msgs::msg::MarkerArray createMarkerArray(
  const DebugData & debug_data, const int64_t module_id)
{
  visualization_msgs::msg::MarkerArray msg;
  const tf2::Transform tf_base_link2front(
    tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(debug_data.base_link2front, 0.0, 0.0));

  // Stop VirtualWall
  const int32_t uid = planning_utils::bitShift(module_id);
  for (size_t j = 0; j < debug_data.stop_poses.size(); ++j) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "stop_virtual_wall";
    marker.id = uid + j;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(debug_data.stop_poses.at(j), tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 5.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    msg.markers.push_back(marker);
  }
  // DeadLine VirtualWall
  for (size_t j = 0; j < debug_data.dead_line_poses.size(); ++j) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "dead_line_virtual_wall";
    marker.id = uid + j;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(debug_data.dead_line_poses.at(j), tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 5.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    msg.markers.push_back(marker);
  }
  // Facto Text
  for (size_t j = 0; j < debug_data.stop_poses.size(); ++j) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "factor_text";
    marker.id = uid + j;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(debug_data.stop_poses.at(j), tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 2.0;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = "detection area";
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray createCorrespondenceMarkerArray(
  const lanelet::autoware::DetectionArea & detection_area_reg_elem, const rclcpp::Time & now)
{
  visualization_msgs::msg::MarkerArray msg;

  const lanelet::ConstLineString3d stop_line = detection_area_reg_elem.stopLine();
  const auto stop_line_center_point =
    (stop_line.front().basicPoint() + stop_line.back().basicPoint()) / 2;

  // ID
  {
    auto marker = createDefaultMarker(
      "map", now, "detection_area_id", detection_area_reg_elem.id(),
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerColor(1.0, 1.0, 1.0, 0.999));
    marker.scale = createMarkerScale(0.0, 0.0, 1.0);
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & detection_area : detection_area_reg_elem.detectionAreas()) {
      const auto poly = detection_area.basicPolygon();

      marker.pose.position = toMsg(poly.front());
      marker.pose.position.z += 2.0;
      marker.text = std::to_string(detection_area_reg_elem.id());

      msg.markers.push_back(marker);
    }
  }

  // Polygon
  {
    auto marker = createDefaultMarker(
      "map", now, "detection_area_polygon", detection_area_reg_elem.id(),
      visualization_msgs::msg::Marker::LINE_LIST, createMarkerColor(0.1, 0.1, 1.0, 0.500));
    marker.scale = createMarkerScale(0.1, 0.0, 0.0);
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & detection_area : detection_area_reg_elem.detectionAreas()) {
      const auto poly = detection_area.basicPolygon();

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

  // Polygon to StopLine
  {
    auto marker = createDefaultMarker(
      "map", now, "detection_area_correspondence", detection_area_reg_elem.id(),
      visualization_msgs::msg::Marker::LINE_LIST, createMarkerColor(0.1, 0.1, 1.0, 0.500));
    marker.scale = createMarkerScale(0.1, 0.0, 0.0);
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (const auto & detection_area : detection_area_reg_elem.detectionAreas()) {
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

visualization_msgs::msg::MarkerArray createObstacleMarkerArray(
  const std::vector<geometry_msgs::msg::Point> & obstacle_points, const rclcpp::Time & now)
{
  visualization_msgs::msg::MarkerArray msg;

  {
    auto marker = createDefaultMarker(
      "map", now, "obstacles", 0, visualization_msgs::msg::Marker::SPHERE,
      createMarkerColor(1.0, 0.0, 0.0, 0.999));
    marker.scale = createMarkerScale(0.3, 0.3, 0.3);
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);

    for (size_t i = 0; i < obstacle_points.size(); ++i) {
      marker.id = i;
      marker.pose.position = obstacle_points.at(i);

      msg.markers.push_back(marker);
    }
  }

  return msg;
}

}  // namespace

visualization_msgs::msg::MarkerArray DetectionAreaModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;
  const rclcpp::Time current_time = clock_->now();
  appendMarkerArray(
    createMarkerArray(debug_data_, getModuleId()), current_time, &debug_marker_array);

  if (!debug_data_.stop_poses.empty()) {
    appendMarkerArray(
      createCorrespondenceMarkerArray(detection_area_reg_elem_, current_time), current_time,
      &debug_marker_array);

    appendMarkerArray(
      createObstacleMarkerArray(debug_data_.obstacle_points, current_time), current_time,
      &debug_marker_array);
  }

  return debug_marker_array;
}
}  // namespace behavior_velocity_planner
