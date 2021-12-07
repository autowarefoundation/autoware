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

#include <scene_module/crosswalk/scene_crosswalk.hpp>
#include <scene_module/crosswalk/scene_walkway.hpp>
#include <utilization/marker_helper.hpp>
#include <utilization/util.hpp>

#include <vector>

namespace behavior_velocity_planner
{
namespace
{
visualization_msgs::msg::MarkerArray createCrosswalkMarkers(
  const DebugData & debug_data, const int64_t module_id)
{
  visualization_msgs::msg::MarkerArray msg;
  tf2::Transform tf_base_link2front(
    tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(debug_data.base_link2front, 0.0, 0.0));

  // Crosswalk polygons
  int32_t uid = planning_utils::bitShift(module_id);
  for (size_t i = 0; i < debug_data.crosswalk_polygons.size(); ++i) {
    std::vector<Eigen::Vector3d> polygon = debug_data.crosswalk_polygons.at(i);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";

    marker.ns = "crosswalk polygon line";
    marker.id = uid + i;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < polygon.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = polygon.at(j).x();
      point.y = polygon.at(j).y();
      point.z = polygon.at(j).z();
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);

    marker.ns = "crosswalk polygon point";
    marker.id = i;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < polygon.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = polygon.at(j).x();
      point.y = polygon.at(j).y();
      point.z = polygon.at(j).z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Collision line
  for (size_t i = 0; i < debug_data.collision_lines.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "collision line";
    marker.id = uid + i;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < debug_data.collision_lines.at(i).size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = debug_data.collision_lines.at(i).at(j).x();
      point.y = debug_data.collision_lines.at(i).at(j).y();
      point.z = debug_data.collision_lines.at(i).at(j).z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Collision point
  if (!debug_data.collision_points.empty()) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "collision point";
    marker.id = 0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < debug_data.collision_points.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = debug_data.collision_points.at(j).x();
      point.y = debug_data.collision_points.at(j).y();
      point.z = debug_data.collision_points.at(j).z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Slow polygon
  for (size_t i = 0; i < debug_data.slow_polygons.size(); ++i) {
    std::vector<Eigen::Vector3d> polygon = debug_data.slow_polygons.at(i);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";

    marker.ns = "slow polygon line";
    marker.id = uid + i;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < polygon.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = polygon.at(j).x();
      point.y = polygon.at(j).y();
      point.z = polygon.at(j).z();
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Slow point
  if (!debug_data.slow_poses.empty()) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "slow point";
    marker.id = 0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < debug_data.slow_poses.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = debug_data.slow_poses.at(j).position.x;
      point.y = debug_data.slow_poses.at(j).position.y;
      point.z = debug_data.slow_poses.at(j).position.z;
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Stop polygon
  for (size_t i = 0; i < debug_data.stop_polygons.size(); ++i) {
    std::vector<Eigen::Vector3d> polygon = debug_data.stop_polygons.at(i);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";

    marker.ns = "stop polygon line";
    marker.id = uid + i;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < polygon.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = polygon.at(j).x();
      point.y = polygon.at(j).y();
      point.z = polygon.at(j).z();
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Stop point
  if (!debug_data.stop_poses.empty()) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "stop point";
    marker.id = module_id;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < debug_data.stop_poses.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = debug_data.stop_poses.at(j).position.x;
      point.y = debug_data.stop_poses.at(j).position.y;
      point.z = debug_data.stop_poses.at(j).position.z;
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Stop VirtualWall
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
  // Factor Text
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
    marker.text = "crosswalk";
    msg.markers.push_back(marker);
  }

  // Slow VirtualWall
  for (size_t j = 0; j < debug_data.slow_poses.size(); ++j) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "slow virtual_wall";
    marker.id = uid + j;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(debug_data.slow_poses.at(j), tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 5.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    msg.markers.push_back(marker);
  }
  // Slow Factor Text
  for (size_t j = 0; j < debug_data.slow_poses.size(); ++j) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "slow factor_text";
    marker.id = uid + j;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(debug_data.slow_poses.at(j), tf_map2base_link);
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
    marker.text = "crosswalk";
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray createWalkwayMarkers(
  const DebugData & debug_data, const int64_t module_id)
{
  int32_t uid = planning_utils::bitShift(module_id);
  visualization_msgs::msg::MarkerArray msg;
  tf2::Transform tf_base_link2front(
    tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(debug_data.base_link2front, 0.0, 0.0));

  // Stop point
  if (!debug_data.stop_poses.empty()) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = "stop point";
    marker.id = module_id;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < debug_data.stop_poses.size(); ++j) {
      geometry_msgs::msg::Point point;
      point.x = debug_data.stop_poses.at(j).position.x;
      point.y = debug_data.stop_poses.at(j).position.y;
      point.z = debug_data.stop_poses.at(j).position.z;
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Stop VirtualWall
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

    visualization_msgs::msg::Marker range_marker;
    range_marker.header.frame_id = "map";
    range_marker.ns = "walkway stop judge range";
    range_marker.id = uid + debug_data.stop_poses.size() + j;
    range_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    range_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    range_marker.action = visualization_msgs::msg::Marker::ADD;
    range_marker.pose.position.x = 0;
    range_marker.pose.position.y = 0;
    range_marker.pose.position.z = 0;
    range_marker.pose.orientation.x = 0.0;
    range_marker.pose.orientation.y = 0.0;
    range_marker.pose.orientation.z = 0.0;
    range_marker.pose.orientation.w = 1.0;
    range_marker.scale.x = 0.1;
    range_marker.scale.y = 0.1;
    range_marker.color.a = 0.5;  // Don't forget to set the alpha!
    range_marker.color.r = 1.0;
    range_marker.color.g = 0.0;
    range_marker.color.b = 0.0;
    geometry_msgs::msg::Point point;
    point.x = debug_data.stop_poses.at(j).position.x;
    point.y = debug_data.stop_poses.at(j).position.y;
    point.z = debug_data.stop_poses.at(j).position.z;
    for (size_t i = 0; i < 50; ++i) {
      geometry_msgs::msg::Point range_point;
      range_point.x = point.x + debug_data.stop_judge_range * std::cos(M_PI * 2 / 50 * i);
      range_point.y = point.y + debug_data.stop_judge_range * std::sin(M_PI * 2 / 50 * i);
      range_point.z = point.z;
      range_marker.points.push_back(range_point);
    }
    range_marker.points.push_back(range_marker.points.front());
    msg.markers.push_back(range_marker);
  }
  // Factor Text
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
    marker.text = "walkway";
    msg.markers.push_back(marker);
  }

  return msg;
}

}  // namespace

visualization_msgs::msg::MarkerArray CrosswalkModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  appendMarkerArray(
    createCrosswalkMarkers(debug_data_, module_id_), this->clock_->now(), &debug_marker_array);

  return debug_marker_array;
}

visualization_msgs::msg::MarkerArray WalkwayModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  appendMarkerArray(
    createWalkwayMarkers(debug_data_, module_id_), this->clock_->now(), &debug_marker_array);

  return debug_marker_array;
}
}  // namespace behavior_velocity_planner
