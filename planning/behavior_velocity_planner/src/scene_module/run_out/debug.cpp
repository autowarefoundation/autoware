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

#include "scene_module/run_out/debug.hpp"

#include "scene_module/run_out/scene.hpp"

using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerOrientation;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;

namespace behavior_velocity_planner
{
namespace
{
RunOutDebug::TextWithPosition createDebugText(
  const std::string text, const geometry_msgs::msg::Pose pose, const float lateral_offset)
{
  const auto offset_pose = tier4_autoware_utils::calcOffsetPose(pose, 0, lateral_offset, 0);

  RunOutDebug::TextWithPosition text_with_position;
  text_with_position.text = text;
  text_with_position.position = offset_pose.position;

  return text_with_position;
}

RunOutDebug::TextWithPosition createDebugText(
  const std::string text, const geometry_msgs::msg::Point position)
{
  RunOutDebug::TextWithPosition text_with_position;
  text_with_position.text = text;
  text_with_position.position = position;

  return text_with_position;
}

}  // namespace

RunOutDebug::RunOutDebug(rclcpp::Node & node) : node_(node)
{
  pub_debug_values_ =
    node.create_publisher<Float32MultiArrayStamped>("~/run_out/debug/debug_values", 1);
  pub_accel_reason_ = node.create_publisher<Int32Stamped>("~/run_out/debug/accel_reason", 1);
  pub_debug_trajectory_ = node.create_publisher<Trajectory>("~/run_out/debug/trajectory", 1);
}

void RunOutDebug::pushDebugPoints(const pcl::PointXYZ & debug_point)
{
  geometry_msgs::msg::Point ros_point;
  ros_point.x = debug_point.x;
  ros_point.y = debug_point.y;
  ros_point.z = debug_point.z;

  debug_points_.push_back(ros_point);
}

void RunOutDebug::pushDebugPoints(const geometry_msgs::msg::Point & debug_point)
{
  debug_points_.push_back(debug_point);
}

void RunOutDebug::pushDebugPoints(const std::vector<geometry_msgs::msg::Point> & debug_points)
{
  for (const auto & p : debug_points) {
    debug_points_.push_back(p);
  }
}

void RunOutDebug::pushDebugPoints(
  const geometry_msgs::msg::Point & debug_point, const PointType point_type)
{
  switch (point_type) {
    case PointType::Blue:
      debug_points_.push_back(debug_point);
      break;

    case PointType::Red:
      debug_points_red_.push_back(debug_point);
      break;

    case PointType::Yellow:
      debug_points_yellow_.push_back(debug_point);
      break;

    default:
      break;
  }
}

void RunOutDebug::pushStopPose(const geometry_msgs::msg::Pose & pose)
{
  stop_pose_.push_back(pose);
}

void RunOutDebug::pushDebugLines(const std::vector<geometry_msgs::msg::Point> & debug_line)
{
  debug_lines_.push_back(debug_line);
}

void RunOutDebug::pushDebugPolygons(const std::vector<geometry_msgs::msg::Point> & debug_polygon)
{
  debug_polygons_.push_back(debug_polygon);
}

void RunOutDebug::pushDetectionAreaPolygons(const Polygon2d & debug_polygon)
{
  std::vector<geometry_msgs::msg::Point> ros_points;
  for (const auto & p : debug_polygon.outer()) {
    ros_points.push_back(tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0));
  }

  detection_area_polygons_.push_back(ros_points);
}

void RunOutDebug::pushDebugTexts(const TextWithPosition & debug_text)
{
  debug_texts_.push_back(debug_text);
}

void RunOutDebug::pushDebugTexts(
  const std::string text, const geometry_msgs::msg::Pose pose, const float lateral_offset)
{
  debug_texts_.push_back(createDebugText(text, pose, lateral_offset));
}

void RunOutDebug::pushDebugTexts(const std::string text, const geometry_msgs::msg::Point position)
{
  debug_texts_.push_back(createDebugText(text, position));
}

void RunOutDebug::clearDebugMarker()
{
  debug_points_.clear();
  debug_points_red_.clear();
  debug_points_yellow_.clear();
  debug_lines_.clear();
  debug_polygons_.clear();
  detection_area_polygons_.clear();
  debug_texts_.clear();
}

visualization_msgs::msg::MarkerArray RunOutDebug::createVisualizationMarkerArray()
{
  rclcpp::Time current_time = node_.now();

  // create marker array from debug data
  auto visualization_marker_array = createVisualizationMarkerArrayFromDebugData(current_time);

  // clear debug data
  clearDebugMarker();

  return visualization_marker_array;
}

visualization_msgs::msg::MarkerArray RunOutDebug::createVirtualWallMarkerArray()
{
  visualization_msgs::msg::MarkerArray wall_marker;
  rclcpp::Time now = node_.now();
  size_t id = 0;

  for (const auto & p : stop_pose_) {
    appendMarkerArray(
      tier4_autoware_utils::createStopVirtualWallMarker(p, "run_out", now, id++), &wall_marker);
  }

  stop_pose_.clear();

  return wall_marker;
}

visualization_msgs::msg::MarkerArray RunOutDebug::createVisualizationMarkerArrayFromDebugData(
  const builtin_interfaces::msg::Time & current_time)
{
  visualization_msgs::msg::MarkerArray msg;

  if (!debug_points_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "debug_points", 0, visualization_msgs::msg::Marker::SPHERE_LIST,
      createMarkerScale(0.5, 0.5, 0.5), createMarkerColor(0, 0.0, 1.0, 0.999));
    for (const auto & p : debug_points_) {
      marker.points.push_back(p);
    }
    msg.markers.push_back(marker);
  }

  if (!debug_points_red_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "debug_points_red", 0, visualization_msgs::msg::Marker::SPHERE_LIST,
      createMarkerScale(1.0, 1.0, 1.0), createMarkerColor(1.0, 0, 0, 0.999));
    for (const auto & p : debug_points_red_) {
      marker.points.push_back(p);
    }
    msg.markers.push_back(marker);
  }

  if (!debug_points_yellow_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "debug_points_yellow", 0, visualization_msgs::msg::Marker::SPHERE_LIST,
      createMarkerScale(0.7, 0.7, 0.7), createMarkerColor(1.0, 1.0, 0, 0.999));
    for (const auto & p : debug_points_yellow_) {
      marker.points.push_back(p);
    }
    msg.markers.push_back(marker);
  }

  if (!debug_lines_.empty()) {
    int32_t marker_id = 0;
    for (const auto & line : debug_lines_) {
      auto marker = createDefaultMarker(
        "map", current_time, "debug_lines", marker_id, visualization_msgs::msg::Marker::LINE_STRIP,
        createMarkerScale(0.1, 0.0, 0.0), createMarkerColor(0, 0, 1.0, 0.999));
      for (const auto & p : line) {
        marker.points.push_back(p);
      }
      msg.markers.push_back(marker);
      marker_id++;
    }
  }

  if (!debug_polygons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "debug_polygons", 0, visualization_msgs::msg::Marker::LINE_LIST,
      createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(1.0, 1.0, 0.0, 0.999));

    for (const auto & poly : debug_polygons_) {
      for (size_t i = 0; i < poly.size() - 1; i++) {
        marker.points.push_back(poly.at(i));
        marker.points.push_back(poly.at(i + 1));
      }
      marker.points.push_back(poly.back());
      marker.points.push_back(poly.front());
    }

    msg.markers.push_back(marker);
  }

  if (!detection_area_polygons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "detection_area_polygon", 0, visualization_msgs::msg::Marker::LINE_LIST,
      createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(0.0, 0.0, 1.0, 0.999));

    for (const auto & poly : detection_area_polygons_) {
      for (size_t i = 0; i < poly.size() - 1; i++) {
        marker.points.push_back(poly.at(i));
        marker.points.push_back(poly.at(i + 1));
      }
      marker.points.push_back(poly.back());
      marker.points.push_back(poly.front());
    }

    msg.markers.push_back(marker);
  }

  if (!debug_texts_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "debug_texts", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.0, 0.0, 0.8), createMarkerColor(1.0, 1.0, 1.0, 0.999));

    constexpr float height_offset = 2.0;
    for (const auto & text : debug_texts_) {
      marker.pose.position = text.position;
      marker.pose.position.z += height_offset;
      marker.text = text.text;

      msg.markers.push_back(marker);
      marker.id++;
    }
  }

  return msg;
}
void RunOutDebug::setAccelReason(const AccelReason & accel_reason) { accel_reason_ = accel_reason; }

void RunOutDebug::publishDebugValue()
{
  // publish debug values
  tier4_debug_msgs::msg::Float32MultiArrayStamped debug_msg{};
  debug_msg.stamp = node_.now();
  for (const auto & v : debug_values_.getValues()) {
    debug_msg.data.push_back(v);
  }
  pub_debug_values_->publish(debug_msg);

  Int32Stamped accel_reason;
  accel_reason.stamp = node_.now();
  accel_reason.data = static_cast<int>(accel_reason_);
  pub_accel_reason_->publish(accel_reason);
}

void RunOutDebug::publishDebugTrajectory(const Trajectory & trajectory)
{
  pub_debug_trajectory_->publish(trajectory);
}

// scene module
visualization_msgs::msg::MarkerArray RunOutModule::createDebugMarkerArray()
{
  return debug_ptr_->createVisualizationMarkerArray();
}

visualization_msgs::msg::MarkerArray RunOutModule::createVirtualWallMarkerArray()
{
  return debug_ptr_->createVirtualWallMarkerArray();
}

}  // namespace behavior_velocity_planner
