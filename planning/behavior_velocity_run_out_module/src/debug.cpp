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

#include "debug.hpp"

#include "scene.hpp"

#include <motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

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

visualization_msgs::msg::MarkerArray createPolygonMarkerArray(
  const std::vector<std::vector<geometry_msgs::msg::Point>> & poly, const rclcpp::Time & time,
  const std::string ns, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color, const double height)
{
  visualization_msgs::msg::MarkerArray marker_array;
  for (size_t i = 0; i < poly.size(); i++) {
    auto marker = createDefaultMarker(
      "map", time, ns, i, visualization_msgs::msg::Marker::LINE_STRIP, scale, color);

    for (const auto & p : poly.at(i)) {
      const auto p_with_height = createPoint(p.x, p.y, height);
      marker.points.push_back(p_with_height);
    }
    // close the polygon
    const auto & p = poly.at(i).front();
    const auto p_with_height = createPoint(p.x, p.y, height);
    marker.points.push_back(p_with_height);

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

}  // namespace

RunOutDebug::RunOutDebug(rclcpp::Node & node) : node_(node)
{
  pub_debug_values_ =
    node.create_publisher<Float32MultiArrayStamped>("~/debug/run_out/debug_values", 1);
  pub_accel_reason_ = node.create_publisher<Int32Stamped>("~/debug/run_out/accel_reason", 1);
  pub_debug_pointcloud_ = node.create_publisher<PointCloud2>(
    "~/debug/run_out/filtered_pointcloud", rclcpp::SensorDataQoS().keep_last(1));
}

void RunOutDebug::pushCollisionPoints(const geometry_msgs::msg::Point & point)
{
  const auto point_with_height = createPoint(point.x, point.y, height_);
  collision_points_.push_back(point_with_height);
}

void RunOutDebug::pushCollisionPoints(const std::vector<geometry_msgs::msg::Point> & points)
{
  for (const auto & p : points) {
    const auto point_with_height = createPoint(p.x, p.y, height_);
    collision_points_.push_back(point_with_height);
  }
}

void RunOutDebug::pushNearestCollisionPoint(const geometry_msgs::msg::Point & point)
{
  const auto point_with_height = createPoint(point.x, point.y, height_);
  nearest_collision_point_.push_back(point_with_height);
}

void RunOutDebug::pushStopPose(const geometry_msgs::msg::Pose & pose)
{
  stop_pose_.push_back(pose);
}

void RunOutDebug::pushPredictedVehiclePolygons(
  const std::vector<geometry_msgs::msg::Point> & polygon)
{
  predicted_vehicle_polygons_.push_back(polygon);
}

void RunOutDebug::pushPredictedObstaclePolygons(
  const std::vector<geometry_msgs::msg::Point> & polygon)
{
  predicted_obstacle_polygons_.push_back(polygon);
}
void RunOutDebug::pushCollisionObstaclePolygons(
  const std::vector<geometry_msgs::msg::Point> & polygon)
{
  collision_obstacle_polygons_.push_back(polygon);
}

void RunOutDebug::pushDetectionAreaPolygons(const Polygon2d & debug_polygon)
{
  std::vector<geometry_msgs::msg::Point> ros_points;
  for (const auto & p : debug_polygon.outer()) {
    ros_points.push_back(tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0));
  }

  detection_area_polygons_.push_back(ros_points);
}

void RunOutDebug::pushMandatoryDetectionAreaPolygons(const Polygon2d & debug_polygon)
{
  std::vector<geometry_msgs::msg::Point> ros_points;
  for (const auto & p : debug_polygon.outer()) {
    ros_points.push_back(tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0));
  }

  mandatory_detection_area_polygons_.push_back(ros_points);
}

void RunOutDebug::pushTravelTimeTexts(
  const double travel_time, const geometry_msgs::msg::Pose pose, const float lateral_offset)
{
  std::stringstream sstream;
  sstream << std::setprecision(4) << travel_time << "s";
  travel_time_texts_.push_back(createDebugText(sstream.str(), pose, lateral_offset));
}

void RunOutDebug::clearDebugMarker()
{
  collision_points_.clear();
  nearest_collision_point_.clear();
  detection_area_polygons_.clear();
  mandatory_detection_area_polygons_.clear();
  predicted_vehicle_polygons_.clear();
  predicted_obstacle_polygons_.clear();
  collision_obstacle_polygons_.clear();
  travel_time_texts_.clear();
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

motion_utils::VirtualWalls RunOutDebug::createVirtualWalls()
{
  motion_utils::VirtualWalls virtual_walls;
  motion_utils::VirtualWall wall;
  wall.text = "run_out";
  wall.style = motion_utils::VirtualWallType::stop;
  for (const auto & p : stop_pose_) {
    wall.pose = p;
    virtual_walls.push_back(wall);
  }
  stop_pose_.clear();

  return virtual_walls;
}

visualization_msgs::msg::MarkerArray RunOutDebug::createVisualizationMarkerArrayFromDebugData(
  const builtin_interfaces::msg::Time & current_time)
{
  visualization_msgs::msg::MarkerArray msg;

  if (!collision_points_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "collision_points", 0, visualization_msgs::msg::Marker::SPHERE_LIST,
      createMarkerScale(0.3, 0.3, 0.3), createMarkerColor(0, 0.0, 1.0, 0.999));
    for (const auto & p : collision_points_) {
      marker.points.push_back(p);
    }
    msg.markers.push_back(marker);
  }

  if (!nearest_collision_point_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "nearest_collision_point", 0,
      visualization_msgs::msg::Marker::SPHERE_LIST, createMarkerScale(0.6, 0.6, 0.6),
      createMarkerColor(1.0, 0, 0, 0.999));
    for (const auto & p : nearest_collision_point_) {
      marker.points.push_back(p);
    }
    msg.markers.push_back(marker);
  }

  if (!predicted_vehicle_polygons_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "predicted_vehicle_polygons", 0,
      visualization_msgs::msg::Marker::LINE_STRIP, createMarkerScale(0.05, 0.0, 0.0),
      createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (const auto & poly : predicted_vehicle_polygons_) {
      for (const auto & p : poly) {
        marker.points.push_back(p);
      }
      // close the polygon
      marker.points.push_back(poly.front());
    }

    msg.markers.push_back(marker);
  }

  if (!predicted_obstacle_polygons_.empty()) {
    appendMarkerArray(
      createPolygonMarkerArray(
        predicted_obstacle_polygons_, current_time, "predicted_obstacle_polygons",
        createMarkerScale(0.02, 0.0, 0.0), createMarkerColor(1.0, 1.0, 0.0, 0.999), height_),
      &msg);
  }

  if (!collision_obstacle_polygons_.empty()) {
    appendMarkerArray(
      createPolygonMarkerArray(
        collision_obstacle_polygons_, current_time, "collision_obstacle_polygons",
        createMarkerScale(0.04, 0.0, 0.0), createMarkerColor(1.0, 0.0, 0.0, 0.999), height_),
      &msg);
  }

  if (!detection_area_polygons_.empty()) {
    appendMarkerArray(
      createPolygonMarkerArray(
        detection_area_polygons_, current_time, "detection_area_polygons",
        createMarkerScale(0.04, 0.0, 0.0), createMarkerColor(0.0, 0.0, 1.0, 0.999), height_),
      &msg);
  }

  if (!mandatory_detection_area_polygons_.empty()) {
    appendMarkerArray(
      createPolygonMarkerArray(
        mandatory_detection_area_polygons_, current_time, "mandatory_detection_area_polygons",
        createMarkerScale(0.04, 0.0, 0.0), createMarkerColor(1.0, 1.0, 0.0, 0.999), height_),
      &msg);
  }

  if (!travel_time_texts_.empty()) {
    auto marker = createDefaultMarker(
      "map", current_time, "travel_time_texts", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.0, 0.0, 0.8),
      createMarkerColor(1.0, 1.0, 1.0, 0.999));

    constexpr float height_offset = 2.0;
    for (const auto & text : travel_time_texts_) {
      marker.pose.position = text.position;
      marker.pose.position.z += height_offset;
      marker.text = text.text;

      msg.markers.push_back(marker);
      marker.id++;
    }
  }

  return msg;
}
void RunOutDebug::setAccelReason(const AccelReason & accel_reason)
{
  accel_reason_ = accel_reason;
}

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

void RunOutDebug::publishFilteredPointCloud(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const std_msgs::msg::Header header)
{
  PointCloud2 pc;
  pcl::toROSMsg(pointcloud, pc);
  pc.header = header;

  pub_debug_pointcloud_->publish(pc);
}

void RunOutDebug::publishFilteredPointCloud(const PointCloud2 & pointcloud)
{
  pub_debug_pointcloud_->publish(pointcloud);
}

void RunOutDebug::publishEmptyPointCloud()
{
  PointCloud2 pc;
  pcl::toROSMsg(pcl::PointCloud<pcl::PointXYZ>(), pc);
  // set arbitrary frame id to avoid warning
  pc.header.frame_id = "map";
  pc.header.stamp = node_.now();

  pub_debug_pointcloud_->publish(pc);
}

void RunOutDebug::setHeight(const double height)
{
  height_ = height;
}

// scene module
visualization_msgs::msg::MarkerArray RunOutModule::createDebugMarkerArray()
{
  return debug_ptr_->createVisualizationMarkerArray();
}

motion_utils::VirtualWalls RunOutModule::createVirtualWalls()
{
  return debug_ptr_->createVirtualWalls();
}

}  // namespace behavior_velocity_planner
