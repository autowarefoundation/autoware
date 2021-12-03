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

#include "obstacle_avoidance_planner/debug.hpp"

#include "obstacle_avoidance_planner/eb_path_optimizer.hpp"
#include "obstacle_avoidance_planner/marker_helper.hpp"
#include "obstacle_avoidance_planner/mpt_optimizer.hpp"
#include "obstacle_avoidance_planner/process_cv.hpp"
#include "obstacle_avoidance_planner/util.hpp"

#include <opencv2/core.hpp>
#include <rclcpp/clock.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/utils.h>

#include <string>
#include <vector>

visualization_msgs::msg::MarkerArray getDebugVisualizationMarker(
  const DebugData & debug_data,
  // const std::vector<geometry_msgs::msg::Point> & interpolated_points,
  // const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & smoothed_points,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points,
  const VehicleParam & vehicle_param)
{
  const auto points_marker_array = getDebugPointsMarkers(
    debug_data.interpolated_points, optimized_points, debug_data.straight_points,
    debug_data.fixed_points, debug_data.non_fixed_points);
  const auto constrain_marker_array =
    getDebugConstrainMarkers(debug_data.constrain_rectangles, "constrain_rect");
  const auto extending_constrain_marker_array = getDebugConstrainMarkers(
    debug_data.constrain_rectangles_for_extending, "extending_constrain_rect");

  visualization_msgs::msg::MarkerArray vis_marker_array;
  if (debug_data.is_expected_to_over_drivable_area && !optimized_points.empty()) {
    const auto virtual_wall_pose = getVirtualWallPose(optimized_points.back().pose, vehicle_param);
    appendMarkerArray(
      getVirtualWallMarkerArray(virtual_wall_pose, "virtual_wall", 1.0, 0, 0), &vis_marker_array);
    appendMarkerArray(
      getVirtualWallTextMarkerArray(virtual_wall_pose, "virtual_wall_text", 1.0, 1.0, 1.0),
      &vis_marker_array);
  }
  appendMarkerArray(points_marker_array, &vis_marker_array);
  appendMarkerArray(constrain_marker_array, &vis_marker_array);
  appendMarkerArray(extending_constrain_marker_array, &vis_marker_array);
  appendMarkerArray(
    getPointsMarkerArray(
      debug_data.fixed_points_for_extending, "fixed_points_for_extending", 0.99, 0.99, 0.2),
    &vis_marker_array);
  appendMarkerArray(
    getPointsMarkerArray(
      debug_data.non_fixed_points_for_extending, "non_fixed_points_for_extending", 0.99, 0.99, 0.2),
    &vis_marker_array);
  appendMarkerArray(
    getPointsMarkerArray(
      debug_data.interpolated_points_for_extending, "interpolated_points_for_extending", 0.99, 0.99,
      0.2),
    &vis_marker_array);
  appendMarkerArray(
    getObjectsMarkerArray(debug_data.avoiding_objects, "avoiding_objects", 0.99, 0.99, 0.2),
    &vis_marker_array);
  appendMarkerArray(
    getRectanglesMarkerArray(debug_data.vehicle_footprints, "vehicle_footprint", 0.99, 0.99, 0.2),
    &vis_marker_array);
  appendMarkerArray(
    getRectanglesMarkerArray(
      debug_data.current_vehicle_footprints, "current_vehicle_footprint", 0.99, 0.99, 0.2),
    &vis_marker_array);
  appendMarkerArray(
    getRectanglesNumMarkerArray(
      debug_data.vehicle_footprints, "num_vehicle_footprint", 0.99, 0.99, 0.2),
    &vis_marker_array);
  appendMarkerArray(
    getPointsMarkerArray(
      debug_data.bounds_candidate_for_base_points, "bounds_candidate_for_base", 0.99, 0.99, 0.2),
    &vis_marker_array);
  appendMarkerArray(
    getPointsMarkerArray(
      debug_data.bounds_candidate_for_top_points, "bounds_candidate_for_top", 0.99, 0.99, 0.2),
    &vis_marker_array);
  appendMarkerArray(
    getPointsMarkerArray(debug_data.fixed_mpt_points, "fixed_mpt_points", 0.99, 0.0, 0.2),
    &vis_marker_array);
  appendMarkerArray(
    getPointsTextMarkerArray(
      debug_data.bounds_candidate_for_base_points, "bounds_candidate_base_text", 0.99, 0.99, 0.2),
    &vis_marker_array);
  appendMarkerArray(
    getPointsTextMarkerArray(
      debug_data.bounds_candidate_for_top_points, "bounds_candidate_top_text", 0.99, 0.99, 0.2),
    &vis_marker_array);
  appendMarkerArray(
    getPointsTextMarkerArray(debug_data.smoothed_points, "smoothed_points_text", 0.99, 0.99, 0.2),
    &vis_marker_array);
  appendMarkerArray(
    getBaseBoundsLineMarkerArray(
      debug_data.bounds, debug_data.bounds_candidate_for_base_points, "base_bounds_line", 0.99,
      0.99, 0.2),
    &vis_marker_array);
  appendMarkerArray(
    getTopBoundsLineMarkerArray(
      debug_data.bounds, debug_data.bounds_candidate_for_top_points, "top_bounds_line", 0.99, 0.99,
      0.2),
    &vis_marker_array);
  appendMarkerArray(
    getMidBoundsLineMarkerArray(
      debug_data.bounds, debug_data.bounds_candidate_for_mid_points, "mid_bounds_line", 0.99, 0.99,
      0.2),
    &vis_marker_array);

  return vis_marker_array;
}

geometry_msgs::msg::Pose getVirtualWallPose(
  const geometry_msgs::msg::Pose & target_pose, const VehicleParam & vehicle_param)
{
  const double base_link2front = vehicle_param.wheelbase + vehicle_param.front_overhang;
  tf2::Transform tf_base_link2front(
    tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(base_link2front, 0.0, 0.0));
  tf2::Transform tf_map2base_link;
  tf2::fromMsg(target_pose, tf_map2base_link);
  tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
  geometry_msgs::msg::Pose virtual_wall_pose;
  tf2::toMsg(tf_map2front, virtual_wall_pose);
  return virtual_wall_pose;
}

visualization_msgs::msg::MarkerArray getDebugPointsMarkers(
  const std::vector<geometry_msgs::msg::Point> & interpolated_points,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points,
  const std::vector<geometry_msgs::msg::Point> & straight_points,
  const std::vector<geometry_msgs::msg::Pose> & fixed_points,
  const std::vector<geometry_msgs::msg::Pose> & non_fixed_points)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int unique_id = 0;

  unique_id = 0;
  visualization_msgs::msg::Marker interpolated_points_marker;
  interpolated_points_marker.lifetime = rclcpp::Duration::from_seconds(0);
  interpolated_points_marker.header.frame_id = "map";
  interpolated_points_marker.header.stamp = rclcpp::Time(0);
  interpolated_points_marker.ns = std::string("interpolated_points_marker");
  interpolated_points_marker.action = visualization_msgs::msg::Marker::ADD;
  interpolated_points_marker.pose.orientation.w = 1.0;
  interpolated_points_marker.id = unique_id;
  interpolated_points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  interpolated_points_marker.scale = createMarkerScale(.3, .3, .3);
  interpolated_points_marker.color = createMarkerColor(1.0f, 0, 0, 0.8);
  unique_id++;
  for (const auto & point : interpolated_points) {
    interpolated_points_marker.points.push_back(point);
  }
  if (!interpolated_points_marker.points.empty()) {
    marker_array.markers.push_back(interpolated_points_marker);
  }

  unique_id = 0;
  visualization_msgs::msg::Marker optimized_points_marker;
  optimized_points_marker.lifetime = rclcpp::Duration::from_seconds(0);
  optimized_points_marker.header.frame_id = "map";
  optimized_points_marker.header.stamp = rclcpp::Time(0);
  optimized_points_marker.ns = std::string("optimized_points_marker");
  optimized_points_marker.action = visualization_msgs::msg::Marker::ADD;
  optimized_points_marker.pose.orientation.w = 1.0;
  optimized_points_marker.id = unique_id;
  optimized_points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  optimized_points_marker.scale = createMarkerScale(0.35, 0.35, 0.35);
  optimized_points_marker.color = createMarkerColor(0, 1.0f, 0, 0.99);
  unique_id++;
  for (const auto & point : optimized_points) {
    optimized_points_marker.points.push_back(point.pose.position);
  }
  if (!optimized_points_marker.points.empty()) {
    marker_array.markers.push_back(optimized_points_marker);
  }

  unique_id = 0;
  for (std::size_t i = 0; i < optimized_points.size(); i++) {
    visualization_msgs::msg::Marker optimized_points_text_marker;
    optimized_points_text_marker.lifetime = rclcpp::Duration::from_seconds(0);
    optimized_points_text_marker.header.frame_id = "map";
    optimized_points_text_marker.header.stamp = rclcpp::Time(0);
    optimized_points_text_marker.ns = std::string("optimized_points_text_marker");
    optimized_points_text_marker.action = visualization_msgs::msg::Marker::ADD;
    optimized_points_text_marker.pose.orientation.w = 1.0;
    optimized_points_text_marker.id = unique_id;
    optimized_points_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    optimized_points_text_marker.pose.position = optimized_points[i].pose.position;
    optimized_points_text_marker.scale = createMarkerScale(0, 0, 0.15);
    optimized_points_text_marker.color = createMarkerColor(0, 1.0, 0, 0.99);
    optimized_points_text_marker.text = std::to_string(i);
    unique_id++;
    marker_array.markers.push_back(optimized_points_text_marker);
  }

  unique_id = 0;
  for (std::size_t i = 0; i < interpolated_points.size(); i++) {
    visualization_msgs::msg::Marker interpolated_points_text_marker;
    interpolated_points_text_marker.lifetime = rclcpp::Duration::from_seconds(0);
    interpolated_points_text_marker.header.frame_id = "map";
    interpolated_points_text_marker.header.stamp = rclcpp::Time(0);
    interpolated_points_text_marker.ns = std::string("interpolated_points_text_marker");
    interpolated_points_text_marker.action = visualization_msgs::msg::Marker::ADD;
    interpolated_points_text_marker.pose.orientation.w = 1.0;
    interpolated_points_text_marker.id = unique_id;
    interpolated_points_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    interpolated_points_text_marker.pose.position = interpolated_points[i];
    interpolated_points_text_marker.scale = createMarkerScale(0, 0, 0.5);
    interpolated_points_text_marker.color = createMarkerColor(0, 1.0, 0, 0.99);
    interpolated_points_text_marker.text = std::to_string(i);
    unique_id++;
    marker_array.markers.push_back(interpolated_points_text_marker);
  }

  unique_id = 0;
  visualization_msgs::msg::Marker straight_points_marker;
  straight_points_marker.lifetime = rclcpp::Duration::from_seconds(40);
  straight_points_marker.header.frame_id = "map";
  straight_points_marker.header.stamp = rclcpp::Time(0);
  straight_points_marker.ns = std::string("straight_points_marker");
  straight_points_marker.action = visualization_msgs::msg::Marker::ADD;
  straight_points_marker.pose.orientation.w = 1.0;
  straight_points_marker.id = unique_id;
  straight_points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  straight_points_marker.scale = createMarkerScale(1.0, 1.0, 1.0);
  straight_points_marker.color = createMarkerColor(1.0, 1.0, 0, 0.99);
  unique_id++;
  for (const auto & point : straight_points) {
    straight_points_marker.points.push_back(point);
  }
  if (!straight_points_marker.points.empty()) {
    marker_array.markers.push_back(straight_points_marker);
  }

  unique_id = 0;
  visualization_msgs::msg::Marker fixed_marker;
  fixed_marker.lifetime = rclcpp::Duration::from_seconds(0);
  fixed_marker.header.frame_id = "map";
  fixed_marker.header.stamp = rclcpp::Time(0);
  fixed_marker.ns = std::string("fixed_points_marker");
  fixed_marker.action = visualization_msgs::msg::Marker::ADD;
  fixed_marker.pose.orientation.w = 1.0;
  fixed_marker.id = unique_id;
  fixed_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  fixed_marker.scale = createMarkerScale(0.3, 0.3, 0.3);
  fixed_marker.color = createMarkerColor(1.0, 0, 0, 0.99);
  unique_id++;
  for (const auto & point : fixed_points) {
    fixed_marker.points.push_back(point.position);
  }
  if (!fixed_marker.points.empty()) {
    marker_array.markers.push_back(fixed_marker);
  }

  visualization_msgs::msg::Marker non_fixed_marker;
  non_fixed_marker.lifetime = rclcpp::Duration::from_seconds(20);
  non_fixed_marker.header.frame_id = "map";
  non_fixed_marker.header.stamp = rclcpp::Time(0);
  non_fixed_marker.ns = std::string("non_fixed_points_marker");
  non_fixed_marker.action = visualization_msgs::msg::Marker::ADD;
  non_fixed_marker.pose.orientation.w = 1.0;
  non_fixed_marker.id = unique_id;
  non_fixed_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  non_fixed_marker.scale = createMarkerScale(1.0, 1.0, 1.0);
  non_fixed_marker.color = createMarkerColor(0, 1.0, 0, 0.99);
  unique_id++;
  for (const auto & point : non_fixed_points) {
    non_fixed_marker.points.push_back(point.position);
  }
  if (!non_fixed_marker.points.empty()) {
    marker_array.markers.push_back(non_fixed_marker);
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray getDebugConstrainMarkers(
  const std::vector<ConstrainRectangle> & constrain_ranges, const std::string & ns)
{
  visualization_msgs::msg::MarkerArray marker_array;
  int unique_id = 0;
  for (std::size_t i = 0; i < constrain_ranges.size(); i++) {
    visualization_msgs::msg::Marker constrain_rect_marker;
    constrain_rect_marker.lifetime = rclcpp::Duration::from_seconds(0);
    constrain_rect_marker.header.frame_id = "map";
    constrain_rect_marker.header.stamp = rclcpp::Time(0);
    constrain_rect_marker.ns = ns;
    constrain_rect_marker.action = visualization_msgs::msg::Marker::ADD;
    constrain_rect_marker.pose.orientation.w = 1.0;
    constrain_rect_marker.id = unique_id;
    constrain_rect_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    constrain_rect_marker.scale = createMarkerScale(0.01, 0, 0);
    constrain_rect_marker.color = createMarkerColor(1.0, 0, 0, 0.99);
    unique_id++;
    geometry_msgs::msg::Point top_left_point = constrain_ranges[i].top_left;
    geometry_msgs::msg::Point top_right_point = constrain_ranges[i].top_right;
    geometry_msgs::msg::Point bottom_right_point = constrain_ranges[i].bottom_right;
    geometry_msgs::msg::Point bottom_left_point = constrain_ranges[i].bottom_left;
    constrain_rect_marker.points.push_back(top_left_point);
    constrain_rect_marker.points.push_back(top_right_point);
    constrain_rect_marker.points.push_back(bottom_right_point);
    constrain_rect_marker.points.push_back(bottom_left_point);
    constrain_rect_marker.points.push_back(top_left_point);
    marker_array.markers.push_back(constrain_rect_marker);
  }

  for (std::size_t i = 0; i < constrain_ranges.size(); i++) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = ns + "_text";
    marker.id = unique_id++;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale = createMarkerScale(0, 0, 0.15);
    marker.color = createMarkerColor(1.0, 0, 0, 0.99);
    marker.text = std::to_string(i);
    marker.pose.position = constrain_ranges[i].top_left;
    marker_array.markers.push_back(marker);
  }

  unique_id = 0;
  for (std::size_t i = 0; i < constrain_ranges.size(); i++) {
    visualization_msgs::msg::Marker constrain_range_text_marker;
    constrain_range_text_marker.lifetime = rclcpp::Duration::from_seconds(0);
    constrain_range_text_marker.header.frame_id = "map";
    constrain_range_text_marker.header.stamp = rclcpp::Time(0);
    constrain_range_text_marker.ns = ns + "_location";
    constrain_range_text_marker.action = visualization_msgs::msg::Marker::ADD;
    constrain_range_text_marker.pose.orientation.w = 1.0;
    constrain_range_text_marker.id = unique_id;
    constrain_range_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    constrain_range_text_marker.pose.position = constrain_ranges[i].top_left;
    constrain_range_text_marker.scale = createMarkerScale(0, 0, 0.1);
    constrain_range_text_marker.color = createMarkerColor(1.0, 0, 0, 0.99);
    constrain_range_text_marker.text = std::to_string(i) + std::string(" x ") +
                                       std::to_string(constrain_range_text_marker.pose.position.x) +
                                       std::string("y ") +
                                       std::to_string(constrain_range_text_marker.pose.position.y);
    unique_id++;
    marker_array.markers.push_back(constrain_range_text_marker);

    constrain_range_text_marker.id = unique_id;
    constrain_range_text_marker.pose.position = constrain_ranges[i].top_right;
    constrain_range_text_marker.text = std::to_string(i) + std::string(" x ") +
                                       std::to_string(constrain_range_text_marker.pose.position.x) +
                                       std::string("y ") +
                                       std::to_string(constrain_range_text_marker.pose.position.y);
    unique_id++;
    marker_array.markers.push_back(constrain_range_text_marker);

    constrain_range_text_marker.id = unique_id;
    constrain_range_text_marker.pose.position = constrain_ranges[i].bottom_left;
    constrain_range_text_marker.text = std::to_string(i) + std::string(" x ") +
                                       std::to_string(constrain_range_text_marker.pose.position.x) +
                                       std::string("y ") +
                                       std::to_string(constrain_range_text_marker.pose.position.y);
    unique_id++;
    marker_array.markers.push_back(constrain_range_text_marker);

    constrain_range_text_marker.id = unique_id;
    constrain_range_text_marker.pose.position = constrain_ranges[i].bottom_right;
    constrain_range_text_marker.text = std::to_string(i) + std::string(" x ") +
                                       std::to_string(constrain_range_text_marker.pose.position.x) +
                                       std::string("y ") +
                                       std::to_string(constrain_range_text_marker.pose.position.y);
    unique_id++;
    marker_array.markers.push_back(constrain_range_text_marker);
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray getObjectsMarkerArray(
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
  const std::string & ns, const double r, const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  int32_t i = 0;
  for (const auto & object : objects) {
    marker.id = i++;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    marker.scale = createMarkerScale(3.0, 1.0, 1.0);
    marker.color = createMarkerColor(r, g, b, 0.8);
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray getRectanglesMarkerArray(
  const std::vector<util::Footprint> & rects, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  int unique_id = 0;
  for (const auto & rect : rects) {
    marker.id = unique_id++;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale = createMarkerScale(0.05, 0, 0);
    marker.color = createMarkerColor(r, g, b, 0.025);
    marker.points.push_back(rect.top_left);
    marker.points.push_back(rect.top_right);
    marker.points.push_back(rect.bottom_right);
    marker.points.push_back(rect.bottom_left);
    msg.markers.push_back(marker);
  }
  return msg;
}

visualization_msgs::msg::MarkerArray getRectanglesNumMarkerArray(
  const std::vector<util::Footprint> & rects, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  int unique_id = 0;
  int number_of_rect = 0;
  for (const auto & rect : rects) {
    marker.id = unique_id++;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale = createMarkerScale(0, 0, 0.125);
    marker.color = createMarkerColor(r, g, b, 0.99);
    marker.text = std::to_string(number_of_rect);
    marker.pose.position = rect.top_left;
    msg.markers.push_back(marker);
    marker.id = unique_id++;
    marker.pose.position = rect.top_right;
    msg.markers.push_back(marker);
    marker.id = unique_id++;
    number_of_rect++;
  }
  return msg;
}

visualization_msgs::msg::MarkerArray getPointsMarkerArray(
  const std::vector<geometry_msgs::msg::Pose> & points, const std::string & ns, const double r,
  const double g, const double b)
{
  if (points.empty()) {
    return visualization_msgs::msg::MarkerArray{};
  }
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  const int unique_id = 0;
  marker.id = unique_id;
  marker.lifetime = rclcpp::Duration::from_seconds(0);
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.scale = createMarkerScale(0.5, 0.5, 0.5);
  marker.color = createMarkerColor(r, g, b, 0.99);
  for (const auto & p : points) {
    marker.points.push_back(p.position);
  }
  msg.markers.push_back(marker);
  return msg;
}

visualization_msgs::msg::MarkerArray getPointsMarkerArray(
  const std::vector<geometry_msgs::msg::Point> & points, const std::string & ns, const double r,
  const double g, const double b)
{
  if (points.empty()) {
    return visualization_msgs::msg::MarkerArray{};
  }
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  const int unique_id = 0;
  marker.id = unique_id;
  marker.lifetime = rclcpp::Duration::from_seconds(0);
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.scale = createMarkerScale(0.7, 0.7, 0.7);
  marker.color = createMarkerColor(r, g, b, 0.99);
  for (const auto & p : points) {
    marker.points.push_back(p);
  }
  msg.markers.push_back(marker);
  return msg;
}

visualization_msgs::msg::MarkerArray getPointsTextMarkerArray(
  const std::vector<geometry_msgs::msg::Pose> & points, const std::string & ns, const double r,
  const double g, const double b)
{
  if (points.empty()) {
    return visualization_msgs::msg::MarkerArray{};
  }
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  int unique_id = 0;
  for (std::size_t i = 0; i < points.size(); i++) {
    marker.id = unique_id++;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale = createMarkerScale(0, 0, 0.15);
    marker.color = createMarkerColor(r, g, b, 0.99);
    marker.text = std::to_string(i);
    // marker.text = std::to_string(i) + " "+ std::to_string(points[i].position.x)+ " "+
    //                                        std::to_string(points[i].position.y);
    marker.pose.position = points[i].position;
    msg.markers.push_back(marker);
  }
  return msg;
}

visualization_msgs::msg::MarkerArray getPointsTextMarkerArray(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const std::string & ns, const double r, const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  int unique_id = 0;
  for (std::size_t i = 0; i < points.size(); i++) {
    marker.id = unique_id++;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale = createMarkerScale(0, 0, 0.15);
    marker.color = createMarkerColor(r, g, b, 0.99);
    // marker.text = std::to_string(i) + " "+ std::to_string(points[i].pose.position.x)+ " "+
    //                                         std::to_string(points[i].pose.position.y);
    marker.text = std::to_string(i);
    marker.pose.position = points[i].pose.position;
    msg.markers.push_back(marker);
  }
  return msg;
}

visualization_msgs::msg::MarkerArray getBaseBoundsLineMarkerArray(
  const std::vector<Bounds> & bounds, const std::vector<geometry_msgs::msg::Pose> & candidate_base,
  const std::string & ns, const double r, const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  int unique_id = 0;
  for (std::size_t i = 0; i < bounds.size(); i++) {
    marker.id = unique_id++;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale = createMarkerScale(0.05, 0, 0);
    marker.color = createMarkerColor(r, g, b, 0.6);
    geometry_msgs::msg::Pose pose;
    pose = candidate_base[i];
    geometry_msgs::msg::Point rel_lb;
    rel_lb.x = 0;
    rel_lb.y = bounds[i].c0.lb;
    geometry_msgs::msg::Point abs_lb = util::transformToAbsoluteCoordinate2D(rel_lb, pose);
    geometry_msgs::msg::Point rel_ub;
    rel_ub.x = 0;
    rel_ub.y = bounds[i].c0.ub;
    geometry_msgs::msg::Point abs_ub = util::transformToAbsoluteCoordinate2D(rel_ub, pose);
    marker.points.push_back(abs_lb);
    marker.points.push_back(abs_ub);
    msg.markers.push_back(marker);
    marker.points.clear();
  }
  return msg;
}

visualization_msgs::msg::MarkerArray getTopBoundsLineMarkerArray(
  const std::vector<Bounds> & bounds, const std::vector<geometry_msgs::msg::Pose> & candidate_top,
  const std::string & ns, const double r, const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  int unique_id = 0;
  for (std::size_t i = 0; i < bounds.size(); i++) {
    marker.id = unique_id++;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale = createMarkerScale(0.05, 0, 0);
    marker.color = createMarkerColor(r, g, b, 0.6);
    geometry_msgs::msg::Point rel_lb;
    rel_lb.x = 0;
    rel_lb.y = bounds[i].c1.lb;
    geometry_msgs::msg::Point abs_lb =
      util::transformToAbsoluteCoordinate2D(rel_lb, candidate_top[i]);
    geometry_msgs::msg::Point rel_ub;
    rel_ub.x = 0;
    rel_ub.y = bounds[i].c1.ub;
    geometry_msgs::msg::Point abs_ub =
      util::transformToAbsoluteCoordinate2D(rel_ub, candidate_top[i]);
    marker.points.push_back(abs_lb);
    marker.points.push_back(abs_ub);
    msg.markers.push_back(marker);
    marker.points.clear();
  }
  return msg;
}

visualization_msgs::msg::MarkerArray getMidBoundsLineMarkerArray(
  const std::vector<Bounds> & bounds, const std::vector<geometry_msgs::msg::Pose> & candidate_top,
  const std::string & ns, const double r, const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  int unique_id = 0;
  for (std::size_t i = 0; i < bounds.size(); i++) {
    marker.id = unique_id++;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale = createMarkerScale(0.05, 0, 0);
    marker.color = createMarkerColor(r, g, b, 0.6);
    geometry_msgs::msg::Point rel_lb;
    rel_lb.x = 0;
    rel_lb.y = bounds[i].c2.lb;
    geometry_msgs::msg::Point abs_lb =
      util::transformToAbsoluteCoordinate2D(rel_lb, candidate_top[i]);
    geometry_msgs::msg::Point rel_ub;
    rel_ub.x = 0;
    rel_ub.y = bounds[i].c2.ub;
    geometry_msgs::msg::Point abs_ub =
      util::transformToAbsoluteCoordinate2D(rel_ub, candidate_top[i]);
    marker.points.push_back(abs_lb);
    marker.points.push_back(abs_ub);
    msg.markers.push_back(marker);
    marker.points.clear();
  }
  return msg;
}

visualization_msgs::msg::MarkerArray getVirtualWallMarkerArray(
  const geometry_msgs::msg::Pose & pose, const std::string & ns, const double r, const double g,
  const double b)
{
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker;
  marker.id = 0;
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose;
  marker.scale = createMarkerScale(0.1, 5.0, 2.0);
  marker.color = createMarkerColor(r, g, b, 0.5);
  msg.markers.push_back(marker);
  return msg;
}

visualization_msgs::msg::MarkerArray getVirtualWallTextMarkerArray(
  const geometry_msgs::msg::Pose & pose, const std::string & ns, const double r, const double g,
  const double b)
{
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker;
  marker.id = 0;
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose;
  marker.text = "drivable area";
  marker.scale = createMarkerScale(0.0, 0.0, 1.0);
  marker.color = createMarkerColor(r, g, b, 0.99);
  msg.markers.push_back(marker);
  return msg;
}

nav_msgs::msg::OccupancyGrid getDebugCostmap(
  const cv::Mat & clearance_map, const nav_msgs::msg::OccupancyGrid & occupancy_grid)
{
  cv::Mat tmp;
  clearance_map.copyTo(tmp);
  cv::normalize(tmp, tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  nav_msgs::msg::OccupancyGrid clearance_map_in_og = occupancy_grid;
  tmp.forEach<unsigned char>([&](const unsigned char & value, const int * position) -> void {
    process_cv::putOccupancyGridValue(clearance_map_in_og, position[0], position[1], value);
  });
  return clearance_map_in_og;
}
