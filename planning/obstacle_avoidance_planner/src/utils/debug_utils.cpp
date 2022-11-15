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

#include "obstacle_avoidance_planner/utils/debug_utils.hpp"

#include "obstacle_avoidance_planner/mpt_optimizer.hpp"
#include "obstacle_avoidance_planner/utils/cv_utils.hpp"
#include "obstacle_avoidance_planner/utils/utils.hpp"
#include "tf2/utils.h"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <string>
#include <vector>

using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;

namespace
{
template <typename T>
visualization_msgs::msg::MarkerArray getPointsMarkerArray(
  const std::vector<T> & points, const std::string & ns, const double r, const double g,
  const double b)
{
  if (points.empty()) {
    return visualization_msgs::msg::MarkerArray{};
  }

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, visualization_msgs::msg::Marker::LINE_LIST,
    createMarkerScale(0.5, 0.5, 0.5), createMarkerColor(r, g, b, 0.99));
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  for (const auto & point : points) {
    marker.points.push_back(tier4_autoware_utils::getPoint(point));
  }

  visualization_msgs::msg::MarkerArray msg;
  msg.markers.push_back(marker);

  return msg;
}

template <typename T>
visualization_msgs::msg::MarkerArray getPointsTextMarkerArray(
  const std::vector<T> & points, const std::string & ns, const double r, const double g,
  const double b)
{
  if (points.empty()) {
    return visualization_msgs::msg::MarkerArray{};
  }

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.0, 0.0, 0.15), createMarkerColor(r, g, b, 0.99));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  visualization_msgs::msg::MarkerArray msg;
  for (size_t i = 0; i < points.size(); i++) {
    marker.id = i;
    marker.text = std::to_string(i);
    marker.pose.position = tier4_autoware_utils::getPoint(points[i]);
    msg.markers.push_back(marker);
  }

  return msg;
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
  for (size_t i = 0; i < optimized_points.size(); i++) {
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
  for (size_t i = 0; i < interpolated_points.size(); i++) {
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
  for (size_t i = 0; i < constrain_ranges.size(); i++) {
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

  for (size_t i = 0; i < constrain_ranges.size(); i++) {
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
  for (size_t i = 0; i < constrain_ranges.size(); i++) {
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
  visualization_msgs::msg::MarkerArray msg;

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, visualization_msgs::msg::Marker::CUBE,
    createMarkerScale(3.0, 1.0, 1.0), createMarkerColor(r, g, b, 0.8));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  for (size_t i = 0; i < objects.size(); ++i) {
    const auto & object = objects.at(i);
    marker.id = i;
    marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray getRectanglesMarkerArray(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> mpt_traj,
  const VehicleParam & vehicle_param, const std::string & ns, const double r, const double g,
  const double b, const size_t sampling_num)
{
  visualization_msgs::msg::MarkerArray msg;
  for (size_t i = 0; i < mpt_traj.size(); ++i) {
    if (i % sampling_num != 0) {
      continue;
    }
    const auto & traj_point = mpt_traj.at(i);

    auto marker = createDefaultMarker(
      "map", rclcpp::Clock().now(), ns, i, visualization_msgs::msg::Marker::LINE_STRIP,
      createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(r, g, b, 1.0));
    marker.lifetime = rclcpp::Duration::from_seconds(1.5);

    const double base_to_right = (vehicle_param.wheel_tread / 2.0) + vehicle_param.right_overhang;
    const double base_to_left = (vehicle_param.wheel_tread / 2.0) + vehicle_param.left_overhang;
    const double base_to_front = vehicle_param.length - vehicle_param.rear_overhang;
    const double base_to_rear = vehicle_param.rear_overhang;

    marker.points.push_back(
      tier4_autoware_utils::calcOffsetPose(traj_point.pose, base_to_front, -base_to_right, 0.0)
        .position);
    marker.points.push_back(
      tier4_autoware_utils::calcOffsetPose(traj_point.pose, base_to_front, base_to_left, 0.0)
        .position);
    marker.points.push_back(
      tier4_autoware_utils::calcOffsetPose(traj_point.pose, -base_to_rear, base_to_left, 0.0)
        .position);
    marker.points.push_back(
      tier4_autoware_utils::calcOffsetPose(traj_point.pose, -base_to_rear, -base_to_right, 0.0)
        .position);
    marker.points.push_back(marker.points.front());

    msg.markers.push_back(marker);
  }
  return msg;
}

visualization_msgs::msg::MarkerArray getRectanglesNumMarkerArray(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> mpt_traj,
  const VehicleParam & vehicle_param, const std::string & ns, const double r, const double g,
  const double b)
{
  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.0, 0.0, 0.125), createMarkerColor(r, g, b, 0.99));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  visualization_msgs::msg::MarkerArray msg;
  for (size_t i = 0; i < mpt_traj.size(); ++i) {
    const auto & traj_point = mpt_traj.at(i);

    marker.text = std::to_string(i);

    const double half_width = vehicle_param.width / 2.0;
    const double base_to_front = vehicle_param.length - vehicle_param.rear_overhang;

    const auto top_right_pos =
      tier4_autoware_utils::calcOffsetPose(traj_point.pose, base_to_front, half_width, 0.0)
        .position;
    marker.id = i;
    marker.pose.position = top_right_pos;
    msg.markers.push_back(marker);

    marker.id = i + mpt_traj.size();
    marker.pose.position = top_right_pos;
    msg.markers.push_back(marker);
  }
  return msg;
}

visualization_msgs::msg::MarkerArray getBoundsCandidatesLineMarkerArray(
  const std::vector<ReferencePoint> & ref_points,
  std::vector<std::vector<Bounds>> & bounds_candidates, const double r, const double g,
  const double b, [[maybe_unused]] const double vehicle_width, const size_t sampling_num)
{
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;
  const std::string ns = "bounds_candidates";

  if (ref_points.empty()) return msg;

  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, visualization_msgs::msg::Marker::LINE_LIST,
    createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(r + 0.5, g, b, 0.3));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  for (size_t i = 0; i < ref_points.size(); i++) {
    if (i % sampling_num != 0) {
      continue;
    }

    const auto & bound_candidates = bounds_candidates.at(i);
    for (size_t j = 0; j < bound_candidates.size(); ++j) {
      geometry_msgs::msg::Pose pose;
      pose.position = ref_points.at(i).p;
      pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(ref_points.at(i).yaw);

      // lower bound
      const double lb_y = bound_candidates.at(j).lower_bound;
      const auto lb = tier4_autoware_utils::calcOffsetPose(pose, 0.0, lb_y, 0.0).position;
      marker.points.push_back(lb);

      // upper bound
      const double ub_y = bound_candidates.at(j).upper_bound;
      const auto ub = tier4_autoware_utils::calcOffsetPose(pose, 0.0, ub_y, 0.0).position;
      marker.points.push_back(ub);

      msg.markers.push_back(marker);
    }
  }

  return msg;
}

visualization_msgs::msg::MarkerArray getBoundsLineMarkerArray(
  const std::vector<ReferencePoint> & ref_points, const double r, const double g, const double b,
  const double vehicle_width, const size_t sampling_num)
{
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  if (ref_points.empty()) return msg;

  for (size_t bound_idx = 0; bound_idx < ref_points.at(0).vehicle_bounds.size(); ++bound_idx) {
    const std::string ns = "base_bounds_" + std::to_string(bound_idx);

    {  // lower bound
      auto marker = createDefaultMarker(
        "map", rclcpp::Clock().now(), ns, 0, visualization_msgs::msg::Marker::LINE_LIST,
        createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(r + 0.5, g, b, 0.3));
      marker.lifetime = rclcpp::Duration::from_seconds(1.5);

      for (size_t i = 0; i < ref_points.size(); i++) {
        if (i % sampling_num != 0) {
          continue;
        }

        const geometry_msgs::msg::Pose & pose = ref_points.at(i).vehicle_bounds_poses.at(bound_idx);
        const double lb_y =
          ref_points.at(i).vehicle_bounds[bound_idx].lower_bound - vehicle_width / 2.0;
        const auto lb = tier4_autoware_utils::calcOffsetPose(pose, 0.0, lb_y, 0.0).position;

        marker.points.push_back(pose.position);
        marker.points.push_back(lb);
      }
      msg.markers.push_back(marker);
    }

    {  // upper bound
      auto marker = createDefaultMarker(
        "map", rclcpp::Clock().now(), ns, 1, visualization_msgs::msg::Marker::LINE_LIST,
        createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(r, g + 0.5, b, 0.3));
      marker.lifetime = rclcpp::Duration::from_seconds(1.5);

      for (size_t i = 0; i < ref_points.size(); i++) {
        if (i % sampling_num != 0) {
          continue;
        }

        const geometry_msgs::msg::Pose & pose = ref_points.at(i).vehicle_bounds_poses.at(bound_idx);
        const double ub_y =
          ref_points.at(i).vehicle_bounds[bound_idx].upper_bound + vehicle_width / 2.0;
        const auto ub = tier4_autoware_utils::calcOffsetPose(pose, 0.0, ub_y, 0.0).position;

        marker.points.push_back(pose.position);
        marker.points.push_back(ub);
      }
      msg.markers.push_back(marker);
    }
  }

  return msg;
}

visualization_msgs::msg::MarkerArray getVehicleCircleLineMarkerArray(
  const std::vector<std::vector<geometry_msgs::msg::Pose>> & vehicle_circles_pose,
  const double vehicle_width, const size_t sampling_num, const std::string & ns, const double r,
  const double g, const double b)
{
  const auto current_time = rclcpp::Clock().now();
  visualization_msgs::msg::MarkerArray msg;

  for (size_t i = 0; i < vehicle_circles_pose.size(); ++i) {
    if (i % sampling_num != 0) {
      continue;
    }

    auto marker = createDefaultMarker(
      "map", rclcpp::Clock().now(), ns, i, visualization_msgs::msg::Marker::LINE_LIST,
      createMarkerScale(0.1, 0, 0), createMarkerColor(r, g, b, 0.25));
    marker.lifetime = rclcpp::Duration::from_seconds(1.5);

    for (size_t j = 0; j < vehicle_circles_pose.at(i).size(); ++j) {
      const geometry_msgs::msg::Pose & pose = vehicle_circles_pose.at(i).at(j);
      const auto ub =
        tier4_autoware_utils::calcOffsetPose(pose, 0.0, vehicle_width / 2.0, 0.0).position;
      const auto lb =
        tier4_autoware_utils::calcOffsetPose(pose, 0.0, -vehicle_width / 2.0, 0.0).position;

      marker.points.push_back(ub);
      marker.points.push_back(lb);
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray getLateralErrorsLineMarkerArray(
  const std::vector<geometry_msgs::msg::Pose> mpt_ref_poses, std::vector<double> lateral_errors,
  const size_t sampling_num, const std::string & ns, const double r, const double g, const double b)
{
  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, visualization_msgs::msg::Marker::LINE_LIST,
    createMarkerScale(0.1, 0, 0), createMarkerColor(r, g, b, 1.0));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  for (size_t i = 0; i < mpt_ref_poses.size(); ++i) {
    if (i % sampling_num != 0) {
      continue;
    }

    const auto vehicle_pose =
      tier4_autoware_utils::calcOffsetPose(mpt_ref_poses.at(i), 0.0, lateral_errors.at(i), 0.0);
    marker.points.push_back(mpt_ref_poses.at(i).position);
    marker.points.push_back(vehicle_pose.position);
  }

  visualization_msgs::msg::MarkerArray msg;
  msg.markers.push_back(marker);

  return msg;
}

visualization_msgs::msg::MarkerArray getCurrentVehicleCirclesMarkerArray(
  const geometry_msgs::msg::Pose & current_ego_pose,
  const std::vector<double> & vehicle_circle_longitudinal_offsets,
  const std::vector<double> & vehicle_circle_radiuses, const std::string & ns, const double r,
  const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  size_t id = 0;
  for (size_t v_idx = 0; v_idx < vehicle_circle_longitudinal_offsets.size(); ++v_idx) {
    const double offset = vehicle_circle_longitudinal_offsets.at(v_idx);

    auto marker = createDefaultMarker(
      "map", rclcpp::Clock().now(), ns, id, visualization_msgs::msg::Marker::LINE_STRIP,
      createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(r, g, b, 0.8));
    marker.lifetime = rclcpp::Duration::from_seconds(1.5);
    marker.pose = tier4_autoware_utils::calcOffsetPose(current_ego_pose, offset, 0.0, 0.0);

    constexpr size_t circle_dividing_num = 16;
    for (size_t e_idx = 0; e_idx < circle_dividing_num + 1; ++e_idx) {
      geometry_msgs::msg::Point edge_pos;

      const double edge_angle =
        static_cast<double>(e_idx) / static_cast<double>(circle_dividing_num) * 2.0 * M_PI;
      edge_pos.x = vehicle_circle_radiuses.at(v_idx) * std::cos(edge_angle);
      edge_pos.y = vehicle_circle_radiuses.at(v_idx) * std::sin(edge_angle);

      marker.points.push_back(edge_pos);
    }

    msg.markers.push_back(marker);
    ++id;
  }
  return msg;
}

visualization_msgs::msg::MarkerArray getVehicleCirclesMarkerArray(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & mpt_traj_points,
  const std::vector<double> & vehicle_circle_longitudinal_offsets,
  const std::vector<double> & vehicle_circle_radiuses, const size_t sampling_num,
  const std::string & ns, const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  size_t id = 0;
  for (size_t i = 0; i < mpt_traj_points.size(); ++i) {
    if (i % sampling_num != 0) {
      continue;
    }
    const auto & mpt_traj_point = mpt_traj_points.at(i);

    for (size_t v_idx = 0; v_idx < vehicle_circle_longitudinal_offsets.size(); ++v_idx) {
      const double offset = vehicle_circle_longitudinal_offsets.at(v_idx);

      auto marker = createDefaultMarker(
        "map", rclcpp::Clock().now(), ns, id, visualization_msgs::msg::Marker::LINE_STRIP,
        createMarkerScale(0.05, 0.0, 0.0), createMarkerColor(r, g, b, 0.8));
      marker.lifetime = rclcpp::Duration::from_seconds(1.5);
      marker.pose = tier4_autoware_utils::calcOffsetPose(mpt_traj_point.pose, offset, 0.0, 0.0);

      constexpr size_t circle_dividing_num = 16;
      for (size_t e_idx = 0; e_idx < circle_dividing_num + 1; ++e_idx) {
        geometry_msgs::msg::Point edge_pos;

        const double edge_angle =
          static_cast<double>(e_idx) / static_cast<double>(circle_dividing_num) * 2.0 * M_PI;
        edge_pos.x = vehicle_circle_radiuses.at(v_idx) * std::cos(edge_angle);
        edge_pos.y = vehicle_circle_radiuses.at(v_idx) * std::sin(edge_angle);

        marker.points.push_back(edge_pos);
      }

      msg.markers.push_back(marker);
      ++id;
    }
  }
  return msg;
}

visualization_msgs::msg::MarkerArray getVirtualWallMarkerArray(
  const geometry_msgs::msg::Pose & pose, const std::string & ns, const double r, const double g,
  const double b)
{
  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, visualization_msgs::msg::Marker::CUBE,
    createMarkerScale(0.1, 5.0, 2.0), createMarkerColor(r, g, b, 0.5));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);
  marker.pose = pose;
  marker.pose.position.z += 1.0;

  visualization_msgs::msg::MarkerArray msg;
  msg.markers.push_back(marker);

  return msg;
}

visualization_msgs::msg::MarkerArray getVirtualWallTextMarkerArray(
  const geometry_msgs::msg::Pose & pose, const std::string & ns, const double r, const double g,
  const double b)
{
  auto marker = createDefaultMarker(
    "map", rclcpp::Clock().now(), ns, 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.0, 0.0, 1.0), createMarkerColor(r, g, b, 0.99));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);
  marker.pose = pose;
  marker.pose.position.z += 2.0;
  marker.text = "drivable area";

  visualization_msgs::msg::MarkerArray msg;
  msg.markers.push_back(marker);

  return msg;
}
}  // namespace

namespace debug_utils
{
visualization_msgs::msg::MarkerArray getDebugVisualizationMarker(
  DebugData & debug_data,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points,
  const VehicleParam & vehicle_param, const bool is_showing_debug_detail)
{
  visualization_msgs::msg::MarkerArray vis_marker_array;

  if (is_showing_debug_detail) {
    const auto points_marker_array = getDebugPointsMarkers(
      debug_data.interpolated_points, optimized_points, debug_data.straight_points,
      debug_data.fixed_points, debug_data.non_fixed_points);

    const auto constrain_marker_array =
      getDebugConstrainMarkers(debug_data.constrain_rectangles, "constrain_rect");

    appendMarkerArray(points_marker_array, &vis_marker_array);
    appendMarkerArray(constrain_marker_array, &vis_marker_array);

    appendMarkerArray(
      getRectanglesNumMarkerArray(
        debug_data.mpt_traj, vehicle_param, "num_vehicle_footprint", 0.99, 0.99, 0.2),
      &vis_marker_array);

    appendMarkerArray(
      getPointsTextMarkerArray(debug_data.eb_traj, "eb_traj_text", 0.99, 0.99, 0.2),
      &vis_marker_array);
  }

  // avoiding objects
  appendMarkerArray(
    getObjectsMarkerArray(debug_data.avoiding_objects, "avoiding_objects", 0.99, 0.99, 0.2),
    &vis_marker_array);
  // mpt footprints
  appendMarkerArray(
    getRectanglesMarkerArray(
      debug_data.mpt_traj, vehicle_param, "mpt_footprints", 0.99, 0.99, 0.2,
      debug_data.mpt_visualize_sampling_num),
    &vis_marker_array);
  // bounds
  appendMarkerArray(
    getBoundsLineMarkerArray(
      debug_data.ref_points, 0.99, 0.99, 0.2, vehicle_param.width,
      debug_data.mpt_visualize_sampling_num),
    &vis_marker_array);

  // bounds candidates
  appendMarkerArray(
    getBoundsCandidatesLineMarkerArray(
      debug_data.ref_points, debug_data.sequential_bounds_candidates, 0.2, 0.99, 0.99,
      vehicle_param.width, debug_data.mpt_visualize_sampling_num),
    &vis_marker_array);

  // vehicle circle line
  appendMarkerArray(
    getVehicleCircleLineMarkerArray(
      debug_data.vehicle_circles_pose, vehicle_param.width, debug_data.mpt_visualize_sampling_num,
      "vehicle_circle_lines", 0.99, 0.99, 0.2),
    &vis_marker_array);

  // lateral error line
  appendMarkerArray(
    getLateralErrorsLineMarkerArray(
      debug_data.mpt_ref_poses, debug_data.lateral_errors, debug_data.mpt_visualize_sampling_num,
      "lateral_errors", 0.1, 0.1, 0.8),
    &vis_marker_array);

  // current vehicle circles
  appendMarkerArray(
    getCurrentVehicleCirclesMarkerArray(
      debug_data.current_ego_pose, debug_data.vehicle_circle_longitudinal_offsets,
      debug_data.vehicle_circle_radiuses, "current_vehicle_circles", 1.0, 0.3, 0.3),
    &vis_marker_array);

  // vehicle circles
  appendMarkerArray(
    getVehicleCirclesMarkerArray(
      debug_data.mpt_traj, debug_data.vehicle_circle_longitudinal_offsets,
      debug_data.vehicle_circle_radiuses, debug_data.mpt_visualize_sampling_num, "vehicle_circles",
      1.0, 0.3, 0.3),
    &vis_marker_array);

  return vis_marker_array;
}

visualization_msgs::msg::MarkerArray getDebugVisualizationWallMarker(
  DebugData & debug_data, const VehicleParam & vehicle_param)
{
  visualization_msgs::msg::MarkerArray vis_marker_array;
  if (debug_data.stop_pose_by_drivable_area) {
    const auto virtual_wall_pose =
      getVirtualWallPose(debug_data.stop_pose_by_drivable_area.get(), vehicle_param);
    appendMarkerArray(
      getVirtualWallMarkerArray(virtual_wall_pose, "virtual_wall", 1.0, 0, 0), &vis_marker_array);
    appendMarkerArray(
      getVirtualWallTextMarkerArray(virtual_wall_pose, "virtual_wall_text", 1.0, 1.0, 1.0),
      &vis_marker_array);
  }
  return vis_marker_array;
}

nav_msgs::msg::OccupancyGrid getDebugCostmap(
  const cv::Mat & clearance_map, const nav_msgs::msg::OccupancyGrid & occupancy_grid)
{
  if (clearance_map.empty()) return nav_msgs::msg::OccupancyGrid();

  cv::Mat tmp;
  clearance_map.copyTo(tmp);
  cv::normalize(tmp, tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  nav_msgs::msg::OccupancyGrid clearance_map_in_og = occupancy_grid;
  tmp.forEach<unsigned char>([&](const unsigned char & value, const int * position) -> void {
    cv_utils::putOccupancyGridValue(clearance_map_in_og, position[0], position[1], value);
  });
  return clearance_map_in_og;
}
}  // namespace debug_utils
