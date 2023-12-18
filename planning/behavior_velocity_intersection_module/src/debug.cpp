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

#include "scene_intersection.hpp"
#include "scene_merge_from_private_road.hpp"

#include <behavior_velocity_planner_common/utilization/debug.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <string>
#include <vector>

namespace behavior_velocity_planner
{
namespace
{
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerOrientation;
using tier4_autoware_utils::createMarkerScale;

static visualization_msgs::msg::MarkerArray createLaneletPolygonsMarkerArray(
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  int32_t i = 0;
  int32_t uid = planning_utils::bitShift(lane_id);
  for (const auto & polygon : polygons) {
    visualization_msgs::msg::Marker marker{};
    marker.header.frame_id = "map";

    marker.ns = ns;
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = createMarkerScale(0.1, 0.0, 0.0);
    marker.color = createMarkerColor(r, g, b, 0.999);
    for (const auto & p : polygon) {
      geometry_msgs::msg::Point point;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray createPoseMarkerArray(
  const geometry_msgs::msg::Pose & pose, const std::string & ns, const int64_t id, const double r,
  const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker_line{};
  marker_line.header.frame_id = "map";
  marker_line.ns = ns + "_line";
  marker_line.id = id;
  marker_line.lifetime = rclcpp::Duration::from_seconds(0.3);
  marker_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_line.action = visualization_msgs::msg::Marker::ADD;
  marker_line.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
  marker_line.scale = createMarkerScale(0.1, 0.0, 0.0);
  marker_line.color = createMarkerColor(r, g, b, 0.999);

  const double yaw = tf2::getYaw(pose.orientation);

  const double a = 3.0;
  geometry_msgs::msg::Point p0;
  p0.x = pose.position.x - a * std::sin(yaw);
  p0.y = pose.position.y + a * std::cos(yaw);
  p0.z = pose.position.z;
  marker_line.points.push_back(p0);

  geometry_msgs::msg::Point p1;
  p1.x = pose.position.x + a * std::sin(yaw);
  p1.y = pose.position.y - a * std::cos(yaw);
  p1.z = pose.position.z;
  marker_line.points.push_back(p1);

  msg.markers.push_back(marker_line);

  return msg;
}

visualization_msgs::msg::MarkerArray createLineMarkerArray(
  const geometry_msgs::msg::Point & point_start, const geometry_msgs::msg::Point & point_end,
  const std::string & ns, const int64_t id, const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = ns + "_line";
  marker.id = id;
  marker.lifetime = rclcpp::Duration::from_seconds(0.3);
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  geometry_msgs::msg::Vector3 arrow;
  arrow.x = 1.0;
  arrow.y = 1.0;
  arrow.z = 1.0;
  marker.scale = arrow;
  marker.color = createMarkerColor(r, g, b, 0.999);
  marker.points.push_back(point_start);
  marker.points.push_back(point_end);

  msg.markers.push_back(marker);
  return msg;
}

[[maybe_unused]] visualization_msgs::msg::Marker createPointMarkerArray(
  const geometry_msgs::msg::Point & point, const std::string & ns, const int64_t id, const double r,
  const double g, const double b)
{
  visualization_msgs::msg::Marker marker_point{};
  marker_point.header.frame_id = "map";
  marker_point.ns = ns + "_point";
  marker_point.id = id;
  marker_point.lifetime = rclcpp::Duration::from_seconds(0.3);
  marker_point.type = visualization_msgs::msg::Marker::SPHERE;
  marker_point.action = visualization_msgs::msg::Marker::ADD;
  marker_point.scale = createMarkerScale(2.0, 2.0, 2.0);
  marker_point.color = createMarkerColor(r, g, b, 0.999);

  marker_point.pose.position = point;

  return marker_point;
}

}  // namespace

visualization_msgs::msg::MarkerArray IntersectionModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  const auto now = this->clock_->now();

  if (debug_data_.attention_area) {
    appendMarkerArray(
      createLaneletPolygonsMarkerArray(
        debug_data_.attention_area.value(), "attention_area", lane_id_, 0.0, 1.0, 0.0),
      &debug_marker_array);
  }

  if (debug_data_.occlusion_attention_area) {
    appendMarkerArray(
      createLaneletPolygonsMarkerArray(
        debug_data_.occlusion_attention_area.value(), "occlusion_attention_area", lane_id_, 0.917,
        0.568, 0.596),
      &debug_marker_array);
  }

  if (debug_data_.adjacent_area) {
    appendMarkerArray(
      createLaneletPolygonsMarkerArray(
        debug_data_.adjacent_area.value(), "adjacent_area", lane_id_, 0.913, 0.639, 0.149),
      &debug_marker_array);
  }

  if (debug_data_.stuck_vehicle_detect_area) {
    appendMarkerArray(
      debug::createPolygonMarkerArray(
        debug_data_.stuck_vehicle_detect_area.value(), "stuck_vehicle_detect_area", lane_id_, now,
        0.3, 0.0, 0.0, 0.0, 0.5, 0.5),
      &debug_marker_array, now);
  }

  if (debug_data_.yield_stuck_detect_area) {
    appendMarkerArray(
      createLaneletPolygonsMarkerArray(
        debug_data_.yield_stuck_detect_area.value(), "yield_stuck_detect_area", lane_id_, 0.6588235,
        0.34509, 0.6588235),
      &debug_marker_array);
  }

  if (debug_data_.ego_lane) {
    appendMarkerArray(
      createLaneletPolygonsMarkerArray(
        {debug_data_.ego_lane.value()}, "ego_lane", lane_id_, 1, 0.647, 0.0),
      &debug_marker_array, now);
  }

  if (debug_data_.candidate_collision_ego_lane_polygon) {
    appendMarkerArray(
      debug::createPolygonMarkerArray(
        debug_data_.candidate_collision_ego_lane_polygon.value(),
        "candidate_collision_ego_lane_polygon", module_id_, now, 0.3, 0.0, 0.0, 0.5, 0.0, 0.0),
      &debug_marker_array, now);
  }

  size_t i{0};
  for (const auto & p : debug_data_.candidate_collision_object_polygons) {
    appendMarkerArray(
      debug::createPolygonMarkerArray(
        p, "candidate_collision_object_polygons", lane_id_ + i++, now, 0.3, 0.0, 0.0, 0.0, 0.5,
        0.5),
      &debug_marker_array, now);
  }

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.conflicting_targets, "conflicting_targets", module_id_, now, 0.99, 0.4, 0.0),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.amber_ignore_targets, "amber_ignore_targets", module_id_, now, 0.0, 1.0, 0.0),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.red_overshoot_ignore_targets, "red_overshoot_ignore_targets", module_id_, now,
      0.0, 1.0, 0.0),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.stuck_targets, "stuck_targets", module_id_, now, 0.99, 0.99, 0.2),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.yield_stuck_targets, "stuck_targets", module_id_, now, 0.4, 0.99, 0.2),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.blocking_attention_objects, "blocking_attention_objects", module_id_, now, 0.99,
      0.99, 0.6),
    &debug_marker_array, now);

  /*
  appendMarkerArray(
    createPoseMarkerArray(
      debug_data_.predicted_obj_pose, "predicted_obj_pose", module_id_, 0.7, 0.85, 0.9),
    &debug_marker_array, now);
  */

  if (debug_data_.pass_judge_wall_pose) {
    appendMarkerArray(
      createPoseMarkerArray(
        debug_data_.pass_judge_wall_pose.value(), "pass_judge_wall_pose", module_id_, 0.7, 0.85,
        0.9),
      &debug_marker_array, now);
  }

  for (size_t j = 0; j < debug_data_.occlusion_polygons.size(); ++j) {
    const auto & p = debug_data_.occlusion_polygons.at(j);
    appendMarkerArray(
      debug::createPolygonMarkerArray(
        p, "occlusion_polygons", lane_id_ + j, now, 0.3, 0.0, 0.0, 1.0, 0.0, 0.0),
      &debug_marker_array, now);
  }

  if (debug_data_.nearest_occlusion_projection) {
    const auto [point_start, point_end] = debug_data_.nearest_occlusion_projection.value();
    appendMarkerArray(
      createLineMarkerArray(
        point_start, point_end, "nearest_occlusion_projection", lane_id_, 0.5, 0.5, 0.0),
      &debug_marker_array, now);
  }
  return debug_marker_array;
}

motion_utils::VirtualWalls IntersectionModule::createVirtualWalls()
{
  motion_utils::VirtualWalls virtual_walls;
  motion_utils::VirtualWall wall;

  if (debug_data_.collision_stop_wall_pose) {
    wall.style = motion_utils::VirtualWallType::stop;
    wall.text = "intersection";
    wall.ns = "intersection" + std::to_string(module_id_) + "_";
    wall.pose = debug_data_.collision_stop_wall_pose.value();
    virtual_walls.push_back(wall);
  }
  if (debug_data_.occlusion_first_stop_wall_pose) {
    wall.style = motion_utils::VirtualWallType::stop;
    wall.text = "intersection";
    wall.ns = "intersection_occlusion_first_stop" + std::to_string(module_id_) + "_";
    wall.pose = debug_data_.occlusion_first_stop_wall_pose.value();
    virtual_walls.push_back(wall);
  }
  if (debug_data_.occlusion_stop_wall_pose) {
    wall.style = motion_utils::VirtualWallType::stop;
    wall.text = "intersection_occlusion";
    if (debug_data_.static_occlusion_with_traffic_light_timeout) {
      std::stringstream timeout;
      timeout << std::setprecision(2)
              << debug_data_.static_occlusion_with_traffic_light_timeout.value();
      wall.text += "(" + timeout.str() + ")";
    }
    wall.ns = "intersection_occlusion" + std::to_string(module_id_) + "_";
    wall.pose = debug_data_.occlusion_stop_wall_pose.value();
    virtual_walls.push_back(wall);
  }
  if (debug_data_.absence_traffic_light_creep_wall) {
    wall.style = motion_utils::VirtualWallType::slowdown;
    wall.text = "intersection_occlusion";
    wall.ns = "intersection_occlusion" + std::to_string(module_id_) + "_";
    wall.pose = debug_data_.absence_traffic_light_creep_wall.value();
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

visualization_msgs::msg::MarkerArray MergeFromPrivateRoadModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  const auto state = state_machine_.getState();

  int32_t uid = planning_utils::bitShift(module_id_);
  const auto now = this->clock_->now();
  if (state == StateMachine::State::STOP) {
    appendMarkerArray(
      createPoseMarkerArray(debug_data_.stop_point_pose, "stop_point_pose", uid, 1.0, 0.0, 0.0),
      &debug_marker_array, now);
  }

  return debug_marker_array;
}

motion_utils::VirtualWalls MergeFromPrivateRoadModule::createVirtualWalls()
{
  motion_utils::VirtualWalls virtual_walls;
  const auto state = state_machine_.getState();
  if (state == StateMachine::State::STOP) {
    motion_utils::VirtualWall wall;
    wall.style = motion_utils::VirtualWallType::stop;
    wall.pose = debug_data_.virtual_wall_pose;
    wall.text = "merge_from_private_road";
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}
}  // namespace behavior_velocity_planner
