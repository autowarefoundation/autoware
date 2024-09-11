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

#include <autoware/behavior_velocity_planner_common/utilization/debug.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <string>
#include <vector>

namespace
{
using autoware::universe_utils::appendMarkerArray;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerOrientation;
using autoware::universe_utils::createMarkerScale;

visualization_msgs::msg::MarkerArray createLaneletPolygonsMarkerArray(
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  int32_t i = 0;
  int32_t uid = autoware::behavior_velocity_planner::planning_utils::bitShift(lane_id);
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
  marker_line.scale = createMarkerScale(0.2, 0.0, 0.0);
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

visualization_msgs::msg::MarkerArray createArrowLineMarkerArray(
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
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;
  marker.color = createMarkerColor(r, g, b, 0.999);
  marker.points.push_back(point_start);
  marker.points.push_back(point_end);

  msg.markers.push_back(marker);
  return msg;
}

constexpr std::tuple<float, float, float> white()
{
  constexpr uint64_t code = 0xfdfdfd;
  constexpr float r = static_cast<int>(code >> 16) / 255.0;
  constexpr float g = static_cast<int>((code << 48) >> 56) / 255.0;
  constexpr float b = static_cast<int>((code << 56) >> 56) / 255.0;
  return {r, g, b};
}

constexpr std::tuple<float, float, float> green()
{
  constexpr uint64_t code = 0x5fa641;
  constexpr float r = static_cast<int>(code >> 16) / 255.0;
  constexpr float g = static_cast<int>((code << 48) >> 56) / 255.0;
  constexpr float b = static_cast<int>((code << 56) >> 56) / 255.0;
  return {r, g, b};
}

constexpr std::tuple<float, float, float> yellow()
{
  constexpr uint64_t code = 0xebce2b;
  constexpr float r = static_cast<int>(code >> 16) / 255.0;
  constexpr float g = static_cast<int>((code << 48) >> 56) / 255.0;
  constexpr float b = static_cast<int>((code << 56) >> 56) / 255.0;
  return {r, g, b};
}

constexpr std::tuple<float, float, float> red()
{
  constexpr uint64_t code = 0xba1c30;
  constexpr float r = static_cast<int>(code >> 16) / 255.0;
  constexpr float g = static_cast<int>((code << 48) >> 56) / 255.0;
  constexpr float b = static_cast<int>((code << 56) >> 56) / 255.0;
  return {r, g, b};
}

constexpr std::tuple<float, float, float> light_blue()
{
  constexpr uint64_t code = 0x96cde6;
  constexpr float r = static_cast<int>(code >> 16) / 255.0;
  constexpr float g = static_cast<int>((code << 48) >> 56) / 255.0;
  constexpr float b = static_cast<int>((code << 56) >> 56) / 255.0;
  return {r, g, b};
}
}  // namespace

namespace autoware::behavior_velocity_planner
{
using autoware::universe_utils::appendMarkerArray;
using autoware::universe_utils::createMarkerColor;
using autoware::universe_utils::createMarkerOrientation;
using autoware::universe_utils::createMarkerScale;

visualization_msgs::msg::MarkerArray IntersectionModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  const auto now = this->clock_->now();

  if (debug_data_.attention_area) {
    appendMarkerArray(
      ::createLaneletPolygonsMarkerArray(
        debug_data_.attention_area.value(), "attention_area", lane_id_, 0.0, 1.0, 0.0),
      &debug_marker_array);
  }

  if (debug_data_.occlusion_attention_area) {
    appendMarkerArray(
      ::createLaneletPolygonsMarkerArray(
        debug_data_.occlusion_attention_area.value(), "occlusion_attention_area", lane_id_, 0.917,
        0.568, 0.596),
      &debug_marker_array);
  }

  if (debug_data_.adjacent_area) {
    appendMarkerArray(
      ::createLaneletPolygonsMarkerArray(
        debug_data_.adjacent_area.value(), "adjacent_area", lane_id_, 0.913, 0.639, 0.149),
      &debug_marker_array);
  }

  if (debug_data_.first_attention_area) {
    appendMarkerArray(
      ::createLaneletPolygonsMarkerArray(
        {debug_data_.first_attention_area.value()}, "first_attention_area", lane_id_, 1, 0.647,
        0.0),
      &debug_marker_array, now);
  }

  if (debug_data_.second_attention_area) {
    appendMarkerArray(
      ::createLaneletPolygonsMarkerArray(
        {debug_data_.second_attention_area.value()}, "second_attention_area", lane_id_, 1, 0.647,
        0.0),
      &debug_marker_array, now);
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
      ::createLaneletPolygonsMarkerArray(
        debug_data_.yield_stuck_detect_area.value(), "yield_stuck_detect_area", lane_id_, 0.6588235,
        0.34509, 0.6588235),
      &debug_marker_array);
  }

  if (debug_data_.ego_lane) {
    appendMarkerArray(
      ::createLaneletPolygonsMarkerArray(
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

  static constexpr auto white = ::white();
  static constexpr auto green = ::green();
  static constexpr auto yellow = ::yellow();
  static constexpr auto red = ::red();
  static constexpr auto light_blue = ::light_blue();
  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.safe_under_traffic_control_targets, "safe_under_traffic_control_targets",
      module_id_, now, std::get<0>(light_blue), std::get<1>(light_blue), std::get<2>(light_blue)),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.unsafe_targets, "unsafe_targets", module_id_, now, std::get<0>(green),
      std::get<1>(green), std::get<2>(green)),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.misjudge_targets, "misjudge_targets", module_id_, now, std::get<0>(yellow),
      std::get<1>(yellow), std::get<2>(yellow)),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.too_late_detect_targets, "too_late_detect_targets", module_id_, now,
      std::get<0>(red), std::get<1>(red), std::get<2>(red)),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.parked_targets, "parked_targets", module_id_, now, std::get<0>(white),
      std::get<1>(white), std::get<2>(white)),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.stuck_targets, "stuck_targets", module_id_, now, std::get<0>(white),
      std::get<1>(white), std::get<2>(white)),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.yield_stuck_targets, "yield_stuck_targets", module_id_, now, std::get<0>(white),
      std::get<1>(white), std::get<2>(white)),
    &debug_marker_array, now);

  if (debug_data_.first_pass_judge_wall_pose) {
    const double r = debug_data_.passed_first_pass_judge ? 1.0 : 0.0;
    const double g = debug_data_.passed_first_pass_judge ? 0.0 : 1.0;
    appendMarkerArray(
      ::createPoseMarkerArray(
        debug_data_.first_pass_judge_wall_pose.value(), "first_pass_judge_wall_pose", module_id_, r,
        g, 0.0),
      &debug_marker_array, now);
  }

  if (debug_data_.second_pass_judge_wall_pose) {
    const double r = debug_data_.passed_second_pass_judge ? 1.0 : 0.0;
    const double g = debug_data_.passed_second_pass_judge ? 0.0 : 1.0;
    appendMarkerArray(
      ::createPoseMarkerArray(
        debug_data_.second_pass_judge_wall_pose.value(), "second_pass_judge_wall_pose", module_id_,
        r, g, 0.0),
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
      ::createArrowLineMarkerArray(
        point_start, point_end, "nearest_occlusion_projection", lane_id_, 0.5, 0.5, 0.0),
      &debug_marker_array, now);
  }

  if (debug_data_.nearest_occlusion_triangle) {
    const auto [p1, p2, p3] = debug_data_.nearest_occlusion_triangle.value();
    const auto color = debug_data_.static_occlusion ? green : red;
    geometry_msgs::msg::Polygon poly;
    poly.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(p1.x).y(p1.y).z(p1.z));
    poly.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(p2.x).y(p2.y).z(p2.z));
    poly.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(p3.x).y(p3.y).z(p3.z));
    appendMarkerArray(
      debug::createPolygonMarkerArray(
        poly, "nearest_occlusion_triangle", lane_id_, now, 0.3, 0.0, 0.0, std::get<0>(color),
        std::get<1>(color), std::get<2>(color)),
      &debug_marker_array, now);
  }
  if (debug_data_.traffic_light_observation) {
    const auto GREEN = autoware_perception_msgs::msg::TrafficLightElement::GREEN;
    const auto YELLOW = autoware_perception_msgs::msg::TrafficLightElement::AMBER;

    const auto [ego, tl_point, id, color] = debug_data_.traffic_light_observation.value();
    geometry_msgs::msg::Point tl_point_point;
    tl_point_point.x = tl_point.x();
    tl_point_point.y = tl_point.y();
    tl_point_point.z = tl_point.z();
    const auto tl_color = (color == GREEN) ? green : (color == YELLOW ? yellow : red);
    const auto [r, g, b] = tl_color;
    appendMarkerArray(
      ::createLineMarkerArray(
        ego.position, tl_point_point, "intersection_traffic_light", lane_id_, r, g, b),
      &debug_marker_array, now);
  }
  return debug_marker_array;
}

autoware::motion_utils::VirtualWalls IntersectionModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;

  if (debug_data_.collision_stop_wall_pose) {
    wall.style = autoware::motion_utils::VirtualWallType::stop;
    wall.text = "intersection";
    wall.ns = "intersection" + std::to_string(module_id_) + "_";
    wall.pose = debug_data_.collision_stop_wall_pose.value();
    virtual_walls.push_back(wall);
  }
  if (debug_data_.occlusion_first_stop_wall_pose) {
    wall.style = autoware::motion_utils::VirtualWallType::stop;
    wall.text = "intersection";
    wall.ns = "intersection_occlusion_first_stop" + std::to_string(module_id_) + "_";
    wall.pose = debug_data_.occlusion_first_stop_wall_pose.value();
    virtual_walls.push_back(wall);
  }
  if (debug_data_.occlusion_stop_wall_pose) {
    wall.style = autoware::motion_utils::VirtualWallType::stop;
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
    wall.style = autoware::motion_utils::VirtualWallType::slowdown;
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

  int32_t uid = autoware::behavior_velocity_planner::planning_utils::bitShift(module_id_);
  const auto now = this->clock_->now();
  if (state == StateMachine::State::STOP) {
    appendMarkerArray(
      ::createPoseMarkerArray(debug_data_.stop_point_pose, "stop_point_pose", uid, 1.0, 0.0, 0.0),
      &debug_marker_array, now);
  }

  return debug_marker_array;
}

autoware::motion_utils::VirtualWalls MergeFromPrivateRoadModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  const auto state = state_machine_.getState();
  if (state == StateMachine::State::STOP) {
    autoware::motion_utils::VirtualWall wall;
    wall.style = autoware::motion_utils::VirtualWallType::stop;
    wall.pose = debug_data_.virtual_wall_pose;
    wall.text = "merge_from_private_road";
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}
}  // namespace autoware::behavior_velocity_planner
