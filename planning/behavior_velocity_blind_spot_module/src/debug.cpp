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

#include "scene.hpp"

#include <behavior_velocity_planner_common/utilization/debug.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <string>

namespace behavior_velocity_planner
{
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerOrientation;
using tier4_autoware_utils::createMarkerScale;

namespace
{

visualization_msgs::msg::MarkerArray createLaneletPolygonsMarkerArray(
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
  const geometry_msgs::msg::Pose & pose, const StateMachine::State & state, const std::string & ns,
  const int64_t id, const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  if (state == StateMachine::State::STOP) {
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
  }

  return msg;
}

}  // namespace

motion_utils::VirtualWalls BlindSpotModule::createVirtualWalls()
{
  motion_utils::VirtualWalls virtual_walls;

  if (!isActivated() && !is_over_pass_judge_line_) {
    motion_utils::VirtualWall wall;
    wall.text = "blind_spot";
    wall.pose = debug_data_.virtual_wall_pose;
    wall.ns = std::to_string(module_id_) + "_";
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

visualization_msgs::msg::MarkerArray BlindSpotModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  const auto state = state_machine_.getState();
  const auto now = this->clock_->now();

  appendMarkerArray(
    createPoseMarkerArray(
      debug_data_.stop_point_pose, state, "stop_point_pose", module_id_, 1.0, 0.0, 0.0),
    &debug_marker_array, now);

  appendMarkerArray(
    createPoseMarkerArray(
      debug_data_.judge_point_pose, state, "judge_point_pose", module_id_, 1.0, 1.0, 0.5),
    &debug_marker_array, now);

  appendMarkerArray(
    createLaneletPolygonsMarkerArray(
      debug_data_.conflict_areas_for_blind_spot, "conflict_area_for_blind_spot", module_id_, 0.0,
      0.5, 0.5),
    &debug_marker_array, now);

  appendMarkerArray(
    createLaneletPolygonsMarkerArray(
      debug_data_.detection_areas_for_blind_spot, "detection_area_for_blind_spot", module_id_, 0.5,
      0.0, 0.0),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.conflicting_targets, "conflicting_targets", module_id_, now, 0.99, 0.4, 0.0),
    &debug_marker_array, now);

  return debug_marker_array;
}
}  // namespace behavior_velocity_planner
