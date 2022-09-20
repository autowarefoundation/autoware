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

#include <motion_utils/motion_utils.hpp>
#include <scene_module/intersection/scene_intersection.hpp>
#include <scene_module/intersection/scene_merge_from_private_road.hpp>
#include <utilization/debug.hpp>
#include <utilization/util.hpp>

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

}  // namespace

visualization_msgs::msg::MarkerArray IntersectionModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  const auto state = state_machine_.getState();
  const auto now = this->clock_->now();

  appendMarkerArray(
    debug::createPathMarkerArray(
      debug_data_.path_raw, "path_raw", lane_id_, now, 0.6, 0.3, 0.3, 0.0, 1.0, 1.0),
    &debug_marker_array, now);

  appendMarkerArray(
    createLaneletPolygonsMarkerArray(
      debug_data_.detection_area, "detection_area", lane_id_, 0.0, 1.0, 0.0),
    &debug_marker_array);

  appendMarkerArray(
    createLaneletPolygonsMarkerArray(
      debug_data_.adjacent_area, "adjacent_area", lane_id_, 0.913, 0.639, 0.149),
    &debug_marker_array);

  appendMarkerArray(
    debug::createPolygonMarkerArray(
      debug_data_.intersection_area, "intersection_area", lane_id_, now, 0.3, 0.0, 0.0, 0.0, 1.0,
      0.0),
    &debug_marker_array);

  appendMarkerArray(
    debug::createPolygonMarkerArray(
      debug_data_.ego_lane_polygon, "ego_lane", lane_id_, now, 0.3, 0.0, 0.0, 0.0, 0.3, 0.7),
    &debug_marker_array);

  appendMarkerArray(
    debug::createPolygonMarkerArray(
      debug_data_.stuck_vehicle_detect_area, "stuck_vehicle_detect_area", lane_id_, now, 0.3, 0.0,
      0.0, 0.0, 0.5, 0.5),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createPolygonMarkerArray(
      debug_data_.candidate_collision_ego_lane_polygon, "candidate_collision_ego_lane_polygon",
      module_id_, now, 0.3, 0.0, 0.0, 0.5, 0.0, 0.0),
    &debug_marker_array, now);

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
      debug_data_.stuck_targets, "stuck_targets", module_id_, now, 0.99, 0.99, 0.2),
    &debug_marker_array, now);

  appendMarkerArray(
    createPoseMarkerArray(
      debug_data_.predicted_obj_pose, "predicted_obj_pose", module_id_, 0.7, 0.85, 0.9),
    &debug_marker_array, now);

  if (state == StateMachine::State::STOP) {
    appendMarkerArray(
      createPoseMarkerArray(
        debug_data_.stop_point_pose, "stop_point_pose", module_id_, 1.0, 0.0, 0.0),
      &debug_marker_array, now);

    appendMarkerArray(
      createPoseMarkerArray(
        debug_data_.judge_point_pose, "judge_point_pose", module_id_, 1.0, 1.0, 0.5),
      &debug_marker_array, now);
  }

  return debug_marker_array;
}

visualization_msgs::msg::MarkerArray IntersectionModule::createVirtualWallMarkerArray()
{
  visualization_msgs::msg::MarkerArray wall_marker;

  const auto now = this->clock_->now();
  const auto state = state_machine_.getState();

  if (debug_data_.stop_required) {
    appendMarkerArray(
      motion_utils::createStopVirtualWallMarker(
        debug_data_.stop_wall_pose, "intersection", now, module_id_),
      &wall_marker, now);
  } else if (state == StateMachine::State::STOP) {
    appendMarkerArray(
      motion_utils::createStopVirtualWallMarker(
        debug_data_.slow_wall_pose, "intersection", now, module_id_),
      &wall_marker, now);
  }
  return wall_marker;
}

visualization_msgs::msg::MarkerArray MergeFromPrivateRoadModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  const auto state = state_machine_.getState();

  const auto now = this->clock_->now();
  if (state == StateMachine::State::STOP) {
    appendMarkerArray(
      createPoseMarkerArray(
        debug_data_.stop_point_pose, "stop_point_pose", module_id_, 1.0, 0.0, 0.0),
      &debug_marker_array, now);
  }

  return debug_marker_array;
}

visualization_msgs::msg::MarkerArray MergeFromPrivateRoadModule::createVirtualWallMarkerArray()
{
  visualization_msgs::msg::MarkerArray wall_marker;

  const auto state = state_machine_.getState();

  const auto now = this->clock_->now();
  if (state == StateMachine::State::STOP) {
    appendMarkerArray(
      motion_utils::createStopVirtualWallMarker(
        debug_data_.virtual_wall_pose, "merge_from_private_road", now, module_id_),
      &wall_marker, now);
  }

  return wall_marker;
}
}  // namespace behavior_velocity_planner
