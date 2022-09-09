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
#include <scene_module/blind_spot/scene.hpp>
#include <utilization/debug.hpp>
#include <utilization/util.hpp>

#include <string>

namespace behavior_velocity_planner
{
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerOrientation;
using tier4_autoware_utils::createMarkerScale;

namespace
{

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

visualization_msgs::msg::MarkerArray BlindSpotModule::createVirtualWallMarkerArray()
{
  visualization_msgs::msg::MarkerArray wall_marker;

  const auto now = this->clock_->now();

  if (!isActivated() && !is_over_pass_judge_line_) {
    appendMarkerArray(
      motion_utils::createStopVirtualWallMarker(
        debug_data_.virtual_wall_pose, "blind_spot", now, module_id_),
      &wall_marker, now);
  }
  return wall_marker;
}

visualization_msgs::msg::MarkerArray BlindSpotModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  const auto state = state_machine_.getState();
  const auto now = this->clock_->now();

  appendMarkerArray(
    debug::createPathMarkerArray(
      debug_data_.path_raw, "path_raw", lane_id_, now, 0.6, 0.3, 0.3, 0.0, 1.0, 1.0),
    &debug_marker_array, now);

  appendMarkerArray(
    createPoseMarkerArray(
      debug_data_.stop_point_pose, state, "stop_point_pose", module_id_, 1.0, 0.0, 0.0),
    &debug_marker_array, now);

  appendMarkerArray(
    createPoseMarkerArray(
      debug_data_.judge_point_pose, state, "judge_point_pose", module_id_, 1.0, 1.0, 0.5),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createPolygonMarkerArray(
      debug_data_.conflict_area_for_blind_spot, "conflict_area_for_blind_spot", module_id_, now,
      0.3, 0.0, 0.0, 0.0, 0.5, 0.5),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createPolygonMarkerArray(
      debug_data_.detection_area_for_blind_spot, "detection_area_for_blind_spot", module_id_, now,
      0.3, 0.0, 0.0, 0.0, 0.5, 0.5),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createObjectsMarkerArray(
      debug_data_.conflicting_targets, "conflicting_targets", module_id_, now, 0.99, 0.4, 0.0),
    &debug_marker_array, now);

  appendMarkerArray(
    debug::createPathMarkerArray(
      debug_data_.spline_path, "spline", lane_id_, now, 0.3, 0.1, 0.1, 0.5, 0.5, 0.5),
    &debug_marker_array, now);

  return debug_marker_array;
}
}  // namespace behavior_velocity_planner
