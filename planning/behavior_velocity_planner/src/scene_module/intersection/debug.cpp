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

#include <scene_module/intersection/scene_intersection.hpp>
#include <scene_module/intersection/scene_merge_from_private_road.hpp>
#include <utilization/marker_helper.hpp>
#include <utilization/util.hpp>

#include <string>
#include <vector>

namespace behavior_velocity_planner
{
namespace
{
using State = IntersectionModule::State;

visualization_msgs::msg::MarkerArray createLaneletPolygonsMarkerArray(
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const std::string & ns,
  const int64_t lane_id)
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
    marker.color = createMarkerColor(0.0, 1.0, 0.0, 0.999);
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

visualization_msgs::msg::MarkerArray createPolygonMarkerArray(
  const geometry_msgs::msg::Polygon & polygon, const std::string & ns, const int64_t lane_id,
  const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";

  marker.ns = ns;
  marker.id = lane_id;
  marker.lifetime = rclcpp::Duration::from_seconds(0.3);
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
  marker.scale = createMarkerScale(0.3, 0.0, 0.0);
  marker.color = createMarkerColor(r, g, b, 0.8);
  for (const auto & p : polygon.points) {
    geometry_msgs::msg::Point point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    marker.points.push_back(point);
  }
  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }
  msg.markers.push_back(marker);

  return msg;
}

visualization_msgs::msg::MarkerArray createObjectsMarkerArray(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker{};
  marker.header.frame_id = "map";
  marker.ns = ns;

  int32_t uid = planning_utils::bitShift(lane_id);
  int32_t i = 0;
  for (const auto & object : objects.objects) {
    marker.id = uid + i++;
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

visualization_msgs::msg::MarkerArray createPathMarkerArray(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const std::string & ns,
  const int64_t lane_id, const double r, const double g, const double b)
{
  visualization_msgs::msg::MarkerArray msg;
  int32_t uid = planning_utils::bitShift(lane_id);
  int32_t i = 0;
  for (const auto & p : path.points) {
    visualization_msgs::msg::Marker marker{};
    marker.header.frame_id = "map";
    marker.ns = ns;
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = p.point.pose;
    marker.scale = createMarkerScale(0.6, 0.3, 0.3);
    if (std::find(p.lane_ids.begin(), p.lane_ids.end(), lane_id) != p.lane_ids.end()) {
      // if p.lane_ids has lane_id
      marker.color = createMarkerColor(r, g, b, 0.999);
    } else {
      marker.color = createMarkerColor(0.5, 0.5, 0.5, 0.999);
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray createVirtualStopWallMarkerArray(
  const geometry_msgs::msg::Pose & pose, const int64_t lane_id, const std::string & stop_factor)
{
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker_virtual_wall{};
  marker_virtual_wall.header.frame_id = "map";
  marker_virtual_wall.ns = "stop_virtual_wall";
  marker_virtual_wall.id = lane_id;
  marker_virtual_wall.lifetime = rclcpp::Duration::from_seconds(0.5);
  marker_virtual_wall.type = visualization_msgs::msg::Marker::CUBE;
  marker_virtual_wall.action = visualization_msgs::msg::Marker::ADD;
  marker_virtual_wall.pose = pose;
  marker_virtual_wall.pose.position.z += 1.0;
  marker_virtual_wall.scale = createMarkerScale(0.1, 5.0, 2.0);
  marker_virtual_wall.color = createMarkerColor(1.0, 0.0, 0.0, 0.5);
  msg.markers.push_back(marker_virtual_wall);

  visualization_msgs::msg::Marker marker_factor_text{};
  marker_factor_text.header.frame_id = "map";
  marker_factor_text.ns = "stop_factor_text";
  marker_factor_text.id = lane_id;
  marker_factor_text.lifetime = rclcpp::Duration::from_seconds(0.5);
  marker_factor_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_factor_text.action = visualization_msgs::msg::Marker::ADD;
  marker_factor_text.pose = pose;
  marker_factor_text.pose.position.z += 2.0;
  marker_factor_text.scale = createMarkerScale(0.0, 0.0, 1.0);
  marker_factor_text.color = createMarkerColor(1.0, 1.0, 1.0, 0.999);
  marker_factor_text.text = stop_factor;
  msg.markers.push_back(marker_factor_text);

  return msg;
}

visualization_msgs::msg::MarkerArray createVirtualSlowWallMarkerArray(
  const geometry_msgs::msg::Pose & pose, const int64_t lane_id, const std::string & slow_factor)
{
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker_virtual_wall{};
  marker_virtual_wall.header.frame_id = "map";
  marker_virtual_wall.ns = "slow_virtual_wall";
  marker_virtual_wall.id = lane_id;
  marker_virtual_wall.lifetime = rclcpp::Duration::from_seconds(0.5);
  marker_virtual_wall.type = visualization_msgs::msg::Marker::CUBE;
  marker_virtual_wall.action = visualization_msgs::msg::Marker::ADD;
  marker_virtual_wall.pose = pose;
  marker_virtual_wall.pose.position.z += 1.0;
  marker_virtual_wall.scale = createMarkerScale(0.1, 5.0, 2.0);
  marker_virtual_wall.color = createMarkerColor(1.0, 1.0, 0.0, 0.5);
  msg.markers.push_back(marker_virtual_wall);

  visualization_msgs::msg::Marker marker_factor_text{};
  marker_factor_text.header.frame_id = "map";
  marker_factor_text.ns = "slow_factor_text";
  marker_factor_text.id = lane_id;
  marker_factor_text.lifetime = rclcpp::Duration::from_seconds(0.5);
  marker_factor_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker_factor_text.action = visualization_msgs::msg::Marker::ADD;
  marker_factor_text.pose = pose;
  marker_factor_text.pose.position.z += 2.0;
  marker_factor_text.scale = createMarkerScale(0.0, 0.0, 1.0);
  marker_factor_text.color = createMarkerColor(1.0, 1.0, 1.0, 0.999);
  marker_factor_text.text = slow_factor;
  msg.markers.push_back(marker_factor_text);

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
  const auto current_time = this->clock_->now();

  appendMarkerArray(
    createPathMarkerArray(debug_data_.path_raw, "path_raw", lane_id_, 0.0, 1.0, 1.0), current_time,
    &debug_marker_array);

  appendMarkerArray(
    createLaneletPolygonsMarkerArray(debug_data_.detection_area, "detection_area", lane_id_),
    current_time, &debug_marker_array);

  appendMarkerArray(
    createPolygonMarkerArray(debug_data_.ego_lane_polygon, "ego_lane", lane_id_, 0.0, 0.3, 0.7),
    current_time, &debug_marker_array);

  appendMarkerArray(
    createPolygonMarkerArray(
      debug_data_.stuck_vehicle_detect_area, "stuck_vehicle_detect_area", lane_id_, 0.0, 0.5, 0.5),
    current_time, &debug_marker_array);

  appendMarkerArray(
    createPolygonMarkerArray(
      debug_data_.candidate_collision_ego_lane_polygon, "candidate_collision_ego_lane_polygon",
      lane_id_, 0.5, 0.0, 0.0),
    current_time, &debug_marker_array);

  size_t i{0};
  for (const auto & p : debug_data_.candidate_collision_object_polygons) {
    appendMarkerArray(
      createPolygonMarkerArray(
        p, "candidate_collision_object_polygons", lane_id_ + i++, 0.0, 0.5, 0.5),
      current_time, &debug_marker_array);
  }

  appendMarkerArray(
    createObjectsMarkerArray(
      debug_data_.conflicting_targets, "conflicting_targets", lane_id_, 0.99, 0.4, 0.0),
    current_time, &debug_marker_array);

  appendMarkerArray(
    createObjectsMarkerArray(debug_data_.stuck_targets, "stuck_targets", lane_id_, 0.99, 0.99, 0.2),
    current_time, &debug_marker_array);

  if (state == IntersectionModule::State::STOP) {
    appendMarkerArray(
      createPoseMarkerArray(
        debug_data_.stop_point_pose, "stop_point_pose", lane_id_, 1.0, 0.0, 0.0),
      current_time, &debug_marker_array);

    appendMarkerArray(
      createPoseMarkerArray(
        debug_data_.judge_point_pose, "judge_point_pose", lane_id_, 1.0, 1.0, 0.5),
      current_time, &debug_marker_array);

    if (debug_data_.stop_required) {
      appendMarkerArray(
        createVirtualStopWallMarkerArray(debug_data_.stop_wall_pose, lane_id_, "intersection"),
        current_time, &debug_marker_array);
    } else {
      appendMarkerArray(
        createVirtualSlowWallMarkerArray(debug_data_.slow_wall_pose, lane_id_, "intersection"),
        current_time, &debug_marker_array);
    }
  }

  return debug_marker_array;
}

visualization_msgs::msg::MarkerArray MergeFromPrivateRoadModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  const auto state = state_machine_.getState();

  const auto current_time = this->clock_->now();
  if (state == MergeFromPrivateRoadModule::State::STOP) {
    appendMarkerArray(
      createPoseMarkerArray(
        debug_data_.stop_point_pose, "stop_point_pose", lane_id_, 1.0, 0.0, 0.0),
      current_time, &debug_marker_array);

    appendMarkerArray(
      createVirtualStopWallMarkerArray(
        debug_data_.virtual_wall_pose, lane_id_, "merge_from_private_road"),
      current_time, &debug_marker_array);
  }

  return debug_marker_array;
}
}  // namespace behavior_velocity_planner
