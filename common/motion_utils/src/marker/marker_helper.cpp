// Copyright 2021 Tier IV, Inc.
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

#include "motion_utils/marker/marker_helper.hpp"

#include <string>

using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createDeletedDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using visualization_msgs::msg::MarkerArray;

namespace
{

inline visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray(
  const geometry_msgs::msg::Pose & vehicle_front_pose, const std::string & module_name,
  const std::string & ns_prefix, const rclcpp::Time & now, const int32_t id,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Virtual Wall
  {
    auto marker = createDefaultMarker(
      "map", now, ns_prefix + "virtual_wall", id, visualization_msgs::msg::Marker::CUBE,
      createMarkerScale(0.1, 5.0, 2.0), color);

    marker.pose = vehicle_front_pose;
    marker.pose.position.z += 1.0;

    marker_array.markers.push_back(marker);
  }

  // Factor Text
  {
    auto marker = createDefaultMarker(
      "map", now, ns_prefix + "factor_text", id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.0, 0.0, 1.0), createMarkerColor(1.0, 1.0, 1.0, 1.0));

    marker.pose = vehicle_front_pose;
    marker.pose.position.z += 2.0;
    marker.text = module_name;

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

inline visualization_msgs::msg::MarkerArray createDeletedVirtualWallMarkerArray(
  const std::string & ns_prefix, const rclcpp::Time & now, const int32_t id)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Virtual Wall
  {
    auto marker = createDeletedDefaultMarker(now, ns_prefix + "virtual_wall", id);
    marker_array.markers.push_back(marker);
  }

  // Factor Text
  {
    auto marker = createDeletedDefaultMarker(now, ns_prefix + "factor_text", id);
    marker_array.markers.push_back(marker);
  }

  return marker_array;
}
}  // namespace

namespace motion_utils
{
visualization_msgs::msg::MarkerArray createStopVirtualWallMarker(
  const geometry_msgs::msg::Pose & pose, const std::string & module_name, const rclcpp::Time & now,
  const int32_t id, const double longitudinal_offset)
{
  const auto pose_with_offset =
    tier4_autoware_utils::calcOffsetPose(pose, longitudinal_offset, 0.0, 0.0);
  return createVirtualWallMarkerArray(
    pose_with_offset, module_name, "stop_", now, id, createMarkerColor(1.0, 0.0, 0.0, 0.5));
}

visualization_msgs::msg::MarkerArray createSlowDownVirtualWallMarker(
  const geometry_msgs::msg::Pose & pose, const std::string & module_name, const rclcpp::Time & now,
  const int32_t id, const double longitudinal_offset)
{
  const auto pose_with_offset =
    tier4_autoware_utils::calcOffsetPose(pose, longitudinal_offset, 0.0, 0.0);
  return createVirtualWallMarkerArray(
    pose_with_offset, module_name, "slow_down_", now, id, createMarkerColor(1.0, 1.0, 0.0, 0.5));
}

visualization_msgs::msg::MarkerArray createDeadLineVirtualWallMarker(
  const geometry_msgs::msg::Pose & pose, const std::string & module_name, const rclcpp::Time & now,
  const int32_t id, const double longitudinal_offset)
{
  const auto pose_with_offset =
    tier4_autoware_utils::calcOffsetPose(pose, longitudinal_offset, 0.0, 0.0);
  return createVirtualWallMarkerArray(
    pose_with_offset, module_name, "dead_line_", now, id, createMarkerColor(0.0, 1.0, 0.0, 0.5));
}

visualization_msgs::msg::MarkerArray createDeletedStopVirtualWallMarker(
  const rclcpp::Time & now, const int32_t id)
{
  return createDeletedVirtualWallMarkerArray("stop_", now, id);
}

visualization_msgs::msg::MarkerArray createDeletedSlowDownVirtualWallMarker(
  const rclcpp::Time & now, const int32_t id)
{
  return createDeletedVirtualWallMarkerArray("slow_down_", now, id);
}

visualization_msgs::msg::MarkerArray createDeletedDeadLineVirtualWallMarker(
  const rclcpp::Time & now, const int32_t id)
{
  return createDeletedVirtualWallMarkerArray("dead_line_", now, id);
}

visualization_msgs::msg::MarkerArray VirtualWallMarkerCreator::handleVirtualWallMarker(
  const std::vector<Pose> & poses, const std::string & module_name, const rclcpp::Time & now,
  create_wall_function function_create_wall_marker,
  delete_wall_function function_delete_wall_marker, size_t & previous_virtual_walls_nb,
  const double longitudinal_offset)
{
  visualization_msgs::msg::MarkerArray wall_marker;

  int32_t id = 0;
  const auto max_id = static_cast<int32_t>(previous_virtual_walls_nb);

  for (const auto & p : poses) {
    appendMarkerArray(
      function_create_wall_marker(p, module_name, now, id++, longitudinal_offset), &wall_marker);
  }

  while (id < max_id) {
    appendMarkerArray(function_delete_wall_marker(now, id++), &wall_marker, now);
  }

  previous_virtual_walls_nb = poses.size();
  return wall_marker;
}

visualization_msgs::msg::MarkerArray VirtualWallMarkerCreator::createStopVirtualWallMarker(
  const std::vector<Pose> & stop_poses, const std::string & module_name, const rclcpp::Time & now,
  const double longitudinal_offset)
{
  create_wall_function creator = motion_utils::createStopVirtualWallMarker;
  delete_wall_function deleter = motion_utils::createDeletedStopVirtualWallMarker;

  return handleVirtualWallMarker(
    stop_poses, module_name, now, creator, deleter, previous_stop_poses_nb_, longitudinal_offset);
}

visualization_msgs::msg::MarkerArray VirtualWallMarkerCreator::createSlowDownVirtualWallMarker(
  const std::vector<Pose> & slow_down_poses, const std::string & module_name,
  const rclcpp::Time & now, const double longitudinal_offset)
{
  create_wall_function creator = motion_utils::createSlowDownVirtualWallMarker;
  delete_wall_function deleter = motion_utils::createDeletedSlowDownVirtualWallMarker;

  return handleVirtualWallMarker(
    slow_down_poses, module_name, now, creator, deleter, previous_slow_down_poses_nb_,
    longitudinal_offset);
}

visualization_msgs::msg::MarkerArray VirtualWallMarkerCreator::createDeadLineVirtualWallMarker(
  const std::vector<Pose> & dead_line_poses, const std::string & module_name,
  const rclcpp::Time & now, const double longitudinal_offset)
{
  create_wall_function creator = motion_utils::createDeadLineVirtualWallMarker;
  delete_wall_function deleter = motion_utils::createDeletedDeadLineVirtualWallMarker;

  return handleVirtualWallMarker(
    dead_line_poses, module_name, now, creator, deleter, previous_dead_line_poses_nb_,
    longitudinal_offset);
}

}  // namespace motion_utils
