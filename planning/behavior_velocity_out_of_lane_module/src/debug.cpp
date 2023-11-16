// Copyright 2023 TIER IV, Inc.
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

#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <visualization_msgs/msg/marker.hpp>

namespace behavior_velocity_planner::out_of_lane::debug
{
namespace
{
visualization_msgs::msg::Marker get_base_marker()
{
  visualization_msgs::msg::Marker base_marker;
  base_marker.header.frame_id = "map";
  base_marker.header.stamp = rclcpp::Time(0);
  base_marker.id = 0;
  base_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  base_marker.action = visualization_msgs::msg::Marker::ADD;
  base_marker.pose.position = tier4_autoware_utils::createMarkerPosition(0.0, 0.0, 0);
  base_marker.pose.orientation = tier4_autoware_utils::createMarkerOrientation(0, 0, 0, 1.0);
  base_marker.scale = tier4_autoware_utils::createMarkerScale(0.1, 0.1, 0.1);
  base_marker.color = tier4_autoware_utils::createMarkerColor(1.0, 0.1, 0.1, 0.5);
  return base_marker;
}
}  // namespace
void add_footprint_markers(
  visualization_msgs::msg::MarkerArray & debug_marker_array,
  const lanelet::BasicPolygons2d & footprints, const double z, const size_t prev_nb)
{
  auto debug_marker = get_base_marker();
  debug_marker.ns = "footprints";
  for (const auto & f : footprints) {
    debug_marker.points.clear();
    for (const auto & p : f)
      debug_marker.points.push_back(
        tier4_autoware_utils::createMarkerPosition(p.x(), p.y(), z + 0.5));
    debug_marker.points.push_back(debug_marker.points.front());
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
    debug_marker.points.clear();
  }
  for (; debug_marker.id < static_cast<int>(prev_nb); ++debug_marker.id)
    debug_marker_array.markers.push_back(debug_marker);
}

void add_current_overlap_marker(
  visualization_msgs::msg::MarkerArray & debug_marker_array,
  const lanelet::BasicPolygon2d & current_footprint,
  const lanelet::ConstLanelets & current_overlapped_lanelets, const double z, const size_t prev_nb)
{
  auto debug_marker = get_base_marker();
  debug_marker.ns = "current_overlap";
  debug_marker.points.clear();
  for (const auto & p : current_footprint)
    debug_marker.points.push_back(tier4_autoware_utils::createMarkerPosition(p.x(), p.y(), z));
  if (!debug_marker.points.empty()) debug_marker.points.push_back(debug_marker.points.front());
  if (current_overlapped_lanelets.empty())
    debug_marker.color = tier4_autoware_utils::createMarkerColor(0.1, 1.0, 0.1, 0.5);
  else
    debug_marker.color = tier4_autoware_utils::createMarkerColor(1.0, 0.1, 0.1, 0.5);
  debug_marker_array.markers.push_back(debug_marker);
  debug_marker.id++;
  for (const auto & ll : current_overlapped_lanelets) {
    debug_marker.points.clear();
    for (const auto & p : ll.polygon3d())
      debug_marker.points.push_back(
        tier4_autoware_utils::createMarkerPosition(p.x(), p.y(), p.z() + 0.5));
    debug_marker.points.push_back(debug_marker.points.front());
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
  }
  for (; debug_marker.id < static_cast<int>(prev_nb); ++debug_marker.id)
    debug_marker_array.markers.push_back(debug_marker);
}

void add_lanelet_markers(
  visualization_msgs::msg::MarkerArray & debug_marker_array,
  const lanelet::ConstLanelets & lanelets, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color, const size_t prev_nb)
{
  auto debug_marker = get_base_marker();
  debug_marker.ns = ns;
  debug_marker.color = color;
  for (const auto & ll : lanelets) {
    debug_marker.points.clear();

    // add a small z offset to draw above the lanelet map
    for (const auto & p : ll.polygon3d())
      debug_marker.points.push_back(
        tier4_autoware_utils::createMarkerPosition(p.x(), p.y(), p.z() + 0.1));
    debug_marker.points.push_back(debug_marker.points.front());
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
  }
  debug_marker.action = debug_marker.DELETE;
  for (; debug_marker.id < static_cast<int>(prev_nb); ++debug_marker.id)
    debug_marker_array.markers.push_back(debug_marker);
}

void add_range_markers(
  visualization_msgs::msg::MarkerArray & debug_marker_array, const OverlapRanges & ranges,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t first_ego_idx,
  const double z, const size_t prev_nb)
{
  auto debug_marker = get_base_marker();
  debug_marker.ns = "ranges";
  debug_marker.color = tier4_autoware_utils::createMarkerColor(0.2, 0.9, 0.1, 0.5);
  for (const auto & range : ranges) {
    debug_marker.points.clear();
    debug_marker.points.push_back(
      path.points[first_ego_idx + range.entering_path_idx].point.pose.position);
    debug_marker.points.push_back(tier4_autoware_utils::createMarkerPosition(
      range.entering_point.x(), range.entering_point.y(), z));
    for (const auto & overlap : range.debug.overlaps) {
      debug_marker.points.push_back(tier4_autoware_utils::createMarkerPosition(
        overlap.min_overlap_point.x(), overlap.min_overlap_point.y(), z));
      debug_marker.points.push_back(tier4_autoware_utils::createMarkerPosition(
        overlap.max_overlap_point.x(), overlap.max_overlap_point.y(), z));
    }
    debug_marker.points.push_back(tier4_autoware_utils::createMarkerPosition(
      range.exiting_point.x(), range.exiting_point.y(), z));
    debug_marker.points.push_back(
      path.points[first_ego_idx + range.exiting_path_idx].point.pose.position);
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
  }
  debug_marker.action = debug_marker.DELETE;
  for (; debug_marker.id < static_cast<int>(prev_nb); ++debug_marker.id)
    debug_marker_array.markers.push_back(debug_marker);
  debug_marker.action = debug_marker.ADD;
  debug_marker.id = 0;
  debug_marker.ns = "decisions";
  debug_marker.color = tier4_autoware_utils::createMarkerColor(0.9, 0.1, 0.1, 1.0);
  debug_marker.points.clear();
  for (const auto & range : ranges) {
    debug_marker.type = debug_marker.LINE_STRIP;
    if (range.debug.decision) {
      debug_marker.points.push_back(tier4_autoware_utils::createMarkerPosition(
        range.entering_point.x(), range.entering_point.y(), z));
      debug_marker.points.push_back(
        range.debug.object->kinematics.initial_pose_with_covariance.pose.position);
    }
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.points.clear();
    debug_marker.id++;

    debug_marker.type = debug_marker.TEXT_VIEW_FACING;
    debug_marker.pose.position.x = range.entering_point.x();
    debug_marker.pose.position.y = range.entering_point.y();
    debug_marker.pose.position.z = z;
    std::stringstream ss;
    ss << "Ego: " << range.debug.times.ego.enter_time << " - " << range.debug.times.ego.exit_time
       << "\n";
    if (range.debug.object) {
      debug_marker.pose.position.x +=
        range.debug.object->kinematics.initial_pose_with_covariance.pose.position.x;
      debug_marker.pose.position.y +=
        range.debug.object->kinematics.initial_pose_with_covariance.pose.position.y;
      debug_marker.pose.position.x /= 2;
      debug_marker.pose.position.y /= 2;
      ss << "Obj: " << range.debug.times.object.enter_time << " - "
         << range.debug.times.object.exit_time << "\n";
    }
    debug_marker.scale.z = 1.0;
    debug_marker.text = ss.str();
    debug_marker_array.markers.push_back(debug_marker);
    debug_marker.id++;
  }
  for (; debug_marker.id < static_cast<int>(prev_nb); ++debug_marker.id)
    debug_marker_array.markers.push_back(debug_marker);
}

}  // namespace behavior_velocity_planner::out_of_lane::debug
