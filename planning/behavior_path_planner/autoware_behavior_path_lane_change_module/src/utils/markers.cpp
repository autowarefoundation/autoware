// Copyright 2022 TIER IV, Inc.
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

#include "autoware/behavior_path_planner_common/marker_utils/colors.hpp"
#include "autoware/behavior_path_planner_common/marker_utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <autoware/behavior_path_lane_change_module/utils/markers.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

namespace marker_utils::lane_change_markers
{
using autoware::universe_utils::createDefaultMarker;
using autoware::universe_utils::createMarkerScale;
using geometry_msgs::msg::Point;

MarkerArray showAllValidLaneChangePath(const std::vector<LaneChangePath> & lanes, std::string && ns)
{
  if (lanes.empty()) {
    return MarkerArray{};
  }

  MarkerArray marker_array;
  int32_t id{0};
  const auto current_time{rclcpp::Clock{RCL_ROS_TIME}.now()};

  const auto colors = colors::colors_list();
  const auto loop_size = std::min(lanes.size(), colors.size());
  marker_array.markers.reserve(loop_size);

  for (std::size_t idx = 0; idx < loop_size; ++idx) {
    if (lanes.at(idx).path.points.empty()) {
      continue;
    }

    const auto & color = colors.at(idx);
    Marker marker = createDefaultMarker(
      "map", current_time, ns, ++id, Marker::LINE_STRIP, createMarkerScale(0.1, 0.1, 0.0), color);
    marker.points.reserve(lanes.at(idx).path.points.size());

    for (const auto & point : lanes.at(idx).path.points) {
      marker.points.push_back(point.point.pose.position);
    }

    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

MarkerArray createLaneChangingVirtualWallMarker(
  const geometry_msgs::msg::Pose & lane_changing_pose, const std::string & module_name,
  const rclcpp::Time & now, const std::string & ns)
{
  int32_t id{0};
  MarkerArray marker_array{};
  marker_array.markers.reserve(2);
  {
    auto wall_marker = createDefaultMarker(
      "map", now, ns + "virtual_wall", id, visualization_msgs::msg::Marker::CUBE,
      createMarkerScale(0.1, 5.0, 2.0), colors::green());
    wall_marker.pose = lane_changing_pose;
    wall_marker.pose.position.z += 1.0;
    marker_array.markers.push_back(wall_marker);
  }

  {
    auto text_marker = createDefaultMarker(
      "map", now, ns + "_text", id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.0, 0.0, 1.0), colors::white());
    text_marker.pose = lane_changing_pose;
    text_marker.pose.position.z += 2.0;
    text_marker.text = module_name;
    marker_array.markers.push_back(text_marker);
  }

  return marker_array;
}

MarkerArray showFilteredObjects(
  const FilteredByLanesExtendedObjects & filtered_objects, const std::string & ns)
{
  int32_t update_id = 0;
  auto current_marker = marker_utils::showFilteredObjects(
    filtered_objects.current_lane, ns, colors::yellow(), update_id);
  update_id += static_cast<int32_t>(current_marker.markers.size());
  auto target_leading_marker = marker_utils::showFilteredObjects(
    filtered_objects.target_lane_leading, ns, colors::aqua(), update_id);
  update_id += static_cast<int32_t>(target_leading_marker.markers.size());
  auto target_trailing_marker = marker_utils::showFilteredObjects(
    filtered_objects.target_lane_trailing, ns, colors::blue(), update_id);
  update_id += static_cast<int32_t>(target_trailing_marker.markers.size());
  auto other_marker = marker_utils::showFilteredObjects(
    filtered_objects.other_lane, ns, colors::medium_orchid(), update_id);

  MarkerArray marker_array;
  std::move(
    current_marker.markers.begin(), current_marker.markers.end(),
    std::back_inserter(marker_array.markers));
  std::move(
    target_leading_marker.markers.begin(), target_leading_marker.markers.end(),
    std::back_inserter(marker_array.markers));

  std::move(
    target_trailing_marker.markers.begin(), target_trailing_marker.markers.end(),
    std::back_inserter(marker_array.markers));

  std::move(
    other_marker.markers.begin(), other_marker.markers.end(),
    std::back_inserter(marker_array.markers));
  return marker_array;
}

MarkerArray showExecutionInfo(const Debug & debug_data, const geometry_msgs::msg::Pose & ego_pose)
{
  auto default_text_marker = [&]() {
    return createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "execution_info", 0, Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.5, 0.5, 0.5), colors::white());
  };

  MarkerArray marker_array;

  auto safety_check_info_text = default_text_marker();
  safety_check_info_text.pose = ego_pose;
  safety_check_info_text.pose.position.z += 4.0;

  std::ostringstream ss;

  ss << "\nDistToEndOfCurrentLane: " << std::setprecision(5)
     << debug_data.distance_to_end_of_current_lane
     << "\nDistToLaneChangeFinished: " << debug_data.distance_to_lane_change_finished
     << (debug_data.is_stuck ? "\nVehicleStuck" : "")
     << (debug_data.is_able_to_return_to_current_lane ? "\nAbleToReturnToCurrentLane" : "")
     << (debug_data.is_abort ? "\nAborting" : "")
     << "\nDistanceToAbortFinished: " << debug_data.distance_to_abort_finished;

  safety_check_info_text.text = ss.str();
  marker_array.markers.push_back(safety_check_info_text);
  return marker_array;
}

MarkerArray createDebugMarkerArray(
  const Debug & debug_data, const geometry_msgs::msg::Pose & ego_pose)
{
  using lanelet::visualization::laneletsAsTriangleMarkerArray;
  using marker_utils::showPolygon;
  using marker_utils::showPredictedPath;
  using marker_utils::showSafetyCheckInfo;
  using marker_utils::lane_change_markers::showAllValidLaneChangePath;
  using marker_utils::lane_change_markers::showFilteredObjects;

  const auto & debug_collision_check_object = debug_data.collision_check_objects;
  const auto & debug_collision_check_object_after_approval =
    debug_data.collision_check_objects_after_approval;
  const auto & debug_valid_paths = debug_data.valid_paths;
  const auto & debug_filtered_objects = debug_data.filtered_objects;

  MarkerArray debug_marker;
  const auto add = [&debug_marker](const MarkerArray & added) {
    autoware::universe_utils::appendMarkerArray(added, &debug_marker);
  };

  if (!debug_data.execution_area.points.empty()) {
    add(createPolygonMarkerArray(
      debug_data.execution_area, "execution_area", 0, 0.16, 1.0, 0.69, 0.1));
  }

  add(showExecutionInfo(debug_data, ego_pose));

  // lanes
  add(laneletsAsTriangleMarkerArray(
    "current_lanes", debug_data.current_lanes, colors::light_yellow(0.2)));
  add(laneletsAsTriangleMarkerArray("target_lanes", debug_data.target_lanes, colors::aqua(0.2)));
  add(laneletsAsTriangleMarkerArray(
    "target_backward_lanes", debug_data.target_backward_lanes, colors::blue(0.2)));

  add(showAllValidLaneChangePath(debug_valid_paths, "lane_change_valid_paths"));
  add(showFilteredObjects(debug_filtered_objects, "object_filtered"));

  if (!debug_collision_check_object.empty()) {
    add(showSafetyCheckInfo(debug_collision_check_object, "collision_check_object_info"));
    add(showPredictedPath(debug_collision_check_object, "ego_predicted_path"));
    add(showPolygon(debug_collision_check_object, "ego_and_target_polygon_relation"));
  }

  if (!debug_collision_check_object_after_approval.empty()) {
    add(showSafetyCheckInfo(
      debug_collision_check_object_after_approval, "object_debug_info_after_approval"));
    add(showPredictedPath(
      debug_collision_check_object_after_approval, "ego_predicted_path_after_approval"));
    add(showPolygon(
      debug_collision_check_object_after_approval,
      "ego_and_target_polygon_relation_after_approval"));
  }

  return debug_marker;
}
}  // namespace marker_utils::lane_change_markers
