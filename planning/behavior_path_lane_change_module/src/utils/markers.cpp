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

#include "behavior_path_planner_common/marker_utils/colors.hpp"
#include "behavior_path_planner_common/marker_utils/utils.hpp"
#include "behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <behavior_path_lane_change_module/utils/markers.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>

#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

namespace marker_utils::lane_change_markers
{
using geometry_msgs::msg::Point;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerScale;

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
  const ExtendedPredictedObjects & current_lane_objects,
  const ExtendedPredictedObjects & target_lane_objects,
  const ExtendedPredictedObjects & other_lane_objects, const std::string & ns)
{
  int32_t update_id = 0;
  auto current_marker =
    marker_utils::showFilteredObjects(current_lane_objects, ns, colors::yellow(), update_id);
  update_id += static_cast<int32_t>(current_marker.markers.size());
  auto target_marker =
    marker_utils::showFilteredObjects(target_lane_objects, ns, colors::aqua(), update_id);
  update_id += static_cast<int32_t>(target_marker.markers.size());
  auto other_marker =
    marker_utils::showFilteredObjects(other_lane_objects, ns, colors::medium_orchid(), update_id);

  MarkerArray marker_array;
  marker_array.markers.insert(
    marker_array.markers.end(), current_marker.markers.begin(), current_marker.markers.end());
  marker_array.markers.insert(
    marker_array.markers.end(), target_marker.markers.begin(), target_marker.markers.end());
  marker_array.markers.insert(
    marker_array.markers.end(), other_marker.markers.begin(), other_marker.markers.end());
  return marker_array;
}
}  // namespace marker_utils::lane_change_markers
