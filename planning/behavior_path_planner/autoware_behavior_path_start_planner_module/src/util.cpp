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

#include "autoware/behavior_path_start_planner_module/util.hpp"

#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::start_planner_utils
{
PathWithLaneId getBackwardPath(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & shoulder_lanes,
  const Pose & current_pose, const Pose & backed_pose, const double velocity)
{
  const auto current_pose_arc_coords =
    lanelet::utils::getArcCoordinates(shoulder_lanes, current_pose);
  const auto backed_pose_arc_coords =
    lanelet::utils::getArcCoordinates(shoulder_lanes, backed_pose);

  const double s_start = backed_pose_arc_coords.length;
  const double s_end = current_pose_arc_coords.length;

  PathWithLaneId backward_path;
  {
    // forward center line path
    backward_path = route_handler.getCenterLinePath(shoulder_lanes, s_start, s_end, true);

    // backward center line path
    std::reverse(backward_path.points.begin(), backward_path.points.end());
    for (auto & p : backward_path.points) {
      p.point.longitudinal_velocity_mps = velocity;
    }
    backward_path.points.back().point.longitudinal_velocity_mps = 0.0;

    // lateral shift to current_pose
    const double lateral_distance_to_shoulder_center = current_pose_arc_coords.distance;
    for (size_t i = 0; i < backward_path.points.size(); ++i) {
      auto & p = backward_path.points.at(i).point.pose;
      p = autoware::universe_utils::calcOffsetPose(p, 0, lateral_distance_to_shoulder_center, 0);
    }
  }

  return backward_path;
}

lanelet::ConstLanelets getPullOutLanes(
  const std::shared_ptr<const PlannerData> & planner_data, const double backward_length)
{
  const double & vehicle_width = planner_data->parameters.vehicle_width;
  const auto & route_handler = planner_data->route_handler;
  const auto start_pose = planner_data->route_handler->getOriginalStartPose();

  const auto current_shoulder_lane = route_handler->getPullOutStartLane(start_pose, vehicle_width);
  if (current_shoulder_lane) {
    // pull out from shoulder lane
    return route_handler->getShoulderLaneletSequence(*current_shoulder_lane, start_pose);
  }

  // pull out from road lane
  return utils::getExtendedCurrentLanes(
    planner_data, backward_length,
    /*forward_length*/ std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);
}

std::optional<PathWithLaneId> extractCollisionCheckSection(
  const PullOutPath & path, const double collision_check_distance_from_end)
{
  PathWithLaneId full_path;
  for (const auto & partial_path : path.partial_paths) {
    full_path.points.insert(
      full_path.points.end(), partial_path.points.begin(), partial_path.points.end());
  }

  if (full_path.points.empty()) return std::nullopt;
  // Find the start index for collision check section based on the shift start pose
  const auto shift_start_idx =
    autoware::motion_utils::findNearestIndex(full_path.points, path.start_pose.position);

  // Find the end index for collision check section based on the end pose and collision check
  // distance
  const auto collision_check_end_idx = [&]() -> size_t {
    const auto end_pose_offset = autoware::motion_utils::calcLongitudinalOffsetPose(
      full_path.points, path.end_pose.position, collision_check_distance_from_end);

    return end_pose_offset
             ? autoware::motion_utils::findNearestIndex(full_path.points, end_pose_offset->position)
             : full_path.points.size() - 1;  // Use the last point if offset pose is not calculable
  }();

  // Extract the collision check section from the full path
  PathWithLaneId collision_check_section;
  if (shift_start_idx < collision_check_end_idx) {
    collision_check_section.points.assign(
      full_path.points.begin() + shift_start_idx,
      full_path.points.begin() + collision_check_end_idx + 1);
  }

  return collision_check_section;
}

}  // namespace autoware::behavior_path_planner::start_planner_utils
