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

#include "behavior_path_start_planner_module/freespace_pull_out.hpp"

#include "behavior_path_planner_common/utils/parking_departure/utils.hpp"
#include "behavior_path_planner_common/utils/path_utils.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"
#include "behavior_path_start_planner_module/util.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
FreespacePullOut::FreespacePullOut(
  rclcpp::Node & node, const StartPlannerParameters & parameters,
  const vehicle_info_util::VehicleInfo & vehicle_info)
: PullOutPlannerBase{node, parameters}, velocity_{parameters.freespace_planner_velocity}
{
  freespace_planning_algorithms::VehicleShape vehicle_shape(
    vehicle_info, parameters.vehicle_shape_margin);
  if (parameters.freespace_planner_algorithm == "astar") {
    use_back_ = parameters.astar_parameters.use_back;
    planner_ = std::make_unique<AstarSearch>(
      parameters.freespace_planner_common_parameters, vehicle_shape, parameters.astar_parameters);
  } else if (parameters.freespace_planner_algorithm == "rrtstar") {
    use_back_ = true;  // no option for disabling back in rrtstar
    planner_ = std::make_unique<RRTStar>(
      parameters.freespace_planner_common_parameters, vehicle_shape,
      parameters.rrt_star_parameters);
  }
}

std::optional<PullOutPath> FreespacePullOut::plan(const Pose & start_pose, const Pose & end_pose)
{
  const auto & route_handler = planner_data_->route_handler;
  const double backward_path_length = planner_data_->parameters.backward_path_length;
  const double forward_path_length = planner_data_->parameters.forward_path_length;

  planner_->setMap(*planner_data_->costmap);

  const bool found_path = planner_->makePlan(start_pose, end_pose);
  if (!found_path) {
    return {};
  }

  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data_, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);
  // find candidate paths
  const auto pull_out_lanes =
    start_planner_utils::getPullOutLanes(planner_data_, backward_path_length);
  const auto lanes = utils::combineLanelets(road_lanes, pull_out_lanes);

  const PathWithLaneId path =
    utils::convertWayPointsToPathWithLaneId(planner_->getWaypoints(), velocity_, lanes);

  const auto reverse_indices = utils::getReversingIndices(path);
  std::vector<PathWithLaneId> partial_paths = utils::dividePath(path, reverse_indices);

  // remove points which are near the end pose
  PathWithLaneId & last_path = partial_paths.back();
  const double th_end_distance = 1.0;
  for (auto it = last_path.points.begin(); it != last_path.points.end(); ++it) {
    const size_t index = std::distance(last_path.points.begin(), it);
    if (index == 0) continue;
    const double distance =
      tier4_autoware_utils::calcDistance2d(end_pose.position, it->point.pose.position);
    if (distance < th_end_distance) {
      last_path.points.erase(it, last_path.points.end());
      break;
    }
  }

  // push back generate road lane path between end pose and goal pose to last path
  const Pose goal_pose = route_handler->getGoalPose();
  constexpr double offset_from_end_pose = 1.0;
  const auto arc_position_end = lanelet::utils::getArcCoordinates(road_lanes, end_pose);
  const double s_start = std::max(arc_position_end.length + offset_from_end_pose, 0.0);
  const auto path_end_info = behavior_path_planner::utils::parking_departure::calcEndArcLength(
    s_start, forward_path_length, road_lanes, goal_pose);
  const double s_end = path_end_info.first;
  const bool path_terminal_is_goal = path_end_info.second;

  const auto road_center_line_path = route_handler->getCenterLinePath(road_lanes, s_start, s_end);
  last_path = utils::resamplePathWithSpline(
    utils::combinePath(last_path, road_center_line_path), parameters_.center_line_path_interval);

  const double original_terminal_velocity = last_path.points.back().point.longitudinal_velocity_mps;
  utils::correctDividedPathVelocity(partial_paths);
  if (!path_terminal_is_goal) {
    // not need to stop at terminal
    last_path.points.back().point.longitudinal_velocity_mps = original_terminal_velocity;
  }

  PullOutPath pull_out_path{};
  pull_out_path.partial_paths = partial_paths;
  pull_out_path.start_pose = start_pose;
  pull_out_path.end_pose = end_pose;

  return pull_out_path;
}
}  // namespace behavior_path_planner
