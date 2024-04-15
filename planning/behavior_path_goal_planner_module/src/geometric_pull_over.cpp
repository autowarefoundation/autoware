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

#include "behavior_path_goal_planner_module/geometric_pull_over.hpp"

#include "behavior_path_goal_planner_module/util.hpp"
#include "behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
GeometricPullOver::GeometricPullOver(
  rclcpp::Node & node, const GoalPlannerParameters & parameters,
  const LaneDepartureChecker & lane_departure_checker,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
  const bool is_forward)
: PullOverPlannerBase{node, parameters},
  parallel_parking_parameters_{parameters.parallel_parking_parameters},
  lane_departure_checker_{lane_departure_checker},
  occupancy_grid_map_{occupancy_grid_map},
  is_forward_{is_forward},
  left_side_parking_{parameters.parking_policy == ParkingPolicy::LEFT_SIDE}
{
  planner_.setParameters(parallel_parking_parameters_);
}

std::optional<PullOverPath> GeometricPullOver::plan(const Pose & goal_pose)
{
  const auto & route_handler = planner_data_->route_handler;

  // prepare road nad shoulder lanes
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data_, parameters_.backward_goal_search_length, parameters_.forward_goal_search_length,
    /*forward_only_in_route*/ false);
  const auto pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *route_handler, left_side_parking_, parameters_.backward_goal_search_length,
    parameters_.forward_goal_search_length);
  if (road_lanes.empty() || pull_over_lanes.empty()) {
    return {};
  }
  const auto lanes = utils::combineLanelets(road_lanes, pull_over_lanes);

  const auto & p = parallel_parking_parameters_;
  const double max_steer_angle =
    is_forward_ ? p.forward_parking_max_steer_angle : p.backward_parking_max_steer_angle;
  planner_.setTurningRadius(planner_data_->parameters, max_steer_angle);
  planner_.setPlannerData(planner_data_);

  const bool found_valid_path =
    planner_.planPullOver(goal_pose, road_lanes, pull_over_lanes, is_forward_, left_side_parking_);
  if (!found_valid_path) {
    return {};
  }

  const auto arc_path = planner_.getArcPath();

  // check lane departure with road and shoulder lanes
  if (lane_departure_checker_.checkPathWillLeaveLane(lanes, arc_path)) return {};

  PullOverPath pull_over_path{};
  pull_over_path.type = getPlannerType();
  pull_over_path.partial_paths = planner_.getPaths();
  pull_over_path.pairs_terminal_velocity_and_accel = planner_.getPairsTerminalVelocityAndAccel();
  pull_over_path.start_pose = planner_.getStartPose();
  pull_over_path.end_pose = planner_.getArcEndPose();

  return pull_over_path;
}
}  // namespace behavior_path_planner
