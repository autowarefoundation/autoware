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

#include "behavior_path_planner/scene_module/pull_over/geometric_pull_over.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/pull_over/util.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
GeometricPullOver::GeometricPullOver(
  rclcpp::Node & node, const PullOverParameters & parameters,
  const ParallelParkingParameters & parallel_parking_parameters,
  const LaneDepartureChecker & lane_departure_checker,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
  const bool is_forward)
: PullOverPlannerBase{node, parameters},
  parallel_parking_parameters_{parallel_parking_parameters},
  lane_departure_checker_{lane_departure_checker},
  occupancy_grid_map_{occupancy_grid_map},
  is_forward_{is_forward}
{
}

boost::optional<PullOverPath> GeometricPullOver::plan(const Pose & goal_pose)
{
  const auto & route_handler = planner_data_->route_handler;

  // prepare road nad shoulder lanes
  const auto road_lanes = util::getExtendedCurrentLanes(planner_data_);
  const auto shoulder_lanes = pull_over_utils::getPullOverLanes(*route_handler);
  if (road_lanes.empty() || shoulder_lanes.empty()) {
    return {};
  }
  auto lanes = road_lanes;
  lanes.insert(lanes.end(), shoulder_lanes.begin(), shoulder_lanes.end());

  // todo: set param only once
  planner_.setData(planner_data_, parallel_parking_parameters_);
  const bool found_valid_path =
    planner_.planPullOver(goal_pose, road_lanes, shoulder_lanes, is_forward_);
  if (!found_valid_path) {
    return {};
  }

  const auto arc_path = planner_.getArcPath();

  // check lane departure with road and shoulder lanes
  if (lane_departure_checker_.checkPathWillLeaveLane(lanes, arc_path)) return {};

  // collision check
  if (parameters_.use_occupancy_grid || !occupancy_grid_map_) {
    const bool check_out_of_range = false;
    if (occupancy_grid_map_->hasObstacleOnPath(arc_path, check_out_of_range)) {
      return {};
    }
  }
  if (parameters_.use_object_recognition) {
    if (util::checkCollisionBetweenPathFootprintsAndObjects(
          vehicle_footprint_, arc_path, *(planner_data_->dynamic_object),
          parameters_.object_recognition_collision_check_margin)) {
      return {};
    }
  }

  PullOverPath pull_over_path{};
  pull_over_path.partial_paths = planner_.getPaths();
  pull_over_path.start_pose = planner_.getStartPose();
  pull_over_path.end_pose = planner_.getArcEndPose();

  return pull_over_path;
}
}  // namespace behavior_path_planner
