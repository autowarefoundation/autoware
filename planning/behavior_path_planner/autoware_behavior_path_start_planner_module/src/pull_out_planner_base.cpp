// Copyright 2024 Tier IV, Inc.
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

#include "autoware/behavior_path_start_planner_module/pull_out_planner_base.hpp"

namespace autoware::behavior_path_planner
{
bool PullOutPlannerBase::isPullOutPathCollided(
  autoware::behavior_path_planner::PullOutPath & pull_out_path,
  double collision_check_distance_from_end) const
{
  universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // check for collisions
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto pull_out_lanes = start_planner_utils::getPullOutLanes(
    planner_data_, planner_data_->parameters.backward_path_length + parameters_.max_back_distance);
  const auto & vehicle_footprint = vehicle_info_.createFootprint();
  // extract stop objects in pull out lane for collision check
  const auto stop_objects = utils::path_safety_checker::filterObjectsByVelocity(
    *dynamic_objects, parameters_.th_moving_object_velocity);
  auto [pull_out_lane_stop_objects, others] = utils::path_safety_checker::separateObjectsByLanelets(
    stop_objects, pull_out_lanes,
    [](const auto & obj, const auto & lane, const auto yaw_threshold) {
      return utils::path_safety_checker::isPolygonOverlapLanelet(obj, lane, yaw_threshold);
    });
  utils::path_safety_checker::filterObjectsByClass(
    pull_out_lane_stop_objects, parameters_.object_types_to_check_for_path_generation);

  const auto collision_check_section_path =
    autoware::behavior_path_planner::start_planner_utils::extractCollisionCheckSection(
      pull_out_path, collision_check_distance_from_end);
  if (!collision_check_section_path) return true;

  return utils::checkCollisionBetweenPathFootprintsAndObjects(
    vehicle_footprint_, collision_check_section_path.value(), pull_out_lane_stop_objects,
    collision_check_margin_);
};
}  // namespace autoware::behavior_path_planner
