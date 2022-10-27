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

#include "behavior_path_planner/scene_module/pull_over/goal_searcher.hpp"

#include "behavior_path_planner/scene_module/pull_over/util.hpp"

#include <boost/optional.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
using lane_departure_checker::LaneDepartureChecker;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::inverseTransformPose;

GoalSearcher::GoalSearcher(
  const PullOverParameters & parameters, const LinearRing2d & vehicle_footprint,
  const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map)
: GoalSearcherBase{parameters},
  vehicle_footprint_{vehicle_footprint},
  occupancy_grid_map_{occupancy_grid_map}
{
}

std::vector<GoalCandidate> GoalSearcher::search(const Pose & original_goal_pose)
{
  std::vector<GoalCandidate> goal_candidates{};

  const auto pull_over_lanes = pull_over_utils::getPullOverLanes(*(planner_data_->route_handler));
  auto lanes = util::getExtendedCurrentLanes(planner_data_);
  lanes.insert(lanes.end(), pull_over_lanes.begin(), pull_over_lanes.end());

  const auto shoulder_lane_objects =
    util::filterObjectsByLanelets(*(planner_data_->dynamic_object), pull_over_lanes);

  for (double dx = -parameters_.backward_goal_search_length;
       dx <= parameters_.forward_goal_search_length; dx += parameters_.goal_search_interval) {
    // search goal_pose in lateral direction
    Pose search_pose{};
    bool found_lateral_no_collision_pose = false;
    double lateral_offset = 0.0;
    for (double dy = 0; dy <= parameters_.max_lateral_offset;
         dy += parameters_.lateral_offset_interval) {
      lateral_offset = dy;
      search_pose = calcOffsetPose(original_goal_pose, dx, -dy, 0);

      const auto & transformed_vehicle_footprint =
        transformVector(vehicle_footprint_, tier4_autoware_utils::pose2transform(search_pose));
      if (LaneDepartureChecker::isOutOfLane(lanes, transformed_vehicle_footprint)) {
        continue;
      }

      if (checkCollision(search_pose)) {
        continue;
      }

      // if finding objects near the search pose,
      // shift search_pose in lateral direction one more
      // because collision may be detected on other path points
      if (dy > 0) {
        search_pose = calcOffsetPose(search_pose, 0, -parameters_.lateral_offset_interval, 0);
      }

      found_lateral_no_collision_pose = true;
      break;
    }
    if (!found_lateral_no_collision_pose) continue;

    constexpr bool filter_inside = true;
    const auto target_objects = pull_over_utils::filterObjectsByLateralDistance(
      search_pose, planner_data_->parameters.vehicle_width, shoulder_lane_objects,
      parameters_.object_recognition_collision_check_margin, filter_inside);
    if (checkCollisionWithLongitudinalDistance(search_pose, target_objects)) {
      continue;
    }

    GoalCandidate goal_candidate{};
    goal_candidate.goal_pose = search_pose;
    goal_candidate.lateral_offset = lateral_offset;
    // use longitudinal_distance as distance_from_original_goal
    // TODO(kosuke55): use arc length for curve lane
    goal_candidate.distance_from_original_goal =
      std::abs(inverseTransformPose(search_pose, original_goal_pose).position.x);

    goal_candidates.push_back(goal_candidate);
  }
  // Sort with distance from original goal
  std::sort(goal_candidates.begin(), goal_candidates.end());

  return goal_candidates;
}

bool GoalSearcher::checkCollision(const Pose & pose) const
{
  if (parameters_.use_occupancy_grid) {
    const Pose pose_grid_coords = global2local(occupancy_grid_map_->getMap(), pose);
    const auto idx = pose2index(
      occupancy_grid_map_->getMap(), pose_grid_coords, occupancy_grid_map_->getParam().theta_size);
    const bool check_out_of_range = false;
    if (occupancy_grid_map_->detectCollision(idx, check_out_of_range)) {
      return true;
    }
  }

  if (parameters_.use_object_recognition) {
    if (util::checkCollisionBetweenFootprintAndObjects(
          vehicle_footprint_, pose, *(planner_data_->dynamic_object),
          parameters_.object_recognition_collision_check_margin)) {
      return true;
    }
  }
  return false;
}

bool GoalSearcher::checkCollisionWithLongitudinalDistance(
  const Pose & ego_pose, const PredictedObjects & dynamic_objects) const
{
  if (parameters_.use_occupancy_grid && parameters_.use_occupancy_grid_for_longitudinal_margin) {
    constexpr bool check_out_of_range = false;
    const double offset = std::max(
      parameters_.longitudinal_margin - parameters_.occupancy_grid_collision_check_margin, 0.0);

    // check forward collision
    const Pose ego_pose_moved_forward = calcOffsetPose(ego_pose, offset, 0, 0);
    const Pose forward_pose_grid_coords =
      global2local(occupancy_grid_map_->getMap(), ego_pose_moved_forward);
    const auto forward_idx = pose2index(
      occupancy_grid_map_->getMap(), forward_pose_grid_coords,
      occupancy_grid_map_->getParam().theta_size);
    if (occupancy_grid_map_->detectCollision(forward_idx, check_out_of_range)) {
      return true;
    }

    // check backward collision
    const Pose ego_pose_moved_backward = calcOffsetPose(ego_pose, -offset, 0, 0);
    const Pose backward_pose_grid_coords =
      global2local(occupancy_grid_map_->getMap(), ego_pose_moved_backward);
    const auto backward_idx = pose2index(
      occupancy_grid_map_->getMap(), backward_pose_grid_coords,
      occupancy_grid_map_->getParam().theta_size);
    if (occupancy_grid_map_->detectCollision(backward_idx, check_out_of_range)) {
      return true;
    }
  }

  if (parameters_.use_object_recognition) {
    if (
      util::calcLongitudinalDistanceFromEgoToObjects(
        ego_pose, planner_data_->parameters.base_link2front,
        planner_data_->parameters.base_link2rear,
        dynamic_objects) < parameters_.longitudinal_margin) {
      return true;
    }
  }
  return false;
}

}  // namespace behavior_path_planner
