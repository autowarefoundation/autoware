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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_SEARCHER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_SEARCHER_HPP_

#include "autoware/behavior_path_goal_planner_module/goal_searcher_base.hpp"
#include "autoware/behavior_path_planner_common/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.hpp"

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::universe_utils::LinearRing2d;
using BasicPolygons2d = std::vector<lanelet::BasicPolygon2d>;

class GoalSearcher : public GoalSearcherBase
{
public:
  GoalSearcher(const GoalPlannerParameters & parameters, const LinearRing2d & vehicle_footprint);

  GoalCandidates search(
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const std::shared_ptr<const PlannerData> & planner_data) override;
  void update(
    GoalCandidates & goal_candidates,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const std::shared_ptr<const PlannerData> & planner_data) const override;

  // todo(kosuke55): Functions for this specific use should not be in the interface,
  // so it is better to consider interface design when we implement other goal searchers.
  GoalCandidate getClosetGoalCandidateAlongLanes(
    const GoalCandidates & goal_candidates,
    const std::shared_ptr<const PlannerData> & planner_data) const override;
  bool isSafeGoalWithMarginScaleFactor(
    const GoalCandidate & goal_candidate, const double margin_scale_factor,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const std::shared_ptr<const PlannerData> & planner_data) const override;

private:
  void countObjectsToAvoid(
    GoalCandidates & goal_candidates, const PredictedObjects & objects,
    const std::shared_ptr<const PlannerData> & planner_data) const;
  void createAreaPolygons(
    std::vector<Pose> original_search_poses,
    const std::shared_ptr<const PlannerData> & planner_data);
  bool checkCollision(
    const Pose & pose, const PredictedObjects & objects,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map) const;
  bool checkCollisionWithLongitudinalDistance(
    const Pose & ego_pose, const PredictedObjects & objects,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const std::shared_ptr<const PlannerData> & planner_data) const;
  BasicPolygons2d getNoParkingAreaPolygons(const lanelet::ConstLanelets & lanes) const;
  BasicPolygons2d getNoStoppingAreaPolygons(const lanelet::ConstLanelets & lanes) const;
  bool isInAreas(const LinearRing2d & footprint, const BasicPolygons2d & areas) const;

  LinearRing2d vehicle_footprint_{};
  bool left_side_parking_{true};
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_SEARCHER_HPP_
