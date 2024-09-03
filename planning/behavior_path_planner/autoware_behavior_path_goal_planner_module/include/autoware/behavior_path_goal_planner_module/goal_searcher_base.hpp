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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_SEARCHER_BASE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_SEARCHER_BASE_HPP_

#include "autoware/behavior_path_goal_planner_module/goal_planner_parameters.hpp"
#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/occupancy_grid_based_collision_detector/occupancy_grid_based_collision_detector.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::universe_utils::MultiPolygon2d;
using geometry_msgs::msg::Pose;

struct GoalCandidate
{
  Pose goal_pose{};
  double distance_from_original_goal{0.0};
  double lateral_offset{0.0};
  size_t id{0};
  bool is_safe{true};
  size_t num_objects_to_avoid{0};
};
using GoalCandidates = std::vector<GoalCandidate>;

class GoalSearcherBase
{
public:
  explicit GoalSearcherBase(const GoalPlannerParameters & parameters) { parameters_ = parameters; }
  virtual ~GoalSearcherBase() = default;

  void setReferenceGoal(const Pose & reference_goal_pose)
  {
    reference_goal_pose_ = reference_goal_pose;
  }
  const Pose & getReferenceGoal() const { return reference_goal_pose_; }

  MultiPolygon2d getAreaPolygons() const { return area_polygons_; }
  virtual GoalCandidates search(const std::shared_ptr<const PlannerData> & planner_data) = 0;
  virtual void update(
    [[maybe_unused]] GoalCandidates & goal_candidates,
    [[maybe_unused]] const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    [[maybe_unused]] const std::shared_ptr<const PlannerData> & planner_data,
    [[maybe_unused]] const PredictedObjects & objects) const
  {
    return;
  }
  virtual GoalCandidate getClosetGoalCandidateAlongLanes(
    const GoalCandidates & goal_candidates,
    const std::shared_ptr<const PlannerData> & planner_data) const = 0;
  virtual bool isSafeGoalWithMarginScaleFactor(
    const GoalCandidate & goal_candidate, const double margin_scale_factor,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const std::shared_ptr<const PlannerData> & planner_data,
    const PredictedObjects & objects) const = 0;

protected:
  GoalPlannerParameters parameters_{};
  Pose reference_goal_pose_{};
  MultiPolygon2d area_polygons_{};
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_SEARCHER_BASE_HPP_
