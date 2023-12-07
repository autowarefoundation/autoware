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

#ifndef BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_SEARCHER_BASE_HPP_
#define BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_SEARCHER_BASE_HPP_

#include "behavior_path_goal_planner_module/goal_planner_parameters.hpp"
#include "behavior_path_planner_common/data_manager.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
using geometry_msgs::msg::Pose;
using tier4_autoware_utils::MultiPolygon2d;

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

  void setPlannerData(const std::shared_ptr<const PlannerData> & planner_data)
  {
    planner_data_ = planner_data;
  }

  MultiPolygon2d getAreaPolygons() { return area_polygons_; }
  virtual GoalCandidates search() = 0;
  virtual void update([[maybe_unused]] GoalCandidates & goal_candidates) const { return; }
  virtual GoalCandidate getClosetGoalCandidateAlongLanes(
    const GoalCandidates & goal_candidates) const = 0;

protected:
  GoalPlannerParameters parameters_{};
  std::shared_ptr<const PlannerData> planner_data_{nullptr};
  Pose reference_goal_pose_{};
  MultiPolygon2d area_polygons_{};
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_SEARCHER_BASE_HPP_
