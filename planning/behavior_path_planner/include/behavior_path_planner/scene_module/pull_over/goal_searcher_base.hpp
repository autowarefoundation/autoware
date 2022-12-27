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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__GOAL_SEARCHER_BASE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__GOAL_SEARCHER_BASE_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/parameters.hpp"
#include "behavior_path_planner/scene_module/pull_over/pull_over_parameters.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <limits>
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

  bool operator<(const GoalCandidate & other) const noexcept
  {
    const double diff = distance_from_original_goal - other.distance_from_original_goal;
    constexpr double eps = 0.01;
    if (std::abs(diff) < eps) {
      return lateral_offset < other.lateral_offset;
    }

    return distance_from_original_goal < other.distance_from_original_goal;
  }
};
using GoalCandidates = std::vector<GoalCandidate>;

class GoalSearcherBase
{
public:
  explicit GoalSearcherBase(const PullOverParameters & parameters) { parameters_ = parameters; }
  virtual ~GoalSearcherBase() = default;

  void setPlannerData(const std::shared_ptr<const PlannerData> & planner_data)
  {
    planner_data_ = planner_data;
  }

  MultiPolygon2d getAreaPolygons() { return area_polygons_; }
  virtual GoalCandidates search(const Pose & original_goal_pose) = 0;
  virtual void update([[maybe_unused]] GoalCandidates & goal_candidates) const { return; }

protected:
  PullOverParameters parameters_;
  std::shared_ptr<const PlannerData> planner_data_;
  MultiPolygon2d area_polygons_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__GOAL_SEARCHER_BASE_HPP_
