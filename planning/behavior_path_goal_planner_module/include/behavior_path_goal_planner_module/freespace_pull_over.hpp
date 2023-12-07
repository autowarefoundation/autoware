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

#ifndef BEHAVIOR_PATH_GOAL_PLANNER_MODULE__FREESPACE_PULL_OVER_HPP_
#define BEHAVIOR_PATH_GOAL_PLANNER_MODULE__FREESPACE_PULL_OVER_HPP_

#include "behavior_path_goal_planner_module/pull_over_planner_base.hpp"

#include <freespace_planning_algorithms/abstract_algorithm.hpp>
#include <freespace_planning_algorithms/astar_search.hpp>
#include <freespace_planning_algorithms/rrtstar.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
using freespace_planning_algorithms::AbstractPlanningAlgorithm;
using freespace_planning_algorithms::AstarSearch;
using freespace_planning_algorithms::RRTStar;

class FreespacePullOver : public PullOverPlannerBase
{
public:
  FreespacePullOver(
    rclcpp::Node & node, const GoalPlannerParameters & parameters,
    const vehicle_info_util::VehicleInfo & vehicle_info);

  PullOverPlannerType getPlannerType() const override { return PullOverPlannerType::FREESPACE; }

  std::optional<PullOverPath> plan(const Pose & goal_pose) override;

protected:
  std::unique_ptr<AbstractPlanningAlgorithm> planner_;
  double velocity_;
  bool use_back_;
  bool left_side_parking_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_GOAL_PLANNER_MODULE__FREESPACE_PULL_OVER_HPP_
