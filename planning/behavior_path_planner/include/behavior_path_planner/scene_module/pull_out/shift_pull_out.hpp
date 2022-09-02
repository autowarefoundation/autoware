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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OUT__SHIFT_PULL_OUT_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OUT__SHIFT_PULL_OUT_HPP_

#include "behavior_path_planner/scene_module/pull_out/pull_out_path.hpp"
#include "behavior_path_planner/scene_module/pull_out/pull_out_planner_base.hpp"

#include <lane_departure_checker/lane_departure_checker.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
using lane_departure_checker::LaneDepartureChecker;

class ShiftPullOut : public PullOutPlannerBase
{
public:
  explicit ShiftPullOut(
    rclcpp::Node & node, const PullOutParameters & parameters,
    std::shared_ptr<LaneDepartureChecker> & lane_departure_checker);

  PlannerType getPlannerType() override { return PlannerType::SHIFT; };
  boost::optional<PullOutPath> plan(Pose start_pose, Pose goal_pose) override;

  std::vector<PullOutPath> calcPullOutPaths(
    const RouteHandler & route_handler, const lanelet::ConstLanelets & road_lanes,
    const lanelet::ConstLanelets & shoulder_lanes, const Pose & start_pose, const Pose & goal_pose,
    const BehaviorPathPlannerParameters & common_parameter,
    const behavior_path_planner::PullOutParameters & parameter);

  bool hasEnoughDistance(
    const double pull_out_total_distance, const lanelet::ConstLanelets & current_lanes,
    const Pose & current_pose, const bool isInGoalRouteSection, const Pose & goal_pose);

  std::shared_ptr<LaneDepartureChecker> lane_departure_checker_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OUT__SHIFT_PULL_OUT_HPP_
