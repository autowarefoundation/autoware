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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__SHIFT_PULL_OUT_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__SHIFT_PULL_OUT_HPP_

#include "autoware/behavior_path_start_planner_module/pull_out_path.hpp"
#include "autoware/behavior_path_start_planner_module/pull_out_planner_base.hpp"
#include "autoware/universe_utils/system/time_keeper.hpp"

#include <autoware/lane_departure_checker/lane_departure_checker.hpp>

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::lane_departure_checker::LaneDepartureChecker;

class ShiftPullOut : public PullOutPlannerBase
{
public:
  explicit ShiftPullOut(
    rclcpp::Node & node, const StartPlannerParameters & parameters,
    std::shared_ptr<LaneDepartureChecker> & lane_departure_checker,
    std::shared_ptr<universe_utils::TimeKeeper> time_keeper);

  PlannerType getPlannerType() const override { return PlannerType::SHIFT; };
  std::optional<PullOutPath> plan(
    const Pose & start_pose, const Pose & goal_pose,
    PlannerDebugData & planner_debug_data) override;

  std::vector<PullOutPath> calcPullOutPaths(
    const RouteHandler & route_handler, const lanelet::ConstLanelets & road_lanes,
    const Pose & start_pose, const Pose & goal_pose);

  double calcBeforeShiftedArcLength(
    const PathWithLaneId & path, const double target_after_arc_length, const double dr);

  bool refineShiftedPathToStartPose(
    ShiftedPath & shifted_path, const Pose & start_pose, const Pose & end_pose,
    const double longitudinal_acc, const double lateral_acc);

  std::shared_ptr<LaneDepartureChecker> lane_departure_checker_;

private:
  // Calculate longitudinal distance based on the acceleration limit, curvature limit, and the
  // minimum distance requirement.
  double calcPullOutLongitudinalDistance(
    const double lon_acc, const double shift_time, const double shift_length,
    const double max_curvature, const double min_distance) const;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__SHIFT_PULL_OUT_HPP_
