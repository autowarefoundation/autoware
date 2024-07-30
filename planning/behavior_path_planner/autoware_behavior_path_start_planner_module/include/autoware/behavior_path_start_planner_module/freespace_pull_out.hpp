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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__FREESPACE_PULL_OUT_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__FREESPACE_PULL_OUT_HPP_

#include "autoware/behavior_path_start_planner_module/pull_out_planner_base.hpp"
#include "autoware/universe_utils/system/time_keeper.hpp"

#include <autoware/freespace_planning_algorithms/abstract_algorithm.hpp>
#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>

namespace autoware::behavior_path_planner
{
using autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm;
using autoware::freespace_planning_algorithms::AstarSearch;
using autoware::freespace_planning_algorithms::RRTStar;

class FreespacePullOut : public PullOutPlannerBase
{
public:
  FreespacePullOut(
    rclcpp::Node & node, const StartPlannerParameters & parameters,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    std::shared_ptr<universe_utils::TimeKeeper> time_keeper);

  PlannerType getPlannerType() const override { return PlannerType::FREESPACE; }

  std::optional<PullOutPath> plan(
    const Pose & start_pose, const Pose & end_pose, PlannerDebugData & planner_debug_data) override;

protected:
  std::unique_ptr<AbstractPlanningAlgorithm> planner_;
  double velocity_;
  bool use_back_;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__FREESPACE_PULL_OUT_HPP_
