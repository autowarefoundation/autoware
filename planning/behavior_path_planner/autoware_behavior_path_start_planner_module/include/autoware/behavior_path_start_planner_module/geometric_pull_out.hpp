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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__GEOMETRIC_PULL_OUT_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__GEOMETRIC_PULL_OUT_HPP_

#include "autoware/behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp"
#include "autoware/behavior_path_start_planner_module/pull_out_path.hpp"
#include "autoware/behavior_path_start_planner_module/pull_out_planner_base.hpp"
#include "autoware/universe_utils/system/time_keeper.hpp"

#include <autoware/lane_departure_checker/lane_departure_checker.hpp>

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>

namespace autoware::behavior_path_planner
{
class GeometricPullOut : public PullOutPlannerBase
{
public:
  explicit GeometricPullOut(
    rclcpp::Node & node, const StartPlannerParameters & parameters,
    const std::shared_ptr<autoware::lane_departure_checker::LaneDepartureChecker>
      lane_departure_checker,
    std::shared_ptr<universe_utils::TimeKeeper> time_keeper);

  PlannerType getPlannerType() const override { return PlannerType::GEOMETRIC; };
  std::optional<PullOutPath> plan(
    const Pose & start_pose, const Pose & goal_pose,
    PlannerDebugData & planner_debug_data) override;

  GeometricParallelParking planner_;
  ParallelParkingParameters parallel_parking_parameters_;
  std::shared_ptr<autoware::lane_departure_checker::LaneDepartureChecker> lane_departure_checker_;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__GEOMETRIC_PULL_OUT_HPP_
