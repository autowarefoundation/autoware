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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OUT__GEOMETRIC_PULL_OUT_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OUT__GEOMETRIC_PULL_OUT_HPP_

#include "behavior_path_planner/scene_module/pull_out/pull_out_path.hpp"
#include "behavior_path_planner/scene_module/pull_out/pull_out_planner_base.hpp"
#include "behavior_path_planner/scene_module/utils/geometric_parallel_parking.hpp"

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

namespace behavior_path_planner
{
class GeometricPullOut : public PullOutPlannerBase
{
public:
  explicit GeometricPullOut(
    rclcpp::Node & node, const PullOutParameters & parameters,
    const ParallelParkingParameters & parallel_parking_parameters);

  PlannerType getPlannerType() override { return PlannerType::GEOMETRIC; };
  boost::optional<PullOutPath> plan(Pose start_pose, Pose goal_pose) override;

  GeometricParallelParking planner_;
  ParallelParkingParameters parallel_parking_parameters_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OUT__GEOMETRIC_PULL_OUT_HPP_
