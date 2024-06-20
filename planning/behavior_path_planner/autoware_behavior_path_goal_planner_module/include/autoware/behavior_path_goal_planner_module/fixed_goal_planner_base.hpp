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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__FIXED_GOAL_PLANNER_BASE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__FIXED_GOAL_PLANNER_BASE_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>

using autoware::universe_utils::LinearRing2d;
using geometry_msgs::msg::Pose;
using tier4_planning_msgs::msg::PathWithLaneId;

namespace autoware::behavior_path_planner
{

class FixedGoalPlannerBase
{
public:
  FixedGoalPlannerBase() = default;
  virtual ~FixedGoalPlannerBase() = default;

  virtual BehaviorModuleOutput plan(
    const std::shared_ptr<const PlannerData> & planner_data) const = 0;

  void setPreviousModuleOutput(const BehaviorModuleOutput & previous_module_output)
  {
    previous_module_output_ = previous_module_output;
  }

  BehaviorModuleOutput getPreviousModuleOutput() const { return previous_module_output_; }

protected:
  BehaviorModuleOutput previous_module_output_;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__FIXED_GOAL_PLANNER_BASE_HPP_
