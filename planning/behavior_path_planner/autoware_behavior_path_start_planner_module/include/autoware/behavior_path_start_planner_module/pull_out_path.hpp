// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__PULL_OUT_PATH_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__PULL_OUT_PATH_HPP_

#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using tier4_planning_msgs::msg::PathWithLaneId;
struct PullOutPath
{
  std::vector<PathWithLaneId> partial_paths{};
  // accelerate with constant acceleration to the target velocity
  std::vector<std::pair<double, double>> pairs_terminal_velocity_and_accel{};
  Pose start_pose{};
  Pose end_pose{};
};
}  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__PULL_OUT_PATH_HPP_
