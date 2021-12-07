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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_PATH_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_PATH_HPP_

#include "behavior_path_planner/path_shifter/path_shifter.hpp"

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
struct PullOverPath
{
  PathWithLaneId path;
  ShiftedPath shifted_path;
  ShiftPoint shift_point;
  double acceleration{0.0};
  double preparation_length{0.0};
  double pull_over_length{0.0};
};
}  // namespace behavior_path_planner
#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_PATH_HPP_
