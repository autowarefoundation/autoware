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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_PATH_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_PATH_HPP_

#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using behavior_path_planner::TurnSignalInfo;
struct LaneChangePath
{
  PathWithLaneId path;
  ShiftedPath shifted_path;
  ShiftLine shift_line;
  double acceleration{0.0};
  double preparation_length{0.0};
  double lane_change_length{0.0};
  TurnSignalInfo turn_signal_info;
};
using LaneChangePaths = std::vector<LaneChangePath>;

}  // namespace behavior_path_planner
#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_PATH_HPP_
