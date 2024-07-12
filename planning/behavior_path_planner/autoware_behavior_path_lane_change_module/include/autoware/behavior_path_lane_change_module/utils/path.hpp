// Copyright 2021 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__PATH_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__PATH_HPP_

#include "autoware/behavior_path_lane_change_module/utils/data_structs.hpp"
#include "autoware/behavior_path_planner_common/turn_signal_decider.hpp"
#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <vector>

namespace autoware::behavior_path_planner::lane_change
{
using autoware::behavior_path_planner::TurnSignalInfo;
using tier4_planning_msgs::msg::PathWithLaneId;
struct Path
{
  PathWithLaneId path{};
  ShiftedPath shifted_path{};
  Info info{};
};

struct Status
{
  Path lane_change_path{};
  bool is_safe{false};
  bool is_valid_path{false};
  double start_distance{0.0};
};

}  // namespace autoware::behavior_path_planner::lane_change

namespace autoware::behavior_path_planner
{
using LaneChangePath = lane_change::Path;
using LaneChangePaths = std::vector<lane_change::Path>;
using LaneChangeStatus = lane_change::Status;
}  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__PATH_HPP_
