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

#ifndef BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__PATH_HPP_
#define BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__PATH_HPP_

#include "behavior_path_lane_change_module/utils/data_structs.hpp"
#include "behavior_path_planner_common/turn_signal_decider.hpp"
#include "behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using behavior_path_planner::TurnSignalInfo;
struct LaneChangePath
{
  PathWithLaneId path{};
  ShiftedPath shifted_path{};
  PathWithLaneId prev_path{};
  LaneChangeInfo info{};
};
using LaneChangePaths = std::vector<LaneChangePath>;

struct LaneChangeStatus
{
  PathWithLaneId lane_follow_path{};
  LaneChangePath lane_change_path{};
  lanelet::ConstLanelets current_lanes{};
  lanelet::ConstLanelets target_lanes{};
  std::vector<lanelet::Id> lane_follow_lane_ids{};
  std::vector<lanelet::Id> lane_change_lane_ids{};
  bool is_safe{false};
  bool is_valid_path{true};
  double start_distance{0.0};
};

}  // namespace behavior_path_planner
#endif  // BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__PATH_HPP_
