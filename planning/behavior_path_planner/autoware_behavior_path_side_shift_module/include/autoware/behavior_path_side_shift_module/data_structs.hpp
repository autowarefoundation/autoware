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

#ifndef AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__DATA_STRUCTS_HPP_

#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <tier4_planning_msgs/msg/lateral_offset.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using tier4_planning_msgs::msg::LateralOffset;

enum class SideShiftStatus { STOP = 0, BEFORE_SHIFT, SHIFTING, AFTER_SHIFT };

struct SideShiftParameters
{
  double time_to_start_shifting;
  double min_distance_to_start_shifting;
  double shifting_lateral_jerk;
  double min_shifting_distance;
  double min_shifting_speed;
  double drivable_area_resolution;
  double drivable_area_width;
  double drivable_area_height;
  double shift_request_time_limit;
  bool publish_debug_marker;
};

struct SideShiftDebugData
{
  std::shared_ptr<PathShifter> path_shifter{};
  ShiftLineArray shift_lines{};
  double current_request{0.0};
};

}  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_SIDE_SHIFT_MODULE__DATA_STRUCTS_HPP_
