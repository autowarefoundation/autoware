// Copyright 2022 Autoware Foundation
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

#ifndef UTIL_HPP_
#define UTIL_HPP_

#include "operation_mode_transition_manager/msg/operation_mode_transition_manager_debug.hpp"

using DebugInfo = operation_mode_transition_manager::msg::OperationModeTransitionManagerDebug;

void setAllOk(DebugInfo & debug_info)
{
  debug_info.is_all_ok = true;
  debug_info.engage_allowed_for_stopped_vehicle = true;
  debug_info.large_acceleration_ok = true;
  debug_info.large_lateral_acceleration_diff_ok = true;
  debug_info.large_lateral_acceleration_ok = true;
  debug_info.lateral_deviation_ok = true;
  debug_info.speed_lower_deviation_ok = true;
  debug_info.speed_upper_deviation_ok = true;
  debug_info.stop_ok = true;
  debug_info.trajectory_available_ok = true;
  debug_info.yaw_deviation_ok = true;
}

#endif  // UTIL_HPP_
