
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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__START_PLANNER__START_PLANNER_PARAMETERS_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__START_PLANNER__START_PLANNER_PARAMETERS_HPP_

#include "behavior_path_planner/utils/geometric_parallel_parking/geometric_parallel_parking.hpp"

#include <string>
#include <vector>

namespace behavior_path_planner
{
struct StartPlannerParameters
{
  double th_arrived_distance;
  double th_stopped_velocity;
  double th_stopped_time;
  double th_blinker_on_lateral_offset;
  double collision_check_margin;
  double collision_check_distance_from_end;
  // shift pull out
  bool enable_shift_pull_out;
  double shift_pull_out_velocity;
  int pull_out_sampling_num;
  double minimum_shift_pull_out_distance;
  double maximum_lateral_jerk;
  double minimum_lateral_jerk;
  double deceleration_interval;
  // geometric pull out
  bool enable_geometric_pull_out;
  bool divide_pull_out_path;
  ParallelParkingParameters parallel_parking_parameters;
  // search start pose backward
  std::string search_priority;  // "efficient_path" or "short_back_distance"
  bool enable_back;
  double backward_velocity;
  double max_back_distance;
  double backward_search_resolution;
  double backward_path_update_duration;
  double ignore_distance_from_lane_end;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__START_PLANNER__START_PLANNER_PARAMETERS_HPP_
