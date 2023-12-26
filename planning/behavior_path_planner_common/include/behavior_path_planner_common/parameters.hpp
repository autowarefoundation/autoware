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

#ifndef BEHAVIOR_PATH_PLANNER_COMMON__PARAMETERS_HPP_
#define BEHAVIOR_PATH_PLANNER_COMMON__PARAMETERS_HPP_

#include <vehicle_info_util/vehicle_info_util.hpp>

struct ModuleConfigParameters
{
  bool enable_module{false};
  bool enable_rtc{false};
  bool enable_simultaneous_execution_as_approved_module{false};
  bool enable_simultaneous_execution_as_candidate_module{false};
  bool keep_last{false};
  uint8_t priority{0};
  uint8_t max_module_size{0};
};

struct BehaviorPathPlannerParameters
{
  bool verbose;
  size_t max_iteration_num{100};
  double traffic_light_signal_timeout{1.0};

  double backward_path_length;
  double forward_path_length;
  double backward_length_buffer_for_end_of_pull_over;
  double backward_length_buffer_for_end_of_pull_out;

  // common parameters
  double min_acc;
  double max_acc;

  double minimum_pull_over_length;
  double minimum_pull_out_length;
  double drivable_area_resolution;

  double refine_goal_search_radius_range;

  double turn_signal_intersection_search_distance;
  double turn_signal_intersection_angle_threshold_deg;
  double turn_signal_search_time;
  double turn_signal_minimum_search_distance;
  double turn_signal_shift_length_threshold;
  bool turn_signal_on_swerving;

  bool enable_akima_spline_first;
  bool enable_cog_on_centerline;
  double input_path_interval;
  double output_path_interval;

  double ego_nearest_dist_threshold;
  double ego_nearest_yaw_threshold;

  // vehicle info
  vehicle_info_util::VehicleInfo vehicle_info;
  double wheel_base;
  double front_overhang;
  double rear_overhang;
  double vehicle_width;
  double vehicle_length;
  double wheel_tread;
  double left_over_hang;
  double right_over_hang;
  double base_link2front;
  double base_link2rear;
};

#endif  // BEHAVIOR_PATH_PLANNER_COMMON__PARAMETERS_HPP_
