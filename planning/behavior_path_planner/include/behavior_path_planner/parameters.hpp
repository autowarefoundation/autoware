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

#ifndef BEHAVIOR_PATH_PLANNER__PARAMETERS_HPP_
#define BEHAVIOR_PATH_PLANNER__PARAMETERS_HPP_

#include <vehicle_info_util/vehicle_info_util.hpp>

#include <string>
#include <vector>

struct ModuleConfigParameters
{
  bool enable_module{false};
  bool enable_simultaneous_execution_as_approved_module{false};
  bool enable_simultaneous_execution_as_candidate_module{false};
  uint8_t priority{0};
  uint8_t max_module_size{0};
};

struct BehaviorPathPlannerParameters
{
  bool verbose;

  ModuleConfigParameters config_avoidance;
  ModuleConfigParameters config_avoidance_by_lc;
  ModuleConfigParameters config_pull_out;
  ModuleConfigParameters config_goal_planner;
  ModuleConfigParameters config_side_shift;
  ModuleConfigParameters config_lane_change_left;
  ModuleConfigParameters config_lane_change_right;
  ModuleConfigParameters config_ext_request_lane_change_left;
  ModuleConfigParameters config_ext_request_lane_change_right;

  double backward_path_length;
  double forward_path_length;
  double backward_length_buffer_for_end_of_lane;
  double backward_length_buffer_for_end_of_pull_over;
  double backward_length_buffer_for_end_of_pull_out;

  // common parameters
  double min_acc;
  double max_acc;

  // lane change parameters
  double lane_changing_lateral_jerk{0.5};
  double lane_changing_lateral_acc{0.315};
  double lane_changing_lateral_acc_at_low_velocity{0.15};
  double lateral_acc_switching_velocity{0.4};
  double minimum_lane_changing_velocity{5.6};
  double lane_change_prepare_duration{4.0};
  double minimum_prepare_length;

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

  double enable_akima_spline_first;
  double enable_cog_on_centerline;
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

  // maximum drivable area visualization
  bool visualize_maximum_drivable_area;

  // collision check
  double lateral_distance_max_threshold;
  double longitudinal_distance_min_threshold;

  double expected_front_deceleration;  // brake parameter under normal lane change
  double expected_rear_deceleration;   // brake parameter under normal lane change

  double expected_front_deceleration_for_abort;  // hard brake parameter for abort
  double expected_rear_deceleration_for_abort;   // hard brake parameter for abort

  double rear_vehicle_reaction_time;
  double rear_vehicle_safety_time_margin;
};

#endif  // BEHAVIOR_PATH_PLANNER__PARAMETERS_HPP_
