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

struct BehaviorPathPlannerParameters
{
  double backward_path_length;
  double forward_path_length;
  double backward_length_buffer_for_end_of_lane;
  double backward_length_buffer_for_end_of_pull_over;
  double backward_length_buffer_for_end_of_pull_out;
  double minimum_lane_change_length;
  double minimum_pull_over_length;
  double minimum_pull_out_length;
  double drivable_area_resolution;

  double drivable_lane_forward_length;
  double drivable_lane_backward_length;
  double drivable_lane_margin;
  double drivable_area_margin;

  double refine_goal_search_radius_range;

  double turn_signal_intersection_search_distance;
  double turn_signal_intersection_angle_threshold_deg;
  double turn_signal_search_time;
  double turn_signal_minimum_search_distance;
  double turn_signal_shift_length_threshold;

  double path_interval;

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

  // drivable area visualization
  bool visualize_drivable_area_for_shared_linestrings_lanelet;

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
