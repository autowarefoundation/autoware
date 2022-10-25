
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_PARAMETERS_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_PARAMETERS_HPP_

#include <string>

namespace behavior_path_planner
{
struct PullOverParameters
{
  double request_length;
  double th_arrived_distance;
  double th_stopped_velocity;
  double th_stopped_time;
  double margin_from_boundary;
  double decide_path_distance;
  double maximum_deceleration;
  // goal research
  std::string search_priority;  // "efficient_path" or "close_goal"
  bool enable_goal_research;
  double forward_goal_search_length;
  double backward_goal_search_length;
  double goal_search_interval;
  double longitudinal_margin;
  double max_lateral_offset;
  double lateral_offset_interval;
  // occupancy grid map
  bool use_occupancy_grid;
  bool use_occupancy_grid_for_longitudinal_margin;
  double occupancy_grid_collision_check_margin;
  double theta_size;
  double obstacle_threshold;
  // object recognition
  bool use_object_recognition;
  double object_recognition_collision_check_margin;
  // shift path
  bool enable_shift_parking;
  int pull_over_sampling_num;
  double maximum_lateral_jerk;
  double minimum_lateral_jerk;
  double deceleration_interval;
  double pull_over_velocity;
  double pull_over_minimum_velocity;
  double after_pull_over_straight_distance;
  double before_pull_over_straight_distance;
  // parallel parking
  bool enable_arc_forward_parking;
  bool enable_arc_backward_parking;
  double after_forward_parking_straight_distance;
  double after_backward_parking_straight_distance;
  double forward_parking_velocity;
  double backward_parking_velocity;
  double forward_parking_lane_departure_margin;
  double backward_parking_lane_departure_margin;
  double arc_path_interval;
  double pull_over_max_steer_angle;
  // hazard
  double hazard_on_threshold_distance;
  double hazard_on_threshold_velocity;
  // check safety with dynamic objects. Not used now.
  double pull_over_duration;
  double pull_over_prepare_duration;
  double min_stop_distance;
  double stop_time;
  double hysteresis_buffer_distance;
  double prediction_time_resolution;
  bool enable_collision_check_at_prepare_phase;
  bool use_predicted_path_outside_lanelet;
  bool use_all_predicted_path;
  // drivable area expansion
  double drivable_area_right_bound_offset;
  double drivable_area_left_bound_offset;
  // debug
  bool print_debug_info;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__PULL_OVER_PARAMETERS_HPP_
