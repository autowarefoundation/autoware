
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

#ifndef BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_PLANNER_PARAMETERS_HPP_
#define BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_PLANNER_PARAMETERS_HPP_

#include "behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <freespace_planning_algorithms/abstract_algorithm.hpp>
#include <freespace_planning_algorithms/astar_search.hpp>
#include <freespace_planning_algorithms/rrtstar.hpp>

#include <string>
#include <vector>

namespace behavior_path_planner
{

using freespace_planning_algorithms::AstarParam;
using freespace_planning_algorithms::PlannerCommonParam;
using freespace_planning_algorithms::RRTStarParam;

enum class ParkingPolicy {
  LEFT_SIDE = 0,
  RIGHT_SIDE,
};

struct GoalPlannerParameters
{
  // general  params
  double th_arrived_distance{0.0};
  double th_stopped_velocity{0.0};
  double th_stopped_time{0.0};
  double th_blinker_on_lateral_offset{0.0};
  double center_line_path_interval{0.0};

  // goal search
  std::string goal_priority;  // "minimum_weighted_distance" or "minimum_longitudinal_distance"
  double minimum_weighted_distance_lateral_weight{0.0};
  bool prioritize_goals_before_objects{false};
  ParkingPolicy parking_policy;  // "left_side" or "right_side"
  double forward_goal_search_length{0.0};
  double backward_goal_search_length{0.0};
  double goal_search_interval{0.0};
  double longitudinal_margin{0.0};
  double max_lateral_offset{0.0};
  double lateral_offset_interval{0.0};
  double ignore_distance_from_lane_start{0.0};
  double margin_from_boundary{0.0};

  // occupancy grid map
  bool use_occupancy_grid_for_goal_search{false};
  bool use_occupancy_grid_for_goal_longitudinal_margin{false};
  bool use_occupancy_grid_for_path_collision_check{false};
  double occupancy_grid_collision_check_margin{0.0};
  int theta_size{0};
  int obstacle_threshold{0};

  // object recognition
  bool use_object_recognition{false};
  double object_recognition_collision_check_margin{0.0};
  double object_recognition_collision_check_max_extra_stopping_margin{0.0};
  double th_moving_object_velocity{0.0};

  // pull over general params
  double pull_over_minimum_request_length{0.0};
  double pull_over_velocity{0.0};
  double pull_over_minimum_velocity{0.0};
  double decide_path_distance{0.0};
  double maximum_deceleration{0.0};
  double maximum_jerk{0.0};
  std::string path_priority;  // "efficient_path" or "close_goal"
  std::vector<std::string> efficient_path_order{};

  // shift path
  bool enable_shift_parking{false};
  int shift_sampling_num{0};
  double maximum_lateral_jerk{0.0};
  double minimum_lateral_jerk{0.0};
  double deceleration_interval{0.0};
  double after_shift_straight_distance{0.0};

  // parallel parking
  bool enable_arc_forward_parking{false};
  bool enable_arc_backward_parking{false};
  ParallelParkingParameters parallel_parking_parameters;

  // freespace parking
  bool enable_freespace_parking{false};
  std::string freespace_parking_algorithm;
  double freespace_parking_velocity{0.0};
  double vehicle_shape_margin{0.0};
  PlannerCommonParam freespace_parking_common_parameters{};
  AstarParam astar_parameters{};
  RRTStarParam rrt_star_parameters{};

  // stop condition
  double maximum_deceleration_for_stop{0.0};
  double maximum_jerk_for_stop{0.0};

  // hysteresis parameter
  double hysteresis_factor_expand_rate{0.0};

  // path safety checker
  utils::path_safety_checker::EgoPredictedPathParams ego_predicted_path_params{};
  utils::path_safety_checker::ObjectsFilteringParams objects_filtering_params{};
  utils::path_safety_checker::SafetyCheckParams safety_check_params{};

  // debug
  bool print_debug_info{false};
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_GOAL_PLANNER_MODULE__GOAL_PLANNER_PARAMETERS_HPP_
