
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

#ifndef BEHAVIOR_PATH_START_PLANNER_MODULE__START_PLANNER_PARAMETERS_HPP_
#define BEHAVIOR_PATH_START_PLANNER_MODULE__START_PLANNER_PARAMETERS_HPP_

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

struct StartPlannerParameters
{
  double th_arrived_distance{0.0};
  double th_stopped_velocity{0.0};
  double th_stopped_time{0.0};
  double th_turn_signal_on_lateral_offset{0.0};
  double th_distance_to_middle_of_the_road{0.0};
  double intersection_search_length{0.0};
  double length_ratio_for_turn_signal_deactivation_near_intersection{0.0};
  double collision_check_margin{0.0};
  double collision_check_distance_from_end{0.0};
  double th_moving_object_velocity{0.0};
  double center_line_path_interval{0.0};

  // shift pull out
  bool enable_shift_pull_out{false};
  bool check_shift_path_lane_departure{false};
  double minimum_shift_pull_out_distance{0.0};
  int lateral_acceleration_sampling_num{0};
  double lateral_jerk{0.0};
  double maximum_lateral_acc{0.0};
  double minimum_lateral_acc{0.0};
  double maximum_curvature{0.0};  // maximum curvature considered in the path generation
  double deceleration_interval{0.0};
  // geometric pull out
  bool enable_geometric_pull_out{false};
  bool divide_pull_out_path{false};
  ParallelParkingParameters parallel_parking_parameters{};
  // search start pose backward
  std::string search_priority;  // "efficient_path" or "short_back_distance"
  bool enable_back{false};
  double backward_velocity{0.0};
  double max_back_distance{0.0};
  double backward_search_resolution{0.0};
  double backward_path_update_duration{0.0};
  double ignore_distance_from_lane_end{0.0};
  // freespace planner
  bool enable_freespace_planner{false};
  std::string freespace_planner_algorithm;
  double end_pose_search_start_distance{0.0};
  double end_pose_search_end_distance{0.0};
  double end_pose_search_interval{0.0};
  double freespace_planner_velocity{0.0};
  double vehicle_shape_margin{0.0};
  PlannerCommonParam freespace_planner_common_parameters;
  AstarParam astar_parameters;
  RRTStarParam rrt_star_parameters;

  // stop condition
  double maximum_deceleration_for_stop{0.0};
  double maximum_jerk_for_stop{0.0};

  // hysteresis parameter
  double hysteresis_factor_expand_rate{0.0};

  // path safety checker
  utils::path_safety_checker::EgoPredictedPathParams ego_predicted_path_params{};
  utils::path_safety_checker::ObjectsFilteringParams objects_filtering_params{};
  utils::path_safety_checker::SafetyCheckParams safety_check_params{};

  // surround moving obstacle check
  double search_radius{0.0};
  double th_moving_obstacle_velocity{0.0};
  behavior_path_planner::utils::path_safety_checker::ObjectTypesToCheck
    surround_moving_obstacles_type_to_check{};

  bool print_debug_info{false};
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_START_PLANNER_MODULE__START_PLANNER_PARAMETERS_HPP_
