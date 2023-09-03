
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
#include "behavior_path_planner/utils/path_safety_checker/path_safety_checker_parameters.hpp"

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
  double th_arrived_distance;
  double th_stopped_velocity;
  double th_stopped_time;
  double th_turn_signal_on_lateral_offset;
  double intersection_search_length;
  double length_ratio_for_turn_signal_deactivation_near_intersection;
  double collision_check_margin;
  double collision_check_distance_from_end;
  double th_moving_object_velocity;
  // shift pull out
  bool enable_shift_pull_out;
  bool check_shift_path_lane_departure;
  double minimum_shift_pull_out_distance;
  int lateral_acceleration_sampling_num;
  double lateral_jerk;
  double maximum_lateral_acc;
  double minimum_lateral_acc;
  double maximum_curvature;  // maximum curvature considered in the path generation
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
  // freespace planner
  bool enable_freespace_planner;
  std::string freespace_planner_algorithm;
  double end_pose_search_start_distance;
  double end_pose_search_end_distance;
  double end_pose_search_interval;
  double freespace_planner_velocity;
  double vehicle_shape_margin;
  PlannerCommonParam freespace_planner_common_parameters;
  AstarParam astar_parameters;
  RRTStarParam rrt_star_parameters;

  // stop condition
  double maximum_deceleration_for_stop;
  double maximum_jerk_for_stop;

  // hysteresis parameter
  double hysteresis_factor_expand_rate;

  // path safety checker
  utils::path_safety_checker::EgoPredictedPathParams ego_predicted_path_params;
  utils::path_safety_checker::ObjectsFilteringParams objects_filtering_params;
  utils::path_safety_checker::SafetyCheckParams safety_check_params;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__START_PLANNER__START_PLANNER_PARAMETERS_HPP_
