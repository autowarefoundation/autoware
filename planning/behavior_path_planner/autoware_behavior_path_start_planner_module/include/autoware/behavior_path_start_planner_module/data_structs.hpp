
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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__DATA_STRUCTS_HPP_

#include "autoware/behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <autoware/freespace_planning_algorithms/abstract_algorithm.hpp>
#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include <magic_enum.hpp>

#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

using autoware::behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap;
using autoware::behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::TargetObjectsOnLane;
using autoware_perception_msgs::msg::PredictedObjects;

using autoware::freespace_planning_algorithms::AstarParam;
using autoware::freespace_planning_algorithms::PlannerCommonParam;
using autoware::freespace_planning_algorithms::RRTStarParam;

enum class PlannerType {
  NONE = 0,
  SHIFT = 1,
  GEOMETRIC = 2,
  STOP = 3,
  FREESPACE = 4,
};

struct PlannerDebugData
{
public:
  PlannerType planner_type;
  std::vector<std::string> conditions_evaluation;
  double required_margin{0.0};
  double backward_distance{0.0};

  auto header_str() const
  {
    std::stringstream ss;
    ss << std::left << std::setw(20) << "| Planner type " << std::setw(20) << "| Required margin "
       << std::setw(20) << "| Backward distance " << std::setw(25) << "| Condition evaluation |"
       << "\n";
    return ss.str();
  }

  auto str() const
  {
    std::stringstream ss;
    for (const auto & result : conditions_evaluation) {
      ss << std::left << std::setw(23) << magic_enum::enum_name(planner_type) << std::setw(23)
         << (std::to_string(required_margin) + "[m]") << std::setw(23)
         << (std::to_string(backward_distance) + "[m]") << std::setw(25) << result << "\n";
    }
    ss << std::setw(40);
    return ss.str();
  }
};
struct StartPlannerDebugData
{
  // filtered objects
  PredictedObjects filtered_objects;
  TargetObjectsOnLane target_objects_on_lane;
  std::vector<PoseWithVelocityStamped> ego_predicted_path;
  // collision check debug map
  CollisionCheckDebugMap collision_check;
  lanelet::ConstLanelets departure_check_lanes;

  Pose refined_start_pose;
  std::vector<Pose> start_pose_candidates;
  size_t selected_start_pose_candidate_index;
  double margin_for_start_pose_candidate;

  // for isPreventingRearVehicleFromPassingThrough
  std::optional<Pose> estimated_stop_pose;
};

struct StartPlannerParameters
{
  double th_arrived_distance{0.0};
  double th_stopped_velocity{0.0};
  double th_stopped_time{0.0};
  double prepare_time_before_start{0.0};
  double th_distance_to_middle_of_the_road{0.0};
  bool skip_rear_vehicle_check{false};
  double extra_width_margin_for_rear_obstacle{0.0};
  std::vector<double> collision_check_margins{};
  double collision_check_margin_from_front_object{0.0};
  double th_moving_object_velocity{0.0};
  autoware::behavior_path_planner::utils::path_safety_checker::ObjectTypesToCheck
    object_types_to_check_for_path_generation{};
  double center_line_path_interval{0.0};
  double lane_departure_check_expansion_margin{0.0};

  // shift pull out
  bool enable_shift_pull_out{false};
  bool check_shift_path_lane_departure{false};
  bool allow_check_shift_path_lane_departure_override{false};
  double shift_collision_check_distance_from_end{0.0};
  double minimum_shift_pull_out_distance{0.0};
  int lateral_acceleration_sampling_num{0};
  double lateral_jerk{0.0};
  double maximum_lateral_acc{0.0};
  double minimum_lateral_acc{0.0};
  double maximum_curvature{0.0};  // maximum curvature considered in the path generation
  double end_pose_curvature_threshold{0.0};
  double maximum_longitudinal_deviation{0.0};
  // geometric pull out
  bool enable_geometric_pull_out{false};
  double geometric_collision_check_distance_from_end{0.0};
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
  autoware::behavior_path_planner::utils::path_safety_checker::ObjectTypesToCheck
    surround_moving_obstacles_type_to_check{};

  bool print_debug_info{false};
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__DATA_STRUCTS_HPP_
