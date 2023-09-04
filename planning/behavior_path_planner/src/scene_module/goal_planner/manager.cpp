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

#include "behavior_path_planner/scene_module/goal_planner/manager.hpp"

#include "behavior_path_planner/utils/goal_planner/util.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

GoalPlannerModuleManager::GoalPlannerModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config)
: SceneModuleManagerInterface(node, name, config, {""})
{
  GoalPlannerParameters p;

  // general params
  {
    std::string ns = "goal_planner.";
    p.minimum_request_length = node->declare_parameter<double>(ns + "minimum_request_length");
    p.th_stopped_velocity = node->declare_parameter<double>(ns + "th_stopped_velocity");
    p.th_arrived_distance = node->declare_parameter<double>(ns + "th_arrived_distance");
    p.th_stopped_time = node->declare_parameter<double>(ns + "th_stopped_time");
  }

  // goal search
  {
    std::string ns = "goal_planner.goal_search.";
    p.search_priority = node->declare_parameter<std::string>(ns + "search_priority");
    p.forward_goal_search_length =
      node->declare_parameter<double>(ns + "forward_goal_search_length");
    p.backward_goal_search_length =
      node->declare_parameter<double>(ns + "backward_goal_search_length");
    p.goal_search_interval = node->declare_parameter<double>(ns + "goal_search_interval");
    p.longitudinal_margin = node->declare_parameter<double>(ns + "longitudinal_margin");
    p.max_lateral_offset = node->declare_parameter<double>(ns + "max_lateral_offset");
    p.lateral_offset_interval = node->declare_parameter<double>(ns + "lateral_offset_interval");
    p.ignore_distance_from_lane_start =
      node->declare_parameter<double>(ns + "ignore_distance_from_lane_start");
    p.margin_from_boundary = node->declare_parameter<double>(ns + "margin_from_boundary");

    const std::string parking_policy_name =
      node->declare_parameter<std::string>(ns + "parking_policy");
    if (parking_policy_name == "left_side") {
      p.parking_policy = ParkingPolicy::LEFT_SIDE;
    } else if (parking_policy_name == "right_side") {
      p.parking_policy = ParkingPolicy::RIGHT_SIDE;
    } else {
      RCLCPP_ERROR_STREAM(
        logger_, "[goal_planner] invalid parking_policy: " << parking_policy_name << std::endl);
      exit(EXIT_FAILURE);
    }
  }

  // occupancy grid map
  {
    std::string ns = "goal_planner.occupancy_grid.";
    p.use_occupancy_grid = node->declare_parameter<bool>(ns + "use_occupancy_grid");
    p.use_occupancy_grid_for_longitudinal_margin =
      node->declare_parameter<bool>(ns + "use_occupancy_grid_for_longitudinal_margin");
    p.occupancy_grid_collision_check_margin =
      node->declare_parameter<double>(ns + "occupancy_grid_collision_check_margin");
    p.theta_size = node->declare_parameter<int>(ns + "theta_size");
    p.obstacle_threshold = node->declare_parameter<int>(ns + "obstacle_threshold");
  }

  // object recognition
  {
    std::string ns = "goal_planner.object_recognition.";
    p.use_object_recognition = node->declare_parameter<bool>(ns + "use_object_recognition");
    p.object_recognition_collision_check_margin =
      node->declare_parameter<double>(ns + "object_recognition_collision_check_margin");
    p.object_recognition_collision_check_max_extra_stopping_margin =
      node->declare_parameter<double>(
        ns + "object_recognition_collision_check_max_extra_stopping_margin");
  }

  // pull over general params
  {
    std::string ns = "goal_planner.pull_over.";
    p.pull_over_velocity = node->declare_parameter<double>(ns + "pull_over_velocity");
    p.pull_over_minimum_velocity =
      node->declare_parameter<double>(ns + "pull_over_minimum_velocity");
    p.decide_path_distance = node->declare_parameter<double>(ns + "decide_path_distance");
    p.maximum_deceleration = node->declare_parameter<double>(ns + "maximum_deceleration");
    p.maximum_jerk = node->declare_parameter<double>(ns + "maximum_jerk");
  }

  // shift parking
  {
    std::string ns = "goal_planner.pull_over.shift_parking.";
    p.enable_shift_parking = node->declare_parameter<bool>(ns + "enable_shift_parking");
    p.shift_sampling_num = node->declare_parameter<int>(ns + "shift_sampling_num");
    p.maximum_lateral_jerk = node->declare_parameter<double>(ns + "maximum_lateral_jerk");
    p.minimum_lateral_jerk = node->declare_parameter<double>(ns + "minimum_lateral_jerk");
    p.deceleration_interval = node->declare_parameter<double>(ns + "deceleration_interval");
    p.after_shift_straight_distance =
      node->declare_parameter<double>(ns + "after_shift_straight_distance");
  }

  // forward parallel parking forward
  {
    std::string ns = "goal_planner.pull_over.parallel_parking.forward.";
    p.enable_arc_forward_parking = node->declare_parameter<bool>(ns + "enable_arc_forward_parking");
    p.parallel_parking_parameters.after_forward_parking_straight_distance =
      node->declare_parameter<double>(ns + "after_forward_parking_straight_distance");
    p.parallel_parking_parameters.forward_parking_velocity =
      node->declare_parameter<double>(ns + "forward_parking_velocity");
    p.parallel_parking_parameters.forward_parking_lane_departure_margin =
      node->declare_parameter<double>(ns + "forward_parking_lane_departure_margin");
    p.parallel_parking_parameters.forward_parking_path_interval =
      node->declare_parameter<double>(ns + "forward_parking_path_interval");
    p.parallel_parking_parameters.forward_parking_max_steer_angle =
      node->declare_parameter<double>(ns + "forward_parking_max_steer_angle");  // 20deg
  }

  // forward parallel parking backward
  {
    std::string ns = "goal_planner.pull_over.parallel_parking.backward.";
    p.enable_arc_backward_parking =
      node->declare_parameter<bool>(ns + "enable_arc_backward_parking");
    p.parallel_parking_parameters.after_backward_parking_straight_distance =
      node->declare_parameter<double>(ns + "after_backward_parking_straight_distance");
    p.parallel_parking_parameters.backward_parking_velocity =
      node->declare_parameter<double>(ns + "backward_parking_velocity");
    p.parallel_parking_parameters.backward_parking_lane_departure_margin =
      node->declare_parameter<double>(ns + "backward_parking_lane_departure_margin");
    p.parallel_parking_parameters.backward_parking_path_interval =
      node->declare_parameter<double>(ns + "backward_parking_path_interval");
    p.parallel_parking_parameters.backward_parking_max_steer_angle =
      node->declare_parameter<double>(ns + "backward_parking_max_steer_angle");  // 20deg
  }

  // freespace parking general params
  {
    std::string ns = "goal_planner.pull_over.freespace_parking.";
    p.enable_freespace_parking = node->declare_parameter<bool>(ns + "enable_freespace_parking");
    p.freespace_parking_algorithm =
      node->declare_parameter<std::string>(ns + "freespace_parking_algorithm");
    p.freespace_parking_velocity = node->declare_parameter<double>(ns + "velocity");
    p.vehicle_shape_margin = node->declare_parameter<double>(ns + "vehicle_shape_margin");
    p.freespace_parking_common_parameters.time_limit =
      node->declare_parameter<double>(ns + "time_limit");
    p.freespace_parking_common_parameters.minimum_turning_radius =
      node->declare_parameter<double>(ns + "minimum_turning_radius");
    p.freespace_parking_common_parameters.maximum_turning_radius =
      node->declare_parameter<double>(ns + "maximum_turning_radius");
    p.freespace_parking_common_parameters.turning_radius_size =
      node->declare_parameter<int>(ns + "turning_radius_size");
    p.freespace_parking_common_parameters.maximum_turning_radius = std::max(
      p.freespace_parking_common_parameters.maximum_turning_radius,
      p.freespace_parking_common_parameters.minimum_turning_radius);
    p.freespace_parking_common_parameters.turning_radius_size =
      std::max(p.freespace_parking_common_parameters.turning_radius_size, 1);
  }

  //  freespace parking search config
  {
    std::string ns = "goal_planner.pull_over.freespace_parking.search_configs.";
    p.freespace_parking_common_parameters.theta_size =
      node->declare_parameter<int>(ns + "theta_size");
    p.freespace_parking_common_parameters.angle_goal_range =
      node->declare_parameter<double>(ns + "angle_goal_range");
    p.freespace_parking_common_parameters.curve_weight =
      node->declare_parameter<double>(ns + "curve_weight");
    p.freespace_parking_common_parameters.reverse_weight =
      node->declare_parameter<double>(ns + "reverse_weight");
    p.freespace_parking_common_parameters.lateral_goal_range =
      node->declare_parameter<double>(ns + "lateral_goal_range");
    p.freespace_parking_common_parameters.longitudinal_goal_range =
      node->declare_parameter<double>(ns + "longitudinal_goal_range");
  }

  //  freespace parking costmap configs
  {
    std::string ns = "goal_planner.pull_over.freespace_parking.costmap_configs.";
    p.freespace_parking_common_parameters.obstacle_threshold =
      node->declare_parameter<int>(ns + "obstacle_threshold");
  }

  //  freespace parking astar
  {
    std::string ns = "goal_planner.pull_over.freespace_parking.astar.";
    p.astar_parameters.only_behind_solutions =
      node->declare_parameter<bool>(ns + "only_behind_solutions");
    p.astar_parameters.use_back = node->declare_parameter<bool>(ns + "use_back");
    p.astar_parameters.distance_heuristic_weight =
      node->declare_parameter<double>(ns + "distance_heuristic_weight");
  }

  //   freespace parking rrtstar
  {
    std::string ns = "goal_planner.pull_over.freespace_parking.rrtstar.";
    p.rrt_star_parameters.enable_update = node->declare_parameter<bool>(ns + "enable_update");
    p.rrt_star_parameters.use_informed_sampling =
      node->declare_parameter<bool>(ns + "use_informed_sampling");
    p.rrt_star_parameters.max_planning_time =
      node->declare_parameter<double>(ns + "max_planning_time");
    p.rrt_star_parameters.neighbor_radius = node->declare_parameter<double>(ns + "neighbor_radius");
    p.rrt_star_parameters.margin = node->declare_parameter<double>(ns + "margin");
  }

  // debug
  {
    std::string ns = "goal_planner.debug.";
    p.print_debug_info = node->declare_parameter<bool>(ns + "print_debug_info");
  }

  // validation of parameters
  if (p.shift_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      logger_, "shift_sampling_num must be positive integer. Given parameter: "
                 << p.shift_sampling_num << std::endl
                 << "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  if (p.maximum_deceleration < 0.0) {
    RCLCPP_FATAL_STREAM(
      logger_, "maximum_deceleration cannot be negative value. Given parameter: "
                 << p.maximum_deceleration << std::endl
                 << "Terminating the program...");
    exit(EXIT_FAILURE);
  }

  parameters_ = std::make_shared<GoalPlannerParameters>(p);
}

void GoalPlannerModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto & p = parameters_;

  [[maybe_unused]] std::string ns = name_ + ".";

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

bool GoalPlannerModuleManager::isAlwaysExecutableModule() const
{
  // enable AlwaysExecutable whenever goal modification is not allowed
  // because only minor path refinements are made for fixed goals
  if (!goal_planner_utils::isAllowedGoalModification(planner_data_->route_handler)) {
    return true;
  }

  return false;
}

bool GoalPlannerModuleManager::isSimultaneousExecutableAsApprovedModule() const
{
  if (isAlwaysExecutableModule()) {
    return true;
  }

  // enable SimultaneousExecutable whenever goal modification is not allowed
  // because only minor path refinements are made for fixed goals
  if (!goal_planner_utils::isAllowedGoalModification(planner_data_->route_handler)) {
    return true;
  }

  return enable_simultaneous_execution_as_approved_module_;
}

bool GoalPlannerModuleManager::isSimultaneousExecutableAsCandidateModule() const
{
  if (isAlwaysExecutableModule()) {
    return true;
  }

  // enable SimultaneousExecutable whenever goal modification is not allowed
  // because only minor path refinements are made for fixed goals
  if (!goal_planner_utils::isAllowedGoalModification(planner_data_->route_handler)) {
    return true;
  }

  return enable_simultaneous_execution_as_candidate_module_;
}

}  // namespace behavior_path_planner
