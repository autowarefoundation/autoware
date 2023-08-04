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

#include "behavior_path_planner/scene_module/start_planner/manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

StartPlannerModuleManager::StartPlannerModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config)
: SceneModuleManagerInterface(node, name, config, {""})
{
  StartPlannerParameters p;

  std::string ns = "start_planner.";

  p.th_arrived_distance = node->declare_parameter<double>(ns + "th_arrived_distance");
  p.th_stopped_velocity = node->declare_parameter<double>(ns + "th_stopped_velocity");
  p.th_stopped_time = node->declare_parameter<double>(ns + "th_stopped_time");
  p.th_turn_signal_on_lateral_offset =
    node->declare_parameter<double>(ns + "th_turn_signal_on_lateral_offset");
  p.intersection_search_length = node->declare_parameter<double>(ns + "intersection_search_length");
  p.length_ratio_for_turn_signal_deactivation_near_intersection = node->declare_parameter<double>(
    ns + "length_ratio_for_turn_signal_deactivation_near_intersection");
  p.collision_check_margin = node->declare_parameter<double>(ns + "collision_check_margin");
  p.collision_check_distance_from_end =
    node->declare_parameter<double>(ns + "collision_check_distance_from_end");
  p.th_moving_object_velocity = node->declare_parameter<double>(ns + "th_moving_object_velocity");
  // shift pull out
  p.enable_shift_pull_out = node->declare_parameter<bool>(ns + "enable_shift_pull_out");
  p.check_shift_path_lane_departure =
    node->declare_parameter<bool>(ns + "check_shift_path_lane_departure");
  p.minimum_shift_pull_out_distance =
    node->declare_parameter<double>(ns + "minimum_shift_pull_out_distance");
  p.lateral_acceleration_sampling_num =
    node->declare_parameter<int>(ns + "lateral_acceleration_sampling_num");
  p.lateral_jerk = node->declare_parameter<double>(ns + "lateral_jerk");
  p.maximum_lateral_acc = node->declare_parameter<double>(ns + "maximum_lateral_acc");
  p.minimum_lateral_acc = node->declare_parameter<double>(ns + "minimum_lateral_acc");
  p.maximum_curvature = node->declare_parameter<double>(ns + "maximum_curvature");
  p.deceleration_interval = node->declare_parameter<double>(ns + "deceleration_interval");
  // geometric pull out
  p.enable_geometric_pull_out = node->declare_parameter<bool>(ns + "enable_geometric_pull_out");
  p.divide_pull_out_path = node->declare_parameter<bool>(ns + "divide_pull_out_path");
  p.parallel_parking_parameters.pull_out_velocity =
    node->declare_parameter<double>(ns + "geometric_pull_out_velocity");
  p.parallel_parking_parameters.pull_out_path_interval =
    node->declare_parameter<double>(ns + "arc_path_interval");
  p.parallel_parking_parameters.pull_out_lane_departure_margin =
    node->declare_parameter<double>(ns + "lane_departure_margin");
  p.parallel_parking_parameters.pull_out_max_steer_angle =
    node->declare_parameter<double>(ns + "pull_out_max_steer_angle");  // 15deg
  // search start pose backward
  p.search_priority = node->declare_parameter<std::string>(
    ns + "search_priority");  // "efficient_path" or "short_back_distance"
  p.enable_back = node->declare_parameter<bool>(ns + "enable_back");
  p.backward_velocity = node->declare_parameter<double>(ns + "backward_velocity");
  p.max_back_distance = node->declare_parameter<double>(ns + "max_back_distance");
  p.backward_search_resolution = node->declare_parameter<double>(ns + "backward_search_resolution");
  p.backward_path_update_duration =
    node->declare_parameter<double>(ns + "backward_path_update_duration");
  p.ignore_distance_from_lane_end =
    node->declare_parameter<double>(ns + "ignore_distance_from_lane_end");

  // validation of parameters
  if (p.lateral_acceleration_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      logger_, "lateral_acceleration_sampling_num must be positive integer. Given parameter: "
                 << p.lateral_acceleration_sampling_num << std::endl
                 << "Terminating the program...");
    exit(EXIT_FAILURE);
  }

  parameters_ = std::make_shared<StartPlannerParameters>(p);
}

void StartPlannerModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto & p = parameters_;

  [[maybe_unused]] std::string ns = name_ + ".";

  std::for_each(observers_.begin(), observers_.end(), [&](const auto & observer) {
    if (!observer.expired()) {
      const auto start_planner_ptr = std::dynamic_pointer_cast<StartPlannerModule>(observer.lock());
      if (start_planner_ptr) {
        start_planner_ptr->updateModuleParams(p);
        start_planner_ptr->setInitialIsSimultaneousExecutableAsApprovedModule(
          enable_simultaneous_execution_as_approved_module_);
        start_planner_ptr->setInitialIsSimultaneousExecutableAsCandidateModule(
          enable_simultaneous_execution_as_candidate_module_);
      }
    }
  });
}
}  // namespace behavior_path_planner
