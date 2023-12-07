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

#include "behavior_path_start_planner_module/manager.hpp"

#include "tier4_autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{
void StartPlannerModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {""});

  StartPlannerParameters p;

  const std::string ns = "start_planner.";

  p.th_arrived_distance = node->declare_parameter<double>(ns + "th_arrived_distance");
  p.th_stopped_velocity = node->declare_parameter<double>(ns + "th_stopped_velocity");
  p.th_stopped_time = node->declare_parameter<double>(ns + "th_stopped_time");
  p.th_turn_signal_on_lateral_offset =
    node->declare_parameter<double>(ns + "th_turn_signal_on_lateral_offset");
  p.th_distance_to_middle_of_the_road =
    node->declare_parameter<double>(ns + "th_distance_to_middle_of_the_road");
  p.intersection_search_length = node->declare_parameter<double>(ns + "intersection_search_length");
  p.length_ratio_for_turn_signal_deactivation_near_intersection = node->declare_parameter<double>(
    ns + "length_ratio_for_turn_signal_deactivation_near_intersection");
  p.collision_check_margin = node->declare_parameter<double>(ns + "collision_check_margin");
  p.collision_check_distance_from_end =
    node->declare_parameter<double>(ns + "collision_check_distance_from_end");
  p.th_moving_object_velocity = node->declare_parameter<double>(ns + "th_moving_object_velocity");
  p.center_line_path_interval = node->declare_parameter<double>(ns + "center_line_path_interval");
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
  p.parallel_parking_parameters.center_line_path_interval =
    p.center_line_path_interval;  // for geometric parallel parking
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
  // freespace planner general params
  {
    const std::string ns = "start_planner.freespace_planner.";
    p.enable_freespace_planner = node->declare_parameter<bool>(ns + "enable_freespace_planner");
    p.freespace_planner_algorithm =
      node->declare_parameter<std::string>(ns + "freespace_planner_algorithm");
    p.end_pose_search_start_distance =
      node->declare_parameter<double>(ns + "end_pose_search_start_distance");
    p.end_pose_search_end_distance =
      node->declare_parameter<double>(ns + "end_pose_search_end_distance");
    p.end_pose_search_interval = node->declare_parameter<double>(ns + "end_pose_search_interval");
    p.freespace_planner_velocity = node->declare_parameter<double>(ns + "velocity");
    p.vehicle_shape_margin = node->declare_parameter<double>(ns + "vehicle_shape_margin");
    p.freespace_planner_common_parameters.time_limit =
      node->declare_parameter<double>(ns + "time_limit");
    p.freespace_planner_common_parameters.minimum_turning_radius =
      node->declare_parameter<double>(ns + "minimum_turning_radius");
    p.freespace_planner_common_parameters.maximum_turning_radius =
      node->declare_parameter<double>(ns + "maximum_turning_radius");
    p.freespace_planner_common_parameters.turning_radius_size =
      node->declare_parameter<int>(ns + "turning_radius_size");
    p.freespace_planner_common_parameters.maximum_turning_radius = std::max(
      p.freespace_planner_common_parameters.maximum_turning_radius,
      p.freespace_planner_common_parameters.minimum_turning_radius);
    p.freespace_planner_common_parameters.turning_radius_size =
      std::max(p.freespace_planner_common_parameters.turning_radius_size, 1);
  }
  //  freespace planner search config
  {
    const std::string ns = "start_planner.freespace_planner.search_configs.";
    p.freespace_planner_common_parameters.theta_size =
      node->declare_parameter<int>(ns + "theta_size");
    p.freespace_planner_common_parameters.angle_goal_range =
      node->declare_parameter<double>(ns + "angle_goal_range");
    p.freespace_planner_common_parameters.curve_weight =
      node->declare_parameter<double>(ns + "curve_weight");
    p.freespace_planner_common_parameters.reverse_weight =
      node->declare_parameter<double>(ns + "reverse_weight");
    p.freespace_planner_common_parameters.lateral_goal_range =
      node->declare_parameter<double>(ns + "lateral_goal_range");
    p.freespace_planner_common_parameters.longitudinal_goal_range =
      node->declare_parameter<double>(ns + "longitudinal_goal_range");
  }
  //  freespace planner costmap configs
  {
    const std::string ns = "start_planner.freespace_planner.costmap_configs.";
    p.freespace_planner_common_parameters.obstacle_threshold =
      node->declare_parameter<int>(ns + "obstacle_threshold");
  }
  //  freespace planner astar
  {
    const std::string ns = "start_planner.freespace_planner.astar.";
    p.astar_parameters.only_behind_solutions =
      node->declare_parameter<bool>(ns + "only_behind_solutions");
    p.astar_parameters.use_back = node->declare_parameter<bool>(ns + "use_back");
    p.astar_parameters.distance_heuristic_weight =
      node->declare_parameter<double>(ns + "distance_heuristic_weight");
  }
  //   freespace planner rrtstar
  {
    const std::string ns = "start_planner.freespace_planner.rrtstar.";
    p.rrt_star_parameters.enable_update = node->declare_parameter<bool>(ns + "enable_update");
    p.rrt_star_parameters.use_informed_sampling =
      node->declare_parameter<bool>(ns + "use_informed_sampling");
    p.rrt_star_parameters.max_planning_time =
      node->declare_parameter<double>(ns + "max_planning_time");
    p.rrt_star_parameters.neighbor_radius = node->declare_parameter<double>(ns + "neighbor_radius");
    p.rrt_star_parameters.margin = node->declare_parameter<double>(ns + "margin");
  }

  // stop condition
  {
    p.maximum_deceleration_for_stop =
      node->declare_parameter<double>(ns + "stop_condition.maximum_deceleration_for_stop");
    p.maximum_jerk_for_stop =
      node->declare_parameter<double>(ns + "stop_condition.maximum_jerk_for_stop");
  }

  const std::string base_ns = "start_planner.path_safety_check.";

  // EgoPredictedPath
  const std::string ego_path_ns = base_ns + "ego_predicted_path.";
  {
    p.ego_predicted_path_params.min_velocity =
      node->declare_parameter<double>(ego_path_ns + "min_velocity");
    p.ego_predicted_path_params.acceleration =
      node->declare_parameter<double>(ego_path_ns + "min_acceleration");
    p.ego_predicted_path_params.time_horizon_for_front_object =
      node->declare_parameter<double>(ego_path_ns + "time_horizon_for_front_object");
    p.ego_predicted_path_params.time_horizon_for_rear_object =
      node->declare_parameter<double>(ego_path_ns + "time_horizon_for_rear_object");
    p.ego_predicted_path_params.time_resolution =
      node->declare_parameter<double>(ego_path_ns + "time_resolution");
    p.ego_predicted_path_params.delay_until_departure =
      node->declare_parameter<double>(ego_path_ns + "delay_until_departure");
  }

  // ObjectFilteringParams
  const std::string obj_filter_ns = base_ns + "target_filtering.";
  {
    p.objects_filtering_params.safety_check_time_horizon =
      node->declare_parameter<double>(obj_filter_ns + "safety_check_time_horizon");
    p.objects_filtering_params.safety_check_time_resolution =
      node->declare_parameter<double>(obj_filter_ns + "safety_check_time_resolution");
    p.objects_filtering_params.object_check_forward_distance =
      node->declare_parameter<double>(obj_filter_ns + "object_check_forward_distance");
    p.objects_filtering_params.object_check_backward_distance =
      node->declare_parameter<double>(obj_filter_ns + "object_check_backward_distance");
    p.objects_filtering_params.ignore_object_velocity_threshold =
      node->declare_parameter<double>(obj_filter_ns + "ignore_object_velocity_threshold");
    p.objects_filtering_params.include_opposite_lane =
      node->declare_parameter<bool>(obj_filter_ns + "include_opposite_lane");
    p.objects_filtering_params.invert_opposite_lane =
      node->declare_parameter<bool>(obj_filter_ns + "invert_opposite_lane");
    p.objects_filtering_params.check_all_predicted_path =
      node->declare_parameter<bool>(obj_filter_ns + "check_all_predicted_path");
    p.objects_filtering_params.use_all_predicted_path =
      node->declare_parameter<bool>(obj_filter_ns + "use_all_predicted_path");
    p.objects_filtering_params.use_predicted_path_outside_lanelet =
      node->declare_parameter<bool>(obj_filter_ns + "use_predicted_path_outside_lanelet");
  }

  // ObjectTypesToCheck
  const std::string obj_types_ns = obj_filter_ns + "object_types_to_check.";
  {
    p.objects_filtering_params.object_types_to_check.check_car =
      node->declare_parameter<bool>(obj_types_ns + "check_car");
    p.objects_filtering_params.object_types_to_check.check_truck =
      node->declare_parameter<bool>(obj_types_ns + "check_truck");
    p.objects_filtering_params.object_types_to_check.check_bus =
      node->declare_parameter<bool>(obj_types_ns + "check_bus");
    p.objects_filtering_params.object_types_to_check.check_trailer =
      node->declare_parameter<bool>(obj_types_ns + "check_trailer");
    p.objects_filtering_params.object_types_to_check.check_unknown =
      node->declare_parameter<bool>(obj_types_ns + "check_unknown");
    p.objects_filtering_params.object_types_to_check.check_bicycle =
      node->declare_parameter<bool>(obj_types_ns + "check_bicycle");
    p.objects_filtering_params.object_types_to_check.check_motorcycle =
      node->declare_parameter<bool>(obj_types_ns + "check_motorcycle");
    p.objects_filtering_params.object_types_to_check.check_pedestrian =
      node->declare_parameter<bool>(obj_types_ns + "check_pedestrian");
  }

  // ObjectLaneConfiguration
  const std::string obj_lane_ns = obj_filter_ns + "object_lane_configuration.";
  {
    p.objects_filtering_params.object_lane_configuration.check_current_lane =
      node->declare_parameter<bool>(obj_lane_ns + "check_current_lane");
    p.objects_filtering_params.object_lane_configuration.check_right_lane =
      node->declare_parameter<bool>(obj_lane_ns + "check_right_side_lane");
    p.objects_filtering_params.object_lane_configuration.check_left_lane =
      node->declare_parameter<bool>(obj_lane_ns + "check_left_side_lane");
    p.objects_filtering_params.object_lane_configuration.check_shoulder_lane =
      node->declare_parameter<bool>(obj_lane_ns + "check_shoulder_lane");
    p.objects_filtering_params.object_lane_configuration.check_other_lane =
      node->declare_parameter<bool>(obj_lane_ns + "check_other_lane");
  }

  // SafetyCheckParams
  const std::string safety_check_ns = base_ns + "safety_check_params.";
  {
    p.safety_check_params.enable_safety_check =
      node->declare_parameter<bool>(safety_check_ns + "enable_safety_check");
    p.safety_check_params.hysteresis_factor_expand_rate =
      node->declare_parameter<double>(safety_check_ns + "hysteresis_factor_expand_rate");
    p.safety_check_params.backward_path_length =
      node->declare_parameter<double>(safety_check_ns + "backward_path_length");
    p.safety_check_params.forward_path_length =
      node->declare_parameter<double>(safety_check_ns + "forward_path_length");
    p.safety_check_params.publish_debug_marker =
      node->declare_parameter<bool>(safety_check_ns + "publish_debug_marker");
  }

  // RSSparams
  const std::string rss_ns = safety_check_ns + "rss_params.";
  {
    p.safety_check_params.rss_params.rear_vehicle_reaction_time =
      node->declare_parameter<double>(rss_ns + "rear_vehicle_reaction_time");
    p.safety_check_params.rss_params.rear_vehicle_safety_time_margin =
      node->declare_parameter<double>(rss_ns + "rear_vehicle_safety_time_margin");
    p.safety_check_params.rss_params.lateral_distance_max_threshold =
      node->declare_parameter<double>(rss_ns + "lateral_distance_max_threshold");
    p.safety_check_params.rss_params.longitudinal_distance_min_threshold =
      node->declare_parameter<double>(rss_ns + "longitudinal_distance_min_threshold");
    p.safety_check_params.rss_params.longitudinal_velocity_delta_time =
      node->declare_parameter<double>(rss_ns + "longitudinal_velocity_delta_time");
  }

  // surround moving obstacle check
  std::string surround_moving_obstacle_check_ns = ns + "surround_moving_obstacle_check.";
  {
    p.search_radius =
      node->declare_parameter<double>(surround_moving_obstacle_check_ns + "search_radius");
    p.th_moving_obstacle_velocity = node->declare_parameter<double>(
      surround_moving_obstacle_check_ns + "th_moving_obstacle_velocity");
    // ObjectTypesToCheck
    std::string obj_types_ns = surround_moving_obstacle_check_ns + "object_types_to_check.";
    {
      p.surround_moving_obstacles_type_to_check.check_car =
        node->declare_parameter<bool>(obj_types_ns + "check_car");
      p.surround_moving_obstacles_type_to_check.check_truck =
        node->declare_parameter<bool>(obj_types_ns + "check_truck");
      p.surround_moving_obstacles_type_to_check.check_bus =
        node->declare_parameter<bool>(obj_types_ns + "check_bus");
      p.surround_moving_obstacles_type_to_check.check_trailer =
        node->declare_parameter<bool>(obj_types_ns + "check_trailer");
      p.surround_moving_obstacles_type_to_check.check_unknown =
        node->declare_parameter<bool>(obj_types_ns + "check_unknown");
      p.surround_moving_obstacles_type_to_check.check_bicycle =
        node->declare_parameter<bool>(obj_types_ns + "check_bicycle");
      p.surround_moving_obstacles_type_to_check.check_motorcycle =
        node->declare_parameter<bool>(obj_types_ns + "check_motorcycle");
      p.surround_moving_obstacles_type_to_check.check_pedestrian =
        node->declare_parameter<bool>(obj_types_ns + "check_pedestrian");
    }
  }

  // debug
  std::string debug_ns = ns + "debug.";
  {
    p.print_debug_info = node->declare_parameter<bool>(debug_ns + "print_debug_info");
  }

  // validation of parameters
  if (p.lateral_acceleration_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      node->get_logger().get_child(name()),
      "lateral_acceleration_sampling_num must be positive integer. Given parameter: "
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

  [[maybe_unused]] const std::string ns = name_ + ".";

  std::for_each(observers_.begin(), observers_.end(), [&](const auto & observer) {
    if (!observer.expired()) {
      const auto start_planner_ptr = std::dynamic_pointer_cast<StartPlannerModule>(observer.lock());
      if (start_planner_ptr) {
        start_planner_ptr->updateModuleParams(p);
      }
    }
  });
}

bool StartPlannerModuleManager::isSimultaneousExecutableAsApprovedModule() const
{
  if (observers_.empty()) {
    return config_.enable_simultaneous_execution_as_approved_module;
  }

  const auto checker = [this](const SceneModuleObserver & observer) {
    if (observer.expired()) {
      return config_.enable_simultaneous_execution_as_approved_module;
    }

    const auto start_planner_ptr = std::dynamic_pointer_cast<StartPlannerModule>(observer.lock());

    // Currently simultaneous execution with other modules is not supported while backward driving
    if (!start_planner_ptr->isDrivingForward()) {
      return false;
    }

    // Other modules are not needed when freespace planning
    if (start_planner_ptr->isFreespacePlanning()) {
      return false;
    }

    return config_.enable_simultaneous_execution_as_approved_module;
  };

  return std::all_of(observers_.begin(), observers_.end(), checker);
}

bool StartPlannerModuleManager::isSimultaneousExecutableAsCandidateModule() const
{
  if (observers_.empty()) {
    return config_.enable_simultaneous_execution_as_candidate_module;
  }

  const auto checker = [this](const SceneModuleObserver & observer) {
    if (observer.expired()) {
      return config_.enable_simultaneous_execution_as_candidate_module;
    }

    const auto start_planner_ptr = std::dynamic_pointer_cast<StartPlannerModule>(observer.lock());

    // Currently simultaneous execution with other modules is not supported while backward driving
    if (start_planner_ptr->isDrivingForward()) {
      return false;
    }

    // Other modules are not needed when freespace planning
    if (start_planner_ptr->isFreespacePlanning()) {
      return false;
    }

    return config_.enable_simultaneous_execution_as_candidate_module;
  };

  return std::all_of(observers_.begin(), observers_.end(), checker);
}
}  // namespace behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_path_planner::StartPlannerModuleManager,
  behavior_path_planner::SceneModuleManagerInterface)
