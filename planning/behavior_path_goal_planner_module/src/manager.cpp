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

#include "behavior_path_goal_planner_module/manager.hpp"

#include "behavior_path_goal_planner_module/util.hpp"
#include "tier4_autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{
void GoalPlannerModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {""});

  GoalPlannerParameters p;

  const std::string base_ns = "goal_planner.";
  // general params
  {
    p.th_stopped_velocity = node->declare_parameter<double>(base_ns + "th_stopped_velocity");
    p.th_arrived_distance = node->declare_parameter<double>(base_ns + "th_arrived_distance");
    p.th_stopped_time = node->declare_parameter<double>(base_ns + "th_stopped_time");
    p.center_line_path_interval =
      node->declare_parameter<double>(base_ns + "center_line_path_interval");
  }

  // goal search
  {
    const std::string ns = base_ns + "goal_search.";
    p.goal_priority = node->declare_parameter<std::string>(ns + "goal_priority");
    p.minimum_weighted_distance_lateral_weight =
      node->declare_parameter<double>(ns + "minimum_weighted_distance.lateral_weight");
    p.prioritize_goals_before_objects =
      node->declare_parameter<bool>(ns + "prioritize_goals_before_objects");
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
        node->get_logger().get_child(name()),
        "[goal_planner] invalid parking_policy: " << parking_policy_name << std::endl);
      exit(EXIT_FAILURE);
    }
  }

  // occupancy grid map
  {
    const std::string ns = base_ns + "occupancy_grid.";
    p.use_occupancy_grid_for_goal_search =
      node->declare_parameter<bool>(ns + "use_occupancy_grid_for_goal_search");
    p.use_occupancy_grid_for_path_collision_check =
      node->declare_parameter<bool>(ns + "use_occupancy_grid_for_path_collision_check");
    p.use_occupancy_grid_for_goal_longitudinal_margin =
      node->declare_parameter<bool>(ns + "use_occupancy_grid_for_goal_longitudinal_margin");
    p.occupancy_grid_collision_check_margin =
      node->declare_parameter<double>(ns + "occupancy_grid_collision_check_margin");
    p.theta_size = node->declare_parameter<int>(ns + "theta_size");
    p.obstacle_threshold = node->declare_parameter<int>(ns + "obstacle_threshold");
  }

  // object recognition
  {
    const std::string ns = base_ns + "object_recognition.";
    p.use_object_recognition = node->declare_parameter<bool>(ns + "use_object_recognition");
    p.object_recognition_collision_check_margin =
      node->declare_parameter<double>(ns + "object_recognition_collision_check_margin");
    p.object_recognition_collision_check_max_extra_stopping_margin =
      node->declare_parameter<double>(
        ns + "object_recognition_collision_check_max_extra_stopping_margin");
    p.th_moving_object_velocity = node->declare_parameter<double>(ns + "th_moving_object_velocity");
  }

  // pull over general params
  {
    const std::string ns = base_ns + "pull_over.";
    p.pull_over_minimum_request_length =
      node->declare_parameter<double>(ns + "minimum_request_length");
    p.pull_over_velocity = node->declare_parameter<double>(ns + "pull_over_velocity");
    p.pull_over_minimum_velocity =
      node->declare_parameter<double>(ns + "pull_over_minimum_velocity");
    p.decide_path_distance = node->declare_parameter<double>(ns + "decide_path_distance");
    p.maximum_deceleration = node->declare_parameter<double>(ns + "maximum_deceleration");
    p.maximum_jerk = node->declare_parameter<double>(ns + "maximum_jerk");
    p.path_priority = node->declare_parameter<std::string>(ns + "path_priority");
    p.efficient_path_order =
      node->declare_parameter<std::vector<std::string>>(ns + "efficient_path_order");
  }

  // shift parking
  {
    const std::string ns = base_ns + "pull_over.shift_parking.";
    p.enable_shift_parking = node->declare_parameter<bool>(ns + "enable_shift_parking");
    p.shift_sampling_num = node->declare_parameter<int>(ns + "shift_sampling_num");
    p.maximum_lateral_jerk = node->declare_parameter<double>(ns + "maximum_lateral_jerk");
    p.minimum_lateral_jerk = node->declare_parameter<double>(ns + "minimum_lateral_jerk");
    p.deceleration_interval = node->declare_parameter<double>(ns + "deceleration_interval");
    p.after_shift_straight_distance =
      node->declare_parameter<double>(ns + "after_shift_straight_distance");
  }

  // parallel parking common
  {
    const std::string ns = base_ns + "pull_over.parallel_parking.";
    p.parallel_parking_parameters.center_line_path_interval =
      p.center_line_path_interval;  // for geometric parallel parking
  }

  // forward parallel parking forward
  {
    const std::string ns = base_ns + "pull_over.parallel_parking.forward.";
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
    const std::string ns = base_ns + "pull_over.parallel_parking.backward.";
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
    const std::string ns = base_ns + "pull_over.freespace_parking.";
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
    const std::string ns = base_ns + "pull_over.freespace_parking.search_configs.";
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
    const std::string ns = base_ns + "pull_over.freespace_parking.costmap_configs.";
    p.freespace_parking_common_parameters.obstacle_threshold =
      node->declare_parameter<int>(ns + "obstacle_threshold");
  }

  //  freespace parking astar
  {
    const std::string ns = base_ns + "pull_over.freespace_parking.astar.";
    p.astar_parameters.only_behind_solutions =
      node->declare_parameter<bool>(ns + "only_behind_solutions");
    p.astar_parameters.use_back = node->declare_parameter<bool>(ns + "use_back");
    p.astar_parameters.distance_heuristic_weight =
      node->declare_parameter<double>(ns + "distance_heuristic_weight");
  }

  //   freespace parking rrtstar
  {
    const std::string ns = base_ns + "pull_over.freespace_parking.rrtstar.";
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
      node->declare_parameter<double>(base_ns + "stop_condition.maximum_deceleration_for_stop");
    p.maximum_jerk_for_stop =
      node->declare_parameter<double>(base_ns + "stop_condition.maximum_jerk_for_stop");
  }

  const std::string path_safety_check_ns = "goal_planner.path_safety_check.";

  // EgoPredictedPath
  const std::string ego_path_ns = path_safety_check_ns + "ego_predicted_path.";
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
  const std::string obj_filter_ns = path_safety_check_ns + "target_filtering.";
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
  const std::string safety_check_ns = path_safety_check_ns + "safety_check_params.";
  {
    p.safety_check_params.enable_safety_check =
      node->declare_parameter<bool>(safety_check_ns + "enable_safety_check");
    p.safety_check_params.keep_unsafe_time =
      node->declare_parameter<double>(safety_check_ns + "keep_unsafe_time");
    p.safety_check_params.method = node->declare_parameter<std::string>(safety_check_ns + "method");
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

  // IntegralPredictedPolygonParams
  const std::string integral_ns = safety_check_ns + "integral_predicted_polygon_params.";
  {
    p.safety_check_params.integral_predicted_polygon_params.forward_margin =
      node->declare_parameter<double>(integral_ns + "forward_margin");
    p.safety_check_params.integral_predicted_polygon_params.backward_margin =
      node->declare_parameter<double>(integral_ns + "backward_margin");
    p.safety_check_params.integral_predicted_polygon_params.lat_margin =
      node->declare_parameter<double>(integral_ns + "lat_margin");
    p.safety_check_params.integral_predicted_polygon_params.time_horizon =
      node->declare_parameter<double>(integral_ns + "time_horizon");
  }

  // debug
  {
    const std::string ns = base_ns + "debug.";
    p.print_debug_info = node->declare_parameter<bool>(ns + "print_debug_info");
  }

  // validation of parameters
  if (p.shift_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      node->get_logger().get_child(name()),
      "shift_sampling_num must be positive integer. Given parameter: "
        << p.shift_sampling_num << std::endl
        << "Terminating the program...");
    exit(EXIT_FAILURE);
  }
  if (p.maximum_deceleration < 0.0) {
    RCLCPP_FATAL_STREAM(
      node->get_logger().get_child(name()),
      "maximum_deceleration cannot be negative value. Given parameter: "
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

  [[maybe_unused]] const std::string ns = name_ + ".";

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

  return config_.enable_simultaneous_execution_as_approved_module;
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

  return config_.enable_simultaneous_execution_as_candidate_module;
}

}  // namespace behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_path_planner::GoalPlannerModuleManager,
  behavior_path_planner::SceneModuleManagerInterface)
