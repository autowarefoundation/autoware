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
#ifndef AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__PARAMETER_HELPER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__PARAMETER_HELPER_HPP_

#include "autoware/universe_utils/ros/parameter.hpp"

#include <autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp>
#include <rclcpp/node.hpp>

#include <autoware_perception_msgs/msg/detail/object_classification__struct.hpp>

#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::universe_utils::getOrDeclareParameter;
using autoware_perception_msgs::msg::ObjectClassification;

AvoidanceParameters getParameter(rclcpp::Node * node)
{
  AvoidanceParameters p{};
  // general params
  {
    const std::string ns = "avoidance.";
    p.resample_interval_for_planning =
      getOrDeclareParameter<double>(*node, ns + "resample_interval_for_planning");
    p.resample_interval_for_output =
      getOrDeclareParameter<double>(*node, ns + "resample_interval_for_output");
    p.path_generation_method =
      getOrDeclareParameter<std::string>(*node, ns + "path_generation_method");
  }

  // drivable area
  {
    const std::string ns = "avoidance.";
    p.use_lane_type = getOrDeclareParameter<std::string>(*node, ns + "use_lane_type");
    p.use_intersection_areas = getOrDeclareParameter<bool>(*node, ns + "use_intersection_areas");
    p.use_hatched_road_markings =
      getOrDeclareParameter<bool>(*node, ns + "use_hatched_road_markings");
    p.use_freespace_areas = getOrDeclareParameter<bool>(*node, ns + "use_freespace_areas");
  }

  // target object
  {
    const auto get_object_param = [&](std::string && ns) {
      ObjectParameter param{};
      param.moving_speed_threshold = getOrDeclareParameter<double>(*node, ns + "th_moving_speed");
      param.moving_time_threshold = getOrDeclareParameter<double>(*node, ns + "th_moving_time");
      param.max_expand_ratio = getOrDeclareParameter<double>(*node, ns + "max_expand_ratio");
      param.envelope_buffer_margin =
        getOrDeclareParameter<double>(*node, ns + "envelope_buffer_margin");
      param.lateral_soft_margin =
        getOrDeclareParameter<double>(*node, ns + "lateral_margin.soft_margin");
      param.lateral_hard_margin =
        getOrDeclareParameter<double>(*node, ns + "lateral_margin.hard_margin");
      param.lateral_hard_margin_for_parked_vehicle =
        getOrDeclareParameter<double>(*node, ns + "lateral_margin.hard_margin_for_parked_vehicle");
      param.longitudinal_margin = getOrDeclareParameter<double>(*node, ns + "longitudinal_margin");
      param.th_error_eclipse_long_radius =
        getOrDeclareParameter<double>(*node, ns + "th_error_eclipse_long_radius");
      return param;
    };

    const std::string ns = "avoidance.target_object.";
    p.object_parameters.emplace(
      ObjectClassification::MOTORCYCLE, get_object_param(ns + "motorcycle."));
    p.object_parameters.emplace(ObjectClassification::CAR, get_object_param(ns + "car."));
    p.object_parameters.emplace(ObjectClassification::TRUCK, get_object_param(ns + "truck."));
    p.object_parameters.emplace(ObjectClassification::TRAILER, get_object_param(ns + "trailer."));
    p.object_parameters.emplace(ObjectClassification::BUS, get_object_param(ns + "bus."));
    p.object_parameters.emplace(
      ObjectClassification::PEDESTRIAN, get_object_param(ns + "pedestrian."));
    p.object_parameters.emplace(ObjectClassification::BICYCLE, get_object_param(ns + "bicycle."));
    p.object_parameters.emplace(ObjectClassification::UNKNOWN, get_object_param(ns + "unknown."));

    p.lower_distance_for_polygon_expansion =
      getOrDeclareParameter<double>(*node, ns + "lower_distance_for_polygon_expansion");
    p.upper_distance_for_polygon_expansion =
      getOrDeclareParameter<double>(*node, ns + "upper_distance_for_polygon_expansion");
  }

  // target filtering
  {
    const auto set_target_flag = [&](const uint8_t & object_type, const std::string & ns) {
      if (p.object_parameters.count(object_type) == 0) {
        return;
      }
      p.object_parameters.at(object_type).is_avoidance_target =
        getOrDeclareParameter<bool>(*node, ns);
    };

    const std::string ns = "avoidance.target_filtering.";
    set_target_flag(ObjectClassification::CAR, ns + "target_type.car");
    set_target_flag(ObjectClassification::TRUCK, ns + "target_type.truck");
    set_target_flag(ObjectClassification::TRAILER, ns + "target_type.trailer");
    set_target_flag(ObjectClassification::BUS, ns + "target_type.bus");
    set_target_flag(ObjectClassification::PEDESTRIAN, ns + "target_type.pedestrian");
    set_target_flag(ObjectClassification::BICYCLE, ns + "target_type.bicycle");
    set_target_flag(ObjectClassification::MOTORCYCLE, ns + "target_type.motorcycle");
    set_target_flag(ObjectClassification::UNKNOWN, ns + "target_type.unknown");

    p.object_check_goal_distance =
      getOrDeclareParameter<double>(*node, ns + "object_check_goal_distance");
    p.object_check_return_pose_distance =
      getOrDeclareParameter<double>(*node, ns + "object_check_return_pose_distance");
    p.object_check_yaw_deviation =
      getOrDeclareParameter<double>(*node, ns + "intersection.yaw_deviation");
    p.object_last_seen_threshold =
      getOrDeclareParameter<double>(*node, ns + "max_compensation_time");
  }

  {
    const std::string ns = "avoidance.target_filtering.parked_vehicle.";
    p.threshold_distance_object_is_on_center =
      getOrDeclareParameter<double>(*node, ns + "th_offset_from_centerline");
    p.object_check_shiftable_ratio =
      getOrDeclareParameter<double>(*node, ns + "th_shiftable_ratio");
    p.object_check_min_road_shoulder_width =
      getOrDeclareParameter<double>(*node, ns + "min_road_shoulder_width");
  }

  {
    const std::string ns = "avoidance.target_filtering.merging_vehicle.";
    p.th_overhang_distance = getOrDeclareParameter<double>(*node, ns + "th_overhang_distance");
  }

  {
    const std::string ns = "avoidance.target_filtering.avoidance_for_ambiguous_vehicle.";
    p.policy_ambiguous_vehicle = getOrDeclareParameter<std::string>(*node, ns + "policy");
    p.wait_and_see_target_behaviors =
      getOrDeclareParameter<std::vector<std::string>>(*node, ns + "wait_and_see.target_behaviors");
    p.wait_and_see_th_closest_distance =
      getOrDeclareParameter<double>(*node, ns + "wait_and_see.th_closest_distance");
    p.time_threshold_for_ambiguous_vehicle =
      getOrDeclareParameter<double>(*node, ns + "condition.th_stopped_time");
    p.distance_threshold_for_ambiguous_vehicle =
      getOrDeclareParameter<double>(*node, ns + "condition.th_moving_distance");
    p.object_ignore_section_traffic_light_in_front_distance =
      getOrDeclareParameter<double>(*node, ns + "ignore_area.traffic_light.front_distance");
    p.object_ignore_section_crosswalk_in_front_distance =
      getOrDeclareParameter<double>(*node, ns + "ignore_area.crosswalk.front_distance");
    p.object_ignore_section_crosswalk_behind_distance =
      getOrDeclareParameter<double>(*node, ns + "ignore_area.crosswalk.behind_distance");
  }

  {
    const std::string ns = "avoidance.target_filtering.freespace.";
    p.freespace_condition_th_stopped_time =
      getOrDeclareParameter<double>(*node, ns + "condition.th_stopped_time");
  }

  {
    const std::string ns = "avoidance.target_filtering.detection_area.";
    p.use_static_detection_area = getOrDeclareParameter<bool>(*node, ns + "static");
    p.object_check_min_forward_distance =
      getOrDeclareParameter<double>(*node, ns + "min_forward_distance");
    p.object_check_max_forward_distance =
      getOrDeclareParameter<double>(*node, ns + "max_forward_distance");
    p.object_check_backward_distance =
      getOrDeclareParameter<double>(*node, ns + "backward_distance");
  }

  // safety check general params
  {
    const auto set_target_flag = [&](const uint8_t & object_type, const std::string & ns) {
      if (p.object_parameters.count(object_type) == 0) {
        return;
      }
      p.object_parameters.at(object_type).is_safety_check_target =
        getOrDeclareParameter<bool>(*node, ns);
    };

    const std::string ns = "avoidance.safety_check.";
    set_target_flag(ObjectClassification::CAR, ns + "target_type.car");
    set_target_flag(ObjectClassification::TRUCK, ns + "target_type.truck");
    set_target_flag(ObjectClassification::TRAILER, ns + "target_type.trailer");
    set_target_flag(ObjectClassification::BUS, ns + "target_type.bus");
    set_target_flag(ObjectClassification::PEDESTRIAN, ns + "target_type.pedestrian");
    set_target_flag(ObjectClassification::BICYCLE, ns + "target_type.bicycle");
    set_target_flag(ObjectClassification::MOTORCYCLE, ns + "target_type.motorcycle");
    set_target_flag(ObjectClassification::UNKNOWN, ns + "target_type.unknown");

    p.enable_safety_check = getOrDeclareParameter<bool>(*node, ns + "enable");
    p.check_current_lane = getOrDeclareParameter<bool>(*node, ns + "check_current_lane");
    p.check_shift_side_lane = getOrDeclareParameter<bool>(*node, ns + "check_shift_side_lane");
    p.check_other_side_lane = getOrDeclareParameter<bool>(*node, ns + "check_other_side_lane");
    p.check_unavoidable_object =
      getOrDeclareParameter<bool>(*node, ns + "check_unavoidable_object");
    p.check_other_object = getOrDeclareParameter<bool>(*node, ns + "check_other_object");
    p.check_all_predicted_path =
      getOrDeclareParameter<bool>(*node, ns + "check_all_predicted_path");
    p.safety_check_backward_distance =
      getOrDeclareParameter<double>(*node, ns + "safety_check_backward_distance");
    p.hysteresis_factor_expand_rate =
      getOrDeclareParameter<double>(*node, ns + "hysteresis_factor_expand_rate");
    p.hysteresis_factor_safe_count =
      getOrDeclareParameter<int>(*node, ns + "hysteresis_factor_safe_count");
    p.collision_check_yaw_diff_threshold =
      getOrDeclareParameter<double>(*node, ns + "collision_check_yaw_diff_threshold");
  }

  // safety check predicted path params
  {
    const std::string ns = "avoidance.safety_check.";
    p.ego_predicted_path_params.min_velocity =
      getOrDeclareParameter<double>(*node, ns + "min_velocity");
    p.ego_predicted_path_params.max_velocity =
      getOrDeclareParameter<double>(*node, ns + "max_velocity");
    p.ego_predicted_path_params.acceleration =
      getOrDeclareParameter<double>(*node, "avoidance.constraints.longitudinal.max_acceleration");
    p.ego_predicted_path_params.time_horizon_for_front_object =
      getOrDeclareParameter<double>(*node, ns + "time_horizon_for_front_object");
    p.ego_predicted_path_params.time_horizon_for_rear_object =
      getOrDeclareParameter<double>(*node, ns + "time_horizon_for_rear_object");
    p.ego_predicted_path_params.time_resolution =
      getOrDeclareParameter<double>(*node, ns + "time_resolution");
    p.ego_predicted_path_params.delay_until_departure =
      getOrDeclareParameter<double>(*node, ns + "delay_until_departure");
  }

  // safety check rss params
  {
    const std::string ns = "avoidance.safety_check.";
    p.rss_params.extended_polygon_policy =
      getOrDeclareParameter<std::string>(*node, ns + "extended_polygon_policy");
    p.rss_params.longitudinal_distance_min_threshold =
      getOrDeclareParameter<double>(*node, ns + "longitudinal_distance_min_threshold");
    p.rss_params.longitudinal_velocity_delta_time =
      getOrDeclareParameter<double>(*node, ns + "longitudinal_velocity_delta_time");
    p.rss_params.front_vehicle_deceleration =
      getOrDeclareParameter<double>(*node, ns + "expected_front_deceleration");
    p.rss_params.rear_vehicle_deceleration =
      getOrDeclareParameter<double>(*node, ns + "expected_rear_deceleration");
    p.rss_params.rear_vehicle_reaction_time =
      getOrDeclareParameter<double>(*node, ns + "rear_vehicle_reaction_time");
    p.rss_params.rear_vehicle_safety_time_margin =
      getOrDeclareParameter<double>(*node, ns + "rear_vehicle_safety_time_margin");
    p.rss_params.lateral_distance_max_threshold =
      getOrDeclareParameter<double>(*node, ns + "lateral_distance_max_threshold");
  }

  // avoidance maneuver (lateral)
  {
    const std::string ns = "avoidance.avoidance.lateral.";
    p.soft_drivable_bound_margin =
      getOrDeclareParameter<double>(*node, ns + "soft_drivable_bound_margin");
    p.hard_drivable_bound_margin =
      getOrDeclareParameter<double>(*node, ns + "hard_drivable_bound_margin");
    p.lateral_execution_threshold = getOrDeclareParameter<double>(*node, ns + "th_avoid_execution");
    p.lateral_small_shift_threshold =
      getOrDeclareParameter<double>(*node, ns + "th_small_shift_length");
    p.max_right_shift_length = getOrDeclareParameter<double>(*node, ns + "max_right_shift_length");
    p.max_left_shift_length = getOrDeclareParameter<double>(*node, ns + "max_left_shift_length");
    p.max_deviation_from_lane =
      getOrDeclareParameter<double>(*node, ns + "max_deviation_from_lane");
    p.ratio_for_return_shift_approval =
      getOrDeclareParameter<double>(*node, ns + "ratio_for_return_shift_approval");
    if (p.ratio_for_return_shift_approval < 0.0 || p.ratio_for_return_shift_approval > 1.0) {
      throw std::domain_error(
        "ratio_for_return_shift_approval should be within range of 0.0 to 1.0");
    }
  }

  // avoidance maneuver (longitudinal)
  {
    const std::string ns = "avoidance.avoidance.longitudinal.";
    p.min_prepare_time = getOrDeclareParameter<double>(*node, ns + "min_prepare_time");
    p.max_prepare_time = getOrDeclareParameter<double>(*node, ns + "max_prepare_time");
    p.min_prepare_distance = getOrDeclareParameter<double>(*node, ns + "min_prepare_distance");
    p.min_slow_down_speed = getOrDeclareParameter<double>(*node, ns + "min_slow_down_speed");
    p.buf_slow_down_speed = getOrDeclareParameter<double>(*node, ns + "buf_slow_down_speed");
    p.nominal_avoidance_speed =
      getOrDeclareParameter<double>(*node, ns + "nominal_avoidance_speed");
    p.consider_front_overhang = getOrDeclareParameter<bool>(*node, ns + "consider_front_overhang");
    p.consider_rear_overhang = getOrDeclareParameter<bool>(*node, ns + "consider_rear_overhang");
  }

  // avoidance maneuver (return shift dead line)
  {
    const std::string ns = "avoidance.avoidance.return_dead_line.";
    p.enable_dead_line_for_goal = getOrDeclareParameter<bool>(*node, ns + "goal.enable");
    p.enable_dead_line_for_traffic_light =
      getOrDeclareParameter<bool>(*node, ns + "traffic_light.enable");
    p.dead_line_buffer_for_goal = getOrDeclareParameter<double>(*node, ns + "goal.buffer");
    p.dead_line_buffer_for_traffic_light =
      getOrDeclareParameter<double>(*node, ns + "traffic_light.buffer");
  }

  // cancel
  {
    const std::string ns = "avoidance.cancel.";
    p.enable_cancel_maneuver = getOrDeclareParameter<bool>(*node, ns + "enable");
    p.force_deactivate_duration_time =
      getOrDeclareParameter<double>(*node, ns + "force.duration_time");
  }

  // yield
  {
    const std::string ns = "avoidance.yield.";
    p.enable_yield_maneuver = getOrDeclareParameter<bool>(*node, ns + "enable");
    p.enable_yield_maneuver_during_shifting =
      getOrDeclareParameter<bool>(*node, ns + "enable_during_shifting");
  }

  // stop
  {
    const std::string ns = "avoidance.stop.";
    p.stop_max_distance = getOrDeclareParameter<double>(*node, ns + "max_distance");
    p.stop_buffer = getOrDeclareParameter<double>(*node, ns + "stop_buffer");
  }

  // policy
  {
    const std::string ns = "avoidance.policy.";
    p.policy_approval = getOrDeclareParameter<std::string>(*node, ns + "make_approval_request");
    p.policy_deceleration = getOrDeclareParameter<std::string>(*node, ns + "deceleration");
    p.policy_lateral_margin = getOrDeclareParameter<std::string>(*node, ns + "lateral_margin");
    p.use_shorten_margin_immediately =
      getOrDeclareParameter<bool>(*node, ns + "use_shorten_margin_immediately");

    if (p.policy_approval != "per_shift_line" && p.policy_approval != "per_avoidance_maneuver") {
      throw std::domain_error("invalid policy. please select 'best_effort' or 'reliable'.");
    }

    if (p.policy_deceleration != "best_effort" && p.policy_deceleration != "reliable") {
      throw std::domain_error("invalid policy. please select 'best_effort' or 'reliable'.");
    }

    if (p.policy_lateral_margin != "best_effort" && p.policy_lateral_margin != "reliable") {
      throw std::domain_error("invalid policy. please select 'best_effort' or 'reliable'.");
    }
  }

  // constraints (longitudinal)
  {
    const std::string ns = "avoidance.constraints.longitudinal.";
    p.nominal_deceleration = getOrDeclareParameter<double>(*node, ns + "nominal_deceleration");
    p.nominal_jerk = getOrDeclareParameter<double>(*node, ns + "nominal_jerk");
    p.max_deceleration = getOrDeclareParameter<double>(*node, ns + "max_deceleration");
    p.max_jerk = getOrDeclareParameter<double>(*node, ns + "max_jerk");
    p.max_acceleration = getOrDeclareParameter<double>(*node, ns + "max_acceleration");
    p.min_velocity_to_limit_max_acceleration =
      getOrDeclareParameter<double>(*node, ns + "min_velocity_to_limit_max_acceleration");
  }

  // constraints (lateral)
  {
    const std::string ns = "avoidance.constraints.lateral.";
    p.velocity_map = getOrDeclareParameter<std::vector<double>>(*node, ns + "velocity");
    p.lateral_max_accel_map =
      getOrDeclareParameter<std::vector<double>>(*node, ns + "max_accel_values");
    p.lateral_min_jerk_map =
      getOrDeclareParameter<std::vector<double>>(*node, ns + "min_jerk_values");
    p.lateral_max_jerk_map =
      getOrDeclareParameter<std::vector<double>>(*node, ns + "max_jerk_values");

    if (p.velocity_map.empty()) {
      throw std::domain_error("invalid velocity map.");
    }

    if (p.velocity_map.size() != p.lateral_max_accel_map.size()) {
      throw std::domain_error("inconsistency among the constraints map.");
    }

    if (p.velocity_map.size() != p.lateral_min_jerk_map.size()) {
      throw std::domain_error("inconsistency among the constraints map.");
    }

    if (p.velocity_map.size() != p.lateral_max_jerk_map.size()) {
      throw std::domain_error("inconsistency among the constraints map.");
    }
  }

  // shift line pipeline
  {
    const std::string ns = "avoidance.shift_line_pipeline.";
    p.quantize_size = getOrDeclareParameter<double>(*node, ns + "trim.quantize_size");
    p.th_similar_grad_1 = getOrDeclareParameter<double>(*node, ns + "trim.th_similar_grad_1");
    p.th_similar_grad_2 = getOrDeclareParameter<double>(*node, ns + "trim.th_similar_grad_2");
    p.th_similar_grad_3 = getOrDeclareParameter<double>(*node, ns + "trim.th_similar_grad_3");
  }

  // debug
  {
    const std::string ns = "avoidance.debug.";
    p.enable_other_objects_marker =
      getOrDeclareParameter<bool>(*node, ns + "enable_other_objects_marker");
    p.enable_other_objects_info =
      getOrDeclareParameter<bool>(*node, ns + "enable_other_objects_info");
    p.enable_detection_area_marker =
      getOrDeclareParameter<bool>(*node, ns + "enable_detection_area_marker");
    p.enable_drivable_bound_marker =
      getOrDeclareParameter<bool>(*node, ns + "enable_drivable_bound_marker");
    p.enable_safety_check_marker =
      getOrDeclareParameter<bool>(*node, ns + "enable_safety_check_marker");
    p.enable_shift_line_marker =
      getOrDeclareParameter<bool>(*node, ns + "enable_shift_line_marker");
    p.enable_lane_marker = getOrDeclareParameter<bool>(*node, ns + "enable_lane_marker");
    p.enable_misc_marker = getOrDeclareParameter<bool>(*node, ns + "enable_misc_marker");
  }

  return p;
}
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__PARAMETER_HELPER_HPP_
