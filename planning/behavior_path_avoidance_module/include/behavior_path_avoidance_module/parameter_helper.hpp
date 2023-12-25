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
#ifndef BEHAVIOR_PATH_AVOIDANCE_MODULE__PARAMETER_HELPER_HPP_
#define BEHAVIOR_PATH_AVOIDANCE_MODULE__PARAMETER_HELPER_HPP_

#include "tier4_autoware_utils/ros/parameter.hpp"

#include <behavior_path_avoidance_module/data_structs.hpp>
#include <rclcpp/node.hpp>

#include <autoware_auto_perception_msgs/msg/detail/object_classification__struct.hpp>

#include <string>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using tier4_autoware_utils::getOrDeclareParameter;

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
    p.enable_bound_clipping = getOrDeclareParameter<bool>(*node, ns + "enable_bound_clipping");
    p.enable_cancel_maneuver = getOrDeclareParameter<bool>(*node, ns + "enable_cancel_maneuver");
    p.enable_yield_maneuver = getOrDeclareParameter<bool>(*node, ns + "enable_yield_maneuver");
    p.enable_yield_maneuver_during_shifting =
      getOrDeclareParameter<bool>(*node, ns + "enable_yield_maneuver_during_shifting");
    p.disable_path_update = getOrDeclareParameter<bool>(*node, ns + "disable_path_update");
    p.publish_debug_marker = getOrDeclareParameter<bool>(*node, ns + "publish_debug_marker");
    p.print_debug_info = getOrDeclareParameter<bool>(*node, ns + "print_debug_info");
  }

  // drivable area
  {
    const std::string ns = "avoidance.";
    p.use_adjacent_lane = getOrDeclareParameter<bool>(*node, ns + "use_adjacent_lane");
    p.use_opposite_lane = getOrDeclareParameter<bool>(*node, ns + "use_opposite_lane");
    p.use_intersection_areas = getOrDeclareParameter<bool>(*node, ns + "use_intersection_areas");
    p.use_hatched_road_markings =
      getOrDeclareParameter<bool>(*node, ns + "use_hatched_road_markings");
  }

  // target object
  {
    const auto get_object_param = [&](std::string && ns) {
      ObjectParameter param{};
      param.execute_num = getOrDeclareParameter<int>(*node, ns + "execute_num");
      param.moving_speed_threshold =
        getOrDeclareParameter<double>(*node, ns + "moving_speed_threshold");
      param.moving_time_threshold =
        getOrDeclareParameter<double>(*node, ns + "moving_time_threshold");
      param.max_expand_ratio = getOrDeclareParameter<double>(*node, ns + "max_expand_ratio");
      param.envelope_buffer_margin =
        getOrDeclareParameter<double>(*node, ns + "envelope_buffer_margin");
      param.avoid_margin_lateral =
        getOrDeclareParameter<double>(*node, ns + "avoid_margin_lateral");
      param.safety_buffer_lateral =
        getOrDeclareParameter<double>(*node, ns + "safety_buffer_lateral");
      param.safety_buffer_longitudinal =
        getOrDeclareParameter<double>(*node, ns + "safety_buffer_longitudinal");
      param.use_conservative_buffer_longitudinal =
        getOrDeclareParameter<bool>(*node, ns + "use_conservative_buffer_longitudinal");
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
    p.threshold_distance_object_is_on_center =
      getOrDeclareParameter<double>(*node, ns + "threshold_distance_object_is_on_center");
    p.object_check_shiftable_ratio =
      getOrDeclareParameter<double>(*node, ns + "object_check_shiftable_ratio");
    p.object_check_min_road_shoulder_width =
      getOrDeclareParameter<double>(*node, ns + "object_check_min_road_shoulder_width");
    p.object_check_yaw_deviation =
      getOrDeclareParameter<double>(*node, ns + "intersection.yaw_deviation");
    p.object_last_seen_threshold =
      getOrDeclareParameter<double>(*node, ns + "object_last_seen_threshold");
  }

  {
    const std::string ns = "avoidance.target_filtering.force_avoidance.";
    p.enable_force_avoidance_for_stopped_vehicle =
      getOrDeclareParameter<bool>(*node, ns + "enable");
    p.threshold_time_force_avoidance_for_stopped_vehicle =
      getOrDeclareParameter<double>(*node, ns + "time_threshold");
    p.force_avoidance_distance_threshold =
      getOrDeclareParameter<double>(*node, ns + "distance_threshold");
    p.object_ignore_section_traffic_light_in_front_distance =
      getOrDeclareParameter<double>(*node, ns + "ignore_area.traffic_light.front_distance");
    p.object_ignore_section_crosswalk_in_front_distance =
      getOrDeclareParameter<double>(*node, ns + "ignore_area.crosswalk.front_distance");
    p.object_ignore_section_crosswalk_behind_distance =
      getOrDeclareParameter<double>(*node, ns + "ignore_area.crosswalk.behind_distance");
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
    p.soft_road_shoulder_margin =
      getOrDeclareParameter<double>(*node, ns + "soft_road_shoulder_margin");
    p.hard_road_shoulder_margin =
      getOrDeclareParameter<double>(*node, ns + "hard_road_shoulder_margin");
    p.lateral_execution_threshold =
      getOrDeclareParameter<double>(*node, ns + "lateral_execution_threshold");
    p.lateral_small_shift_threshold =
      getOrDeclareParameter<double>(*node, ns + "lateral_small_shift_threshold");
    p.lateral_avoid_check_threshold =
      getOrDeclareParameter<double>(*node, ns + "lateral_avoid_check_threshold");
    p.max_right_shift_length = getOrDeclareParameter<double>(*node, ns + "max_right_shift_length");
    p.max_left_shift_length = getOrDeclareParameter<double>(*node, ns + "max_left_shift_length");
    p.max_deviation_from_lane =
      getOrDeclareParameter<double>(*node, ns + "max_deviation_from_lane");
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

  // yield
  {
    const std::string ns = "avoidance.yield.";
    p.yield_velocity = getOrDeclareParameter<double>(*node, ns + "yield_velocity");
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
    p.quantize_filter_threshold =
      getOrDeclareParameter<double>(*node, ns + "trim.quantize_filter_threshold");
    p.same_grad_filter_1_threshold =
      getOrDeclareParameter<double>(*node, ns + "trim.same_grad_filter_1_threshold");
    p.same_grad_filter_2_threshold =
      getOrDeclareParameter<double>(*node, ns + "trim.same_grad_filter_2_threshold");
    p.same_grad_filter_3_threshold =
      getOrDeclareParameter<double>(*node, ns + "trim.same_grad_filter_3_threshold");
    p.sharp_shift_filter_threshold =
      getOrDeclareParameter<double>(*node, ns + "trim.sharp_shift_filter_threshold");
  }
  return p;
}
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_AVOIDANCE_MODULE__PARAMETER_HELPER_HPP_
