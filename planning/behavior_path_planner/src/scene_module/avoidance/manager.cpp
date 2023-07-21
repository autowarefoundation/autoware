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

#include "behavior_path_planner/scene_module/avoidance/manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

namespace
{
template <typename T>
T get_parameter(rclcpp::Node * node, const std::string & ns)
{
  if (node->has_parameter(ns)) {
    return node->get_parameter(ns).get_value<T>();
  }

  return node->declare_parameter<T>(ns);
}
}  // namespace

AvoidanceModuleManager::AvoidanceModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config)
: SceneModuleManagerInterface(node, name, config, {"left", "right"})
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;

  AvoidanceParameters p{};
  // general params
  {
    std::string ns = "avoidance.";
    p.resample_interval_for_planning =
      get_parameter<double>(node, ns + "resample_interval_for_planning");
    p.resample_interval_for_output =
      get_parameter<double>(node, ns + "resample_interval_for_output");
    p.detection_area_right_expand_dist =
      get_parameter<double>(node, ns + "detection_area_right_expand_dist");
    p.detection_area_left_expand_dist =
      get_parameter<double>(node, ns + "detection_area_left_expand_dist");
    p.enable_bound_clipping = get_parameter<bool>(node, ns + "enable_bound_clipping");
    p.enable_update_path_when_object_is_gone =
      get_parameter<bool>(node, ns + "enable_update_path_when_object_is_gone");
    p.enable_force_avoidance_for_stopped_vehicle =
      get_parameter<bool>(node, ns + "enable_force_avoidance_for_stopped_vehicle");
    p.enable_safety_check = get_parameter<bool>(node, ns + "enable_safety_check");
    p.enable_yield_maneuver = get_parameter<bool>(node, ns + "enable_yield_maneuver");
    p.enable_yield_maneuver_during_shifting =
      get_parameter<bool>(node, ns + "enable_yield_maneuver_during_shifting");
    p.disable_path_update = get_parameter<bool>(node, ns + "disable_path_update");
    p.publish_debug_marker = get_parameter<bool>(node, ns + "publish_debug_marker");
    p.print_debug_info = get_parameter<bool>(node, ns + "print_debug_info");
  }

  // drivable area
  {
    std::string ns = "avoidance.";
    p.use_adjacent_lane = get_parameter<bool>(node, ns + "use_adjacent_lane");
    p.use_opposite_lane = get_parameter<bool>(node, ns + "use_opposite_lane");
    p.use_intersection_areas = get_parameter<bool>(node, ns + "use_intersection_areas");
    p.use_hatched_road_markings = get_parameter<bool>(node, ns + "use_hatched_road_markings");
  }

  // target object
  {
    const auto get_object_param = [&](std::string && ns) {
      ObjectParameter param{};
      param.is_target = get_parameter<bool>(node, ns + "is_target");
      param.moving_speed_threshold = get_parameter<double>(node, ns + "moving_speed_threshold");
      param.moving_time_threshold = get_parameter<double>(node, ns + "moving_time_threshold");
      param.max_expand_ratio = get_parameter<double>(node, ns + "max_expand_ratio");
      param.envelope_buffer_margin = get_parameter<double>(node, ns + "envelope_buffer_margin");
      param.avoid_margin_lateral = get_parameter<double>(node, ns + "avoid_margin_lateral");
      param.safety_buffer_lateral = get_parameter<double>(node, ns + "safety_buffer_lateral");
      param.safety_buffer_longitudinal =
        get_parameter<double>(node, ns + "safety_buffer_longitudinal");
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
      get_parameter<double>(node, ns + "lower_distance_for_polygon_expansion");
    p.upper_distance_for_polygon_expansion =
      get_parameter<double>(node, ns + "upper_distance_for_polygon_expansion");
  }

  // target filtering
  {
    std::string ns = "avoidance.target_filtering.";
    p.threshold_time_force_avoidance_for_stopped_vehicle =
      get_parameter<double>(node, ns + "threshold_time_force_avoidance_for_stopped_vehicle");
    p.object_ignore_section_traffic_light_in_front_distance =
      get_parameter<double>(node, ns + "object_ignore_section_traffic_light_in_front_distance");
    p.object_ignore_section_crosswalk_in_front_distance =
      get_parameter<double>(node, ns + "object_ignore_section_crosswalk_in_front_distance");
    p.object_ignore_section_crosswalk_behind_distance =
      get_parameter<double>(node, ns + "object_ignore_section_crosswalk_behind_distance");
    p.object_check_forward_distance =
      get_parameter<double>(node, ns + "object_check_forward_distance");
    p.object_check_backward_distance =
      get_parameter<double>(node, ns + "object_check_backward_distance");
    p.object_check_goal_distance = get_parameter<double>(node, ns + "object_check_goal_distance");
    p.threshold_distance_object_is_on_center =
      get_parameter<double>(node, ns + "threshold_distance_object_is_on_center");
    p.object_check_shiftable_ratio =
      get_parameter<double>(node, ns + "object_check_shiftable_ratio");
    p.object_check_min_road_shoulder_width =
      get_parameter<double>(node, ns + "object_check_min_road_shoulder_width");
    p.object_last_seen_threshold = get_parameter<double>(node, ns + "object_last_seen_threshold");
  }

  // safety check
  {
    std::string ns = "avoidance.safety_check.";
    p.safety_check_backward_distance =
      get_parameter<double>(node, ns + "safety_check_backward_distance");
    p.safety_check_time_horizon = get_parameter<double>(node, ns + "safety_check_time_horizon");
    p.safety_check_idling_time = get_parameter<double>(node, ns + "safety_check_idling_time");
    p.safety_check_accel_for_rss = get_parameter<double>(node, ns + "safety_check_accel_for_rss");
    p.safety_check_hysteresis_factor =
      get_parameter<double>(node, ns + "safety_check_hysteresis_factor");
    p.safety_check_ego_offset = get_parameter<double>(node, ns + "safety_check_ego_offset");
  }

  // avoidance maneuver (lateral)
  {
    std::string ns = "avoidance.avoidance.lateral.";
    p.road_shoulder_safety_margin = get_parameter<double>(node, ns + "road_shoulder_safety_margin");
    p.lateral_execution_threshold = get_parameter<double>(node, ns + "lateral_execution_threshold");
    p.lateral_small_shift_threshold =
      get_parameter<double>(node, ns + "lateral_small_shift_threshold");
    p.max_right_shift_length = get_parameter<double>(node, ns + "max_right_shift_length");
    p.max_left_shift_length = get_parameter<double>(node, ns + "max_left_shift_length");
  }

  // avoidance maneuver (longitudinal)
  {
    std::string ns = "avoidance.avoidance.longitudinal.";
    p.prepare_time = get_parameter<double>(node, ns + "prepare_time");
    p.min_prepare_distance = get_parameter<double>(node, ns + "min_prepare_distance");
    p.min_slow_down_speed = get_parameter<double>(node, ns + "min_slow_down_speed");
    p.buf_slow_down_speed = get_parameter<double>(node, ns + "buf_slow_down_speed");
    p.nominal_avoidance_speed = get_parameter<double>(node, ns + "nominal_avoidance_speed");
  }

  // yield
  {
    std::string ns = "avoidance.yield.";
    p.yield_velocity = get_parameter<double>(node, ns + "yield_velocity");
  }

  // stop
  {
    std::string ns = "avoidance.stop.";
    p.stop_max_distance = get_parameter<double>(node, ns + "max_distance");
    p.stop_buffer = get_parameter<double>(node, ns + "stop_buffer");
  }

  // constraints
  {
    std::string ns = "avoidance.constraints.";
    p.use_constraints_for_decel = get_parameter<bool>(node, ns + "use_constraints_for_decel");
  }

  // constraints (longitudinal)
  {
    std::string ns = "avoidance.constraints.longitudinal.";
    p.nominal_deceleration = get_parameter<double>(node, ns + "nominal_deceleration");
    p.nominal_jerk = get_parameter<double>(node, ns + "nominal_jerk");
    p.max_deceleration = get_parameter<double>(node, ns + "max_deceleration");
    p.max_jerk = get_parameter<double>(node, ns + "max_jerk");
    p.max_acceleration = get_parameter<double>(node, ns + "max_acceleration");
  }

  // constraints (lateral)
  {
    std::string ns = "avoidance.constraints.lateral.";
    p.velocity_map = get_parameter<std::vector<double>>(node, ns + "velocity");
    p.lateral_max_accel_map = get_parameter<std::vector<double>>(node, ns + "max_accel_values");
    p.lateral_min_jerk_map = get_parameter<std::vector<double>>(node, ns + "min_jerk_values");
    p.lateral_max_jerk_map = get_parameter<std::vector<double>>(node, ns + "max_jerk_values");

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

  // velocity matrix
  {
    std::string ns = "avoidance.target_velocity_matrix.";
    p.col_size = get_parameter<int>(node, ns + "col_size");
    p.target_velocity_matrix = get_parameter<std::vector<double>>(node, ns + "matrix");
  }

  // shift line pipeline
  {
    std::string ns = "avoidance.shift_line_pipeline.";
    p.quantize_filter_threshold =
      get_parameter<double>(node, ns + "trim.quantize_filter_threshold");
    p.same_grad_filter_1_threshold =
      get_parameter<double>(node, ns + "trim.same_grad_filter_1_threshold");
    p.same_grad_filter_2_threshold =
      get_parameter<double>(node, ns + "trim.same_grad_filter_2_threshold");
    p.same_grad_filter_3_threshold =
      get_parameter<double>(node, ns + "trim.same_grad_filter_3_threshold");
    p.sharp_shift_filter_threshold =
      get_parameter<double>(node, ns + "trim.sharp_shift_filter_threshold");
  }

  parameters_ = std::make_shared<AvoidanceParameters>(p);
}

void AvoidanceModuleManager::updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using tier4_autoware_utils::updateParam;

  auto p = parameters_;

  {
    const std::string ns = "avoidance.";
    updateParam<bool>(parameters, ns + "enable_safety_check", p->enable_safety_check);
    updateParam<bool>(parameters, ns + "publish_debug_marker", p->publish_debug_marker);
    updateParam<bool>(parameters, ns + "print_debug_info", p->print_debug_info);
  }

  const auto update_object_param = [&p, &parameters](
                                     const auto & semantic, const std::string & ns) {
    auto & config = p->object_parameters.at(semantic);
    updateParam<bool>(parameters, ns + "is_target", config.is_target);
    updateParam<double>(parameters, ns + "moving_speed_threshold", config.moving_speed_threshold);
    updateParam<double>(parameters, ns + "moving_time_threshold", config.moving_time_threshold);
    updateParam<double>(parameters, ns + "max_expand_ratio", config.max_expand_ratio);
    updateParam<double>(parameters, ns + "envelope_buffer_margin", config.envelope_buffer_margin);
    updateParam<double>(parameters, ns + "avoid_margin_lateral", config.avoid_margin_lateral);
    updateParam<double>(parameters, ns + "safety_buffer_lateral", config.safety_buffer_lateral);
    updateParam<double>(
      parameters, ns + "safety_buffer_longitudinal", config.safety_buffer_longitudinal);
  };

  {
    const std::string ns = "avoidance.target_object.";
    update_object_param(ObjectClassification::MOTORCYCLE, ns + "motorcycle.");
    update_object_param(ObjectClassification::CAR, ns + "car.");
    update_object_param(ObjectClassification::TRUCK, ns + "truck.");
    update_object_param(ObjectClassification::TRAILER, ns + "trailer.");
    update_object_param(ObjectClassification::BUS, ns + "bus.");
    update_object_param(ObjectClassification::PEDESTRIAN, ns + "pedestrian.");
    update_object_param(ObjectClassification::BICYCLE, ns + "bicycle.");
    update_object_param(ObjectClassification::UNKNOWN, ns + "unknown.");

    updateParam<double>(
      parameters, ns + "lower_distance_for_polygon_expansion",
      p->lower_distance_for_polygon_expansion);
    updateParam<double>(
      parameters, ns + "upper_distance_for_polygon_expansion",
      p->upper_distance_for_polygon_expansion);
  }

  {
    const std::string ns = "avoidance.avoidance.lateral.";
    updateParam<double>(
      parameters, ns + "lateral_execution_threshold", p->lateral_execution_threshold);
    updateParam<double>(
      parameters, ns + "lateral_small_shift_threshold", p->lateral_small_shift_threshold);
    updateParam<double>(
      parameters, ns + "road_shoulder_safety_margin", p->road_shoulder_safety_margin);
  }

  {
    const std::string ns = "avoidance.avoidance.longitudinal.";
    updateParam<double>(parameters, ns + "prepare_time", p->prepare_time);
    updateParam<double>(parameters, ns + "min_slow_down_speed", p->min_slow_down_speed);
    updateParam<double>(parameters, ns + "buf_slow_down_speed", p->buf_slow_down_speed);
  }

  {
    const std::string ns = "avoidance.stop.";
    updateParam<double>(parameters, ns + "max_distance", p->stop_max_distance);
    updateParam<double>(parameters, ns + "stop_buffer", p->stop_buffer);
  }

  {
    const std::string ns = "avoidance.constrains.lateral.";

    std::vector<double> velocity_map;
    updateParam<std::vector<double>>(parameters, ns + "velocity", velocity_map);
    std::vector<double> lateral_max_accel_map;
    updateParam<std::vector<double>>(parameters, ns + "max_accel_values", lateral_max_accel_map);
    std::vector<double> lateral_min_jerk_map;
    updateParam<std::vector<double>>(parameters, ns + "min_jerk_values", lateral_min_jerk_map);
    std::vector<double> lateral_max_jerk_map;
    updateParam<std::vector<double>>(parameters, ns + "max_jerk_values", lateral_max_jerk_map);

    if (!velocity_map.empty()) {
      p->velocity_map = velocity_map;
    }

    if (!velocity_map.empty() && velocity_map.size() == lateral_max_accel_map.size()) {
      p->lateral_max_accel_map = lateral_max_accel_map;
    }

    if (!velocity_map.empty() && velocity_map.size() == lateral_min_jerk_map.size()) {
      p->lateral_min_jerk_map = lateral_min_jerk_map;
    }

    if (!velocity_map.empty() && velocity_map.size() == lateral_max_jerk_map.size()) {
      p->lateral_max_jerk_map = lateral_max_jerk_map;
    }
  }

  {
    const std::string ns = "avoidance.shift_line_pipeline.";
    updateParam<double>(
      parameters, ns + "trim.quantize_filter_threshold", p->quantize_filter_threshold);
    updateParam<double>(
      parameters, ns + "trim.same_grad_filter_1_threshold", p->same_grad_filter_1_threshold);
    updateParam<double>(
      parameters, ns + "trim.same_grad_filter_2_threshold", p->same_grad_filter_2_threshold);
    updateParam<double>(
      parameters, ns + "trim.same_grad_filter_3_threshold", p->same_grad_filter_3_threshold);
    updateParam<double>(
      parameters, ns + "trim.sharp_shift_filter_threshold", p->sharp_shift_filter_threshold);
  }

  std::for_each(registered_modules_.begin(), registered_modules_.end(), [&p](const auto & m) {
    m->updateModuleParams(p);
  });
}
}  // namespace behavior_path_planner
