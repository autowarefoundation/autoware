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

#include "behavior_path_planner/scene_module/lane_change/manager.hpp"

#include "tier4_autoware_utils/ros/parameter.hpp"
#include "tier4_autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

using route_handler::Direction;
using utils::convertToSnakeCase;

LaneChangeModuleManager::LaneChangeModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config,
  const Direction direction, const LaneChangeModuleType type)
: SceneModuleManagerInterface(node, name, config, {""}), direction_{direction}, type_{type}
{
  using tier4_autoware_utils::getOrDeclareParameter;

  LaneChangeParameters p{};

  const auto parameter = [](std::string && name) { return "lane_change." + name; };

  // trajectory generation
  p.backward_lane_length = getOrDeclareParameter<double>(*node, parameter("backward_lane_length"));
  p.prediction_time_resolution =
    getOrDeclareParameter<double>(*node, parameter("prediction_time_resolution"));
  p.longitudinal_acc_sampling_num =
    getOrDeclareParameter<int>(*node, parameter("longitudinal_acceleration_sampling_num"));
  p.lateral_acc_sampling_num =
    getOrDeclareParameter<int>(*node, parameter("lateral_acceleration_sampling_num"));

  // parked vehicle detection
  p.object_check_min_road_shoulder_width =
    getOrDeclareParameter<double>(*node, parameter("object_check_min_road_shoulder_width"));
  p.object_shiftable_ratio_threshold =
    getOrDeclareParameter<double>(*node, parameter("object_shiftable_ratio_threshold"));

  // turn signal
  p.min_length_for_turn_signal_activation =
    getOrDeclareParameter<double>(*node, parameter("min_length_for_turn_signal_activation"));
  p.length_ratio_for_turn_signal_deactivation =
    getOrDeclareParameter<double>(*node, parameter("length_ratio_for_turn_signal_deactivation"));

  // acceleration
  p.min_longitudinal_acc = getOrDeclareParameter<double>(*node, parameter("min_longitudinal_acc"));
  p.max_longitudinal_acc = getOrDeclareParameter<double>(*node, parameter("max_longitudinal_acc"));

  // collision check
  p.enable_prepare_segment_collision_check =
    getOrDeclareParameter<bool>(*node, parameter("enable_prepare_segment_collision_check"));
  p.prepare_segment_ignore_object_velocity_thresh = getOrDeclareParameter<double>(
    *node, parameter("prepare_segment_ignore_object_velocity_thresh"));
  p.check_objects_on_current_lanes =
    getOrDeclareParameter<bool>(*node, parameter("check_objects_on_current_lanes"));
  p.check_objects_on_other_lanes =
    getOrDeclareParameter<bool>(*node, parameter("check_objects_on_other_lanes"));
  p.use_all_predicted_path =
    getOrDeclareParameter<bool>(*node, parameter("use_all_predicted_path"));
  p.lane_expansion_left_offset =
    getOrDeclareParameter<double>(*node, parameter("safety_check.lane_expansion.left_offset"));
  p.lane_expansion_right_offset =
    getOrDeclareParameter<double>(*node, parameter("safety_check.lane_expansion.right_offset"));
  // lane change regulations
  p.regulate_on_crosswalk = getOrDeclareParameter<bool>(*node, parameter("regulation.crosswalk"));
  p.regulate_on_intersection =
    getOrDeclareParameter<bool>(*node, parameter("regulation.intersection"));

  // ego vehicle stuck detection
  p.stop_velocity_threshold =
    getOrDeclareParameter<double>(*node, parameter("stuck_detection.velocity"));
  p.stop_time_threshold =
    getOrDeclareParameter<double>(*node, parameter("stuck_detection.stop_time"));

  // safety check
  p.allow_loose_check_for_cancel =
    getOrDeclareParameter<bool>(*node, parameter("safety_check.allow_loose_check_for_cancel"));

  p.rss_params.longitudinal_distance_min_threshold = getOrDeclareParameter<double>(
    *node, parameter("safety_check.execution.longitudinal_distance_min_threshold"));
  p.rss_params.longitudinal_distance_min_threshold = getOrDeclareParameter<double>(
    *node, parameter("safety_check.execution.longitudinal_distance_min_threshold"));
  p.rss_params.longitudinal_velocity_delta_time = getOrDeclareParameter<double>(
    *node, parameter("safety_check.execution.longitudinal_velocity_delta_time"));
  p.rss_params.front_vehicle_deceleration = getOrDeclareParameter<double>(
    *node, parameter("safety_check.execution.expected_front_deceleration"));
  p.rss_params.rear_vehicle_deceleration = getOrDeclareParameter<double>(
    *node, parameter("safety_check.execution.expected_rear_deceleration"));
  p.rss_params.rear_vehicle_reaction_time = getOrDeclareParameter<double>(
    *node, parameter("safety_check.execution.rear_vehicle_reaction_time"));
  p.rss_params.rear_vehicle_safety_time_margin = getOrDeclareParameter<double>(
    *node, parameter("safety_check.execution.rear_vehicle_safety_time_margin"));
  p.rss_params.lateral_distance_max_threshold = getOrDeclareParameter<double>(
    *node, parameter("safety_check.execution.lateral_distance_max_threshold"));

  p.rss_params_for_abort.longitudinal_distance_min_threshold = getOrDeclareParameter<double>(
    *node, parameter("safety_check.cancel.longitudinal_distance_min_threshold"));
  p.rss_params_for_abort.longitudinal_velocity_delta_time = getOrDeclareParameter<double>(
    *node, parameter("safety_check.cancel.longitudinal_velocity_delta_time"));
  p.rss_params_for_abort.front_vehicle_deceleration = getOrDeclareParameter<double>(
    *node, parameter("safety_check.cancel.expected_front_deceleration"));
  p.rss_params_for_abort.rear_vehicle_deceleration = getOrDeclareParameter<double>(
    *node, parameter("safety_check.cancel.expected_rear_deceleration"));
  p.rss_params_for_abort.rear_vehicle_reaction_time = getOrDeclareParameter<double>(
    *node, parameter("safety_check.cancel.rear_vehicle_reaction_time"));
  p.rss_params_for_abort.rear_vehicle_safety_time_margin = getOrDeclareParameter<double>(
    *node, parameter("safety_check.cancel.rear_vehicle_safety_time_margin"));
  p.rss_params_for_abort.lateral_distance_max_threshold = getOrDeclareParameter<double>(
    *node, parameter("safety_check.cancel.lateral_distance_max_threshold"));

  p.rss_params_for_stuck.longitudinal_distance_min_threshold = getOrDeclareParameter<double>(
    *node, parameter("safety_check.stuck.longitudinal_distance_min_threshold"));
  p.rss_params_for_stuck.longitudinal_velocity_delta_time = getOrDeclareParameter<double>(
    *node, parameter("safety_check.stuck.longitudinal_velocity_delta_time"));
  p.rss_params_for_stuck.front_vehicle_deceleration = getOrDeclareParameter<double>(
    *node, parameter("safety_check.stuck.expected_front_deceleration"));
  p.rss_params_for_stuck.rear_vehicle_deceleration = getOrDeclareParameter<double>(
    *node, parameter("safety_check.stuck.expected_rear_deceleration"));
  p.rss_params_for_stuck.rear_vehicle_reaction_time = getOrDeclareParameter<double>(
    *node, parameter("safety_check.stuck.rear_vehicle_reaction_time"));
  p.rss_params_for_stuck.rear_vehicle_safety_time_margin = getOrDeclareParameter<double>(
    *node, parameter("safety_check.stuck.rear_vehicle_safety_time_margin"));
  p.rss_params_for_stuck.lateral_distance_max_threshold = getOrDeclareParameter<double>(
    *node, parameter("safety_check.stuck.lateral_distance_max_threshold"));

  // target object
  {
    std::string ns = "lane_change.target_object.";
    p.object_types_to_check.check_car = getOrDeclareParameter<bool>(*node, ns + "car");
    p.object_types_to_check.check_truck = getOrDeclareParameter<bool>(*node, ns + "truck");
    p.object_types_to_check.check_bus = getOrDeclareParameter<bool>(*node, ns + "bus");
    p.object_types_to_check.check_trailer = getOrDeclareParameter<bool>(*node, ns + "trailer");
    p.object_types_to_check.check_unknown = getOrDeclareParameter<bool>(*node, ns + "unknown");
    p.object_types_to_check.check_bicycle = getOrDeclareParameter<bool>(*node, ns + "bicycle");
    p.object_types_to_check.check_motorcycle =
      getOrDeclareParameter<bool>(*node, ns + "motorcycle");
    p.object_types_to_check.check_pedestrian =
      getOrDeclareParameter<bool>(*node, ns + "pedestrian");
  }

  // lane change cancel
  p.cancel.enable_on_prepare_phase =
    getOrDeclareParameter<bool>(*node, parameter("cancel.enable_on_prepare_phase"));
  p.cancel.enable_on_lane_changing_phase =
    getOrDeclareParameter<bool>(*node, parameter("cancel.enable_on_lane_changing_phase"));
  p.cancel.delta_time = getOrDeclareParameter<double>(*node, parameter("cancel.delta_time"));
  p.cancel.duration = getOrDeclareParameter<double>(*node, parameter("cancel.duration"));
  p.cancel.max_lateral_jerk =
    getOrDeclareParameter<double>(*node, parameter("cancel.max_lateral_jerk"));
  p.cancel.overhang_tolerance =
    getOrDeclareParameter<double>(*node, parameter("cancel.overhang_tolerance"));

  p.finish_judge_lateral_threshold =
    getOrDeclareParameter<double>(*node, parameter("finish_judge_lateral_threshold"));

  // debug marker
  p.publish_debug_marker = getOrDeclareParameter<bool>(*node, parameter("publish_debug_marker"));

  // validation of parameters
  if (p.longitudinal_acc_sampling_num < 1 || p.lateral_acc_sampling_num < 1) {
    RCLCPP_FATAL_STREAM(
      logger_, "lane_change_sampling_num must be positive integer. Given longitudinal parameter: "
                 << p.longitudinal_acc_sampling_num
                 << "Given lateral parameter: " << p.lateral_acc_sampling_num << std::endl
                 << "Terminating the program...");
    exit(EXIT_FAILURE);
  }

  // validation of safety check parameters
  // if loosely check is not allowed, lane change module will keep on chattering and canceling, and
  // false positive situation might  occur
  if (!p.allow_loose_check_for_cancel) {
    if (
      p.rss_params.front_vehicle_deceleration > p.rss_params_for_abort.front_vehicle_deceleration ||
      p.rss_params.rear_vehicle_deceleration > p.rss_params_for_abort.rear_vehicle_deceleration ||
      p.rss_params.rear_vehicle_reaction_time > p.rss_params_for_abort.rear_vehicle_reaction_time ||
      p.rss_params.rear_vehicle_safety_time_margin >
        p.rss_params_for_abort.rear_vehicle_safety_time_margin ||
      p.rss_params.lateral_distance_max_threshold >
        p.rss_params_for_abort.lateral_distance_max_threshold ||
      p.rss_params.longitudinal_distance_min_threshold >
        p.rss_params_for_abort.longitudinal_distance_min_threshold ||
      p.rss_params.longitudinal_velocity_delta_time >
        p.rss_params_for_abort.longitudinal_velocity_delta_time) {
      RCLCPP_FATAL_STREAM(logger_, "abort parameter might be loose... Terminating the program...");
      exit(EXIT_FAILURE);
    }
  }
  if (p.cancel.delta_time < 1.0) {
    RCLCPP_WARN_STREAM(
      logger_, "cancel.delta_time: " << p.cancel.delta_time
                                     << ", is too short. This could cause a danger behavior.");
  }

  parameters_ = std::make_shared<LaneChangeParameters>(p);
}

std::unique_ptr<SceneModuleInterface> LaneChangeModuleManager::createNewSceneModuleInstance()
{
  if (type_ == LaneChangeModuleType::NORMAL) {
    return std::make_unique<LaneChangeInterface>(
      name_, *node_, parameters_, rtc_interface_ptr_map_,
      std::make_unique<NormalLaneChange>(parameters_, LaneChangeModuleType::NORMAL, direction_));
  }
  return std::make_unique<LaneChangeInterface>(
    name_, *node_, parameters_, rtc_interface_ptr_map_,
    std::make_unique<ExternalRequestLaneChange>(parameters_, direction_));
}

void LaneChangeModuleManager::updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto p = parameters_;

  const std::string ns = name_ + ".";
  updateParam<double>(
    parameters, ns + "finish_judge_lateral_threshold", p->finish_judge_lateral_threshold);

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

AvoidanceByLaneChangeModuleManager::AvoidanceByLaneChangeModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config)
: LaneChangeModuleManager(
    node, name, config, Direction::NONE, LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using tier4_autoware_utils::getOrDeclareParameter;

  rtc_interface_ptr_map_.clear();
  const std::vector<std::string> rtc_types = {"left", "right"};
  for (const auto & rtc_type : rtc_types) {
    const auto snake_case_name = convertToSnakeCase(name);
    const std::string rtc_interface_name = snake_case_name + "_" + rtc_type;
    rtc_interface_ptr_map_.emplace(
      rtc_type, std::make_shared<RTCInterface>(node, rtc_interface_name));
  }

  AvoidanceByLCParameters p{};
  // unique parameters
  {
    std::string ns = "avoidance_by_lane_change.";
    p.execute_object_longitudinal_margin =
      getOrDeclareParameter<double>(*node, ns + "execute_object_longitudinal_margin");
    p.execute_only_when_lane_change_finish_before_object =
      getOrDeclareParameter<bool>(*node, ns + "execute_only_when_lane_change_finish_before_object");
  }

  // general params
  {
    std::string ns = "avoidance.";
    p.resample_interval_for_planning =
      getOrDeclareParameter<double>(*node, ns + "resample_interval_for_planning");
    p.resample_interval_for_output =
      getOrDeclareParameter<double>(*node, ns + "resample_interval_for_output");
    p.enable_force_avoidance_for_stopped_vehicle =
      getOrDeclareParameter<bool>(*node, ns + "enable_force_avoidance_for_stopped_vehicle");
  }

  // target object
  {
    const auto get_object_param = [&](std::string && ns) {
      ObjectParameter param{};
      param.is_target = getOrDeclareParameter<bool>(*node, ns + "is_target");
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
      return param;
    };

    const std::string ns = "avoidance_by_lane_change.target_object.";
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
    std::string ns = "avoidance.target_filtering.";
    p.threshold_time_force_avoidance_for_stopped_vehicle = getOrDeclareParameter<double>(
      *node, ns + "threshold_time_force_avoidance_for_stopped_vehicle");
    p.object_ignore_section_traffic_light_in_front_distance = getOrDeclareParameter<double>(
      *node, ns + "object_ignore_section_traffic_light_in_front_distance");
    p.object_ignore_section_crosswalk_in_front_distance = getOrDeclareParameter<double>(
      *node, ns + "object_ignore_section_crosswalk_in_front_distance");
    p.object_ignore_section_crosswalk_behind_distance =
      getOrDeclareParameter<double>(*node, ns + "object_ignore_section_crosswalk_behind_distance");
    p.object_check_goal_distance =
      getOrDeclareParameter<double>(*node, ns + "object_check_goal_distance");
    p.threshold_distance_object_is_on_center =
      getOrDeclareParameter<double>(*node, ns + "threshold_distance_object_is_on_center");
    p.object_check_shiftable_ratio =
      getOrDeclareParameter<double>(*node, ns + "object_check_shiftable_ratio");
    p.object_check_min_road_shoulder_width =
      getOrDeclareParameter<double>(*node, ns + "object_check_min_road_shoulder_width");
    p.object_last_seen_threshold =
      getOrDeclareParameter<double>(*node, ns + "object_last_seen_threshold");
  }

  {
    std::string ns = "avoidance.target_filtering.detection_area.";
    p.use_static_detection_area = getOrDeclareParameter<bool>(*node, ns + "static");
    p.object_check_min_forward_distance =
      getOrDeclareParameter<double>(*node, ns + "min_forward_distance");
    p.object_check_max_forward_distance =
      getOrDeclareParameter<double>(*node, ns + "max_forward_distance");
    p.object_check_backward_distance =
      getOrDeclareParameter<double>(*node, ns + "backward_distance");
  }

  // safety check
  {
    std::string ns = "avoidance.safety_check.";
    p.hysteresis_factor_expand_rate =
      getOrDeclareParameter<double>(*node, ns + "hysteresis_factor_expand_rate");
  }

  avoidance_parameters_ = std::make_shared<AvoidanceByLCParameters>(p);
}

std::unique_ptr<SceneModuleInterface>
AvoidanceByLaneChangeModuleManager::createNewSceneModuleInstance()
{
  return std::make_unique<AvoidanceByLaneChangeInterface>(
    name_, *node_, parameters_, avoidance_parameters_, rtc_interface_ptr_map_);
}

}  // namespace behavior_path_planner
