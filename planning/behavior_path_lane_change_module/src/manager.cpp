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

#include "behavior_path_lane_change_module/manager.hpp"

#include "behavior_path_lane_change_module/interface.hpp"
#include "tier4_autoware_utils/ros/parameter.hpp"
#include "tier4_autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

void LaneChangeModuleManager::init(rclcpp::Node * node)
{
  using tier4_autoware_utils::getOrDeclareParameter;

  // init manager interface
  initInterface(node, {""});

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
  p.regulate_on_traffic_light =
    getOrDeclareParameter<bool>(*node, parameter("regulation.traffic_light"));

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

  // lane change parameters
  const auto max_acc = getOrDeclareParameter<double>(*node, "normal.max_acc");
  p.backward_length_buffer_for_end_of_lane =
    getOrDeclareParameter<double>(*node, parameter("backward_length_buffer_for_end_of_lane"));
  p.backward_length_buffer_for_blocking_object =
    getOrDeclareParameter<double>(*node, parameter("backward_length_buffer_for_blocking_object"));
  p.lane_changing_lateral_jerk =
    getOrDeclareParameter<double>(*node, parameter("lane_changing_lateral_jerk"));
  p.lane_change_prepare_duration =
    getOrDeclareParameter<double>(*node, parameter("prepare_duration"));
  p.minimum_lane_changing_velocity =
    getOrDeclareParameter<double>(*node, parameter("minimum_lane_changing_velocity"));
  p.minimum_lane_changing_velocity =
    std::min(p.minimum_lane_changing_velocity, max_acc * p.lane_change_prepare_duration);
  p.lane_change_finish_judge_buffer =
    getOrDeclareParameter<double>(*node, parameter("lane_change_finish_judge_buffer"));

  if (p.backward_length_buffer_for_end_of_lane < 1.0) {
    RCLCPP_WARN_STREAM(
      node->get_logger().get_child(name()),
      "Lane change buffer must be more than 1 meter. Modifying the buffer.");
  }

  // lateral acceleration map for lane change
  const auto lateral_acc_velocity =
    getOrDeclareParameter<std::vector<double>>(*node, parameter("lateral_acceleration.velocity"));
  const auto min_lateral_acc =
    getOrDeclareParameter<std::vector<double>>(*node, parameter("lateral_acceleration.min_values"));
  const auto max_lateral_acc =
    getOrDeclareParameter<std::vector<double>>(*node, parameter("lateral_acceleration.max_values"));
  if (
    lateral_acc_velocity.size() != min_lateral_acc.size() ||
    lateral_acc_velocity.size() != max_lateral_acc.size()) {
    RCLCPP_ERROR(
      node->get_logger().get_child(name()),
      "Lane change lateral acceleration map has invalid size.");
    exit(EXIT_FAILURE);
  }
  for (size_t i = 0; i < lateral_acc_velocity.size(); ++i) {
    p.lane_change_lat_acc_map.add(
      lateral_acc_velocity.at(i), min_lateral_acc.at(i), max_lateral_acc.at(i));
  }

  // target object
  {
    const std::string ns = "lane_change.target_object.";
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
      node->get_logger().get_child(name()),
      "lane_change_sampling_num must be positive integer. Given longitudinal parameter: "
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
      RCLCPP_FATAL_STREAM(
        node->get_logger().get_child(name()),
        "abort parameter might be loose... Terminating the program...");
      exit(EXIT_FAILURE);
    }
  }
  if (p.cancel.delta_time < 1.0) {
    RCLCPP_WARN_STREAM(
      node->get_logger().get_child(name()),
      "cancel.delta_time: " << p.cancel.delta_time
                            << ", is too short. This could cause a danger behavior.");
  }

  parameters_ = std::make_shared<LaneChangeParameters>(p);
}

std::unique_ptr<SceneModuleInterface> LaneChangeModuleManager::createNewSceneModuleInstance()
{
  return std::make_unique<LaneChangeInterface>(
    name_, *node_, parameters_, rtc_interface_ptr_map_,
    objects_of_interest_marker_interface_ptr_map_,
    std::make_unique<NormalLaneChange>(parameters_, LaneChangeModuleType::NORMAL, direction_));
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

}  // namespace behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_path_planner::LaneChangeRightModuleManager,
  behavior_path_planner::SceneModuleManagerInterface)
PLUGINLIB_EXPORT_CLASS(
  behavior_path_planner::LaneChangeLeftModuleManager,
  behavior_path_planner::SceneModuleManagerInterface)
