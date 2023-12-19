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

#include "behavior_path_avoidance_by_lane_change_module/manager.hpp"

#include "behavior_path_avoidance_by_lane_change_module/data_structs.hpp"
#include "behavior_path_avoidance_module/parameter_helper.hpp"
#include "tier4_autoware_utils/ros/parameter.hpp"
#include "tier4_autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

void AvoidanceByLaneChangeModuleManager::init(rclcpp::Node * node)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using tier4_autoware_utils::getOrDeclareParameter;

  // init manager interface
  initInterface(node, {"left", "right"});

  // init lane change manager
  LaneChangeModuleManager::init(node);

  const auto avoidance_params = getParameter(node);
  AvoidanceByLCParameters p(avoidance_params);

  // unique parameters
  {
    const std::string ns = "avoidance_by_lane_change.";
    p.execute_object_longitudinal_margin =
      getOrDeclareParameter<double>(*node, ns + "execute_object_longitudinal_margin");
    p.execute_only_when_lane_change_finish_before_object =
      getOrDeclareParameter<bool>(*node, ns + "execute_only_when_lane_change_finish_before_object");
  }

  // general params
  {
    const std::string ns = "avoidance.";
    p.resample_interval_for_planning =
      getOrDeclareParameter<double>(*node, ns + "resample_interval_for_planning");
    p.resample_interval_for_output =
      getOrDeclareParameter<double>(*node, ns + "resample_interval_for_output");
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
    p.object_last_seen_threshold =
      getOrDeclareParameter<double>(*node, ns + "object_last_seen_threshold");
  }

  {
    const std::string ns = "avoidance.target_filtering.force_avoidance.";
    p.enable_force_avoidance_for_stopped_vehicle =
      getOrDeclareParameter<bool>(*node, ns + "enable");
    p.threshold_time_force_avoidance_for_stopped_vehicle =
      getOrDeclareParameter<double>(*node, ns + "time_threshold");
    p.object_ignore_section_traffic_light_in_front_distance =
      getOrDeclareParameter<double>(*node, ns + "ignore_area.traffic_light.front_distance");
    p.object_ignore_section_crosswalk_in_front_distance =
      getOrDeclareParameter<double>(*node, ns + "ignore_area.crosswalk.front_distance");
    p.object_ignore_section_crosswalk_behind_distance =
      getOrDeclareParameter<double>(*node, ns + "ignore_area.crosswalk.behind_distance");
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

  // safety check
  {
    const std::string ns = "avoidance.safety_check.";
    p.hysteresis_factor_expand_rate =
      getOrDeclareParameter<double>(*node, ns + "hysteresis_factor_expand_rate");
  }

  avoidance_parameters_ = std::make_shared<AvoidanceByLCParameters>(p);
}

std::unique_ptr<SceneModuleInterface>
AvoidanceByLaneChangeModuleManager::createNewSceneModuleInstance()
{
  return std::make_unique<AvoidanceByLaneChangeInterface>(
    name_, *node_, parameters_, avoidance_parameters_, rtc_interface_ptr_map_,
    objects_of_interest_marker_interface_ptr_map_);
}

}  // namespace behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_path_planner::AvoidanceByLaneChangeModuleManager,
  behavior_path_planner::SceneModuleManagerInterface)
