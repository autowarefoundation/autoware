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

#include "behavior_path_avoidance_module/manager.hpp"

#include "behavior_path_avoidance_module/parameter_helper.hpp"
#include "tier4_autoware_utils/ros/parameter.hpp"
#include "tier4_autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{
void AvoidanceModuleManager::init(rclcpp::Node * node)
{
  using autoware_auto_perception_msgs::msg::ObjectClassification;
  using tier4_autoware_utils::getOrDeclareParameter;

  // init manager interface
  initInterface(node, {"left", "right"});

  auto p = getParameter(node);

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
    updateParam<double>(parameters, ns + "moving_speed_threshold", config.moving_speed_threshold);
    updateParam<double>(parameters, ns + "moving_time_threshold", config.moving_time_threshold);
    updateParam<double>(parameters, ns + "max_expand_ratio", config.max_expand_ratio);
    updateParam<double>(parameters, ns + "envelope_buffer_margin", config.envelope_buffer_margin);
    updateParam<double>(parameters, ns + "avoid_margin_lateral", config.avoid_margin_lateral);
    updateParam<double>(parameters, ns + "safety_buffer_lateral", config.safety_buffer_lateral);
    updateParam<double>(
      parameters, ns + "safety_buffer_longitudinal", config.safety_buffer_longitudinal);
    updateParam<bool>(
      parameters, ns + "use_conservative_buffer_longitudinal",
      config.use_conservative_buffer_longitudinal);
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
      parameters, ns + "lateral_avoid_check_threshold", p->lateral_avoid_check_threshold);
    updateParam<double>(parameters, ns + "soft_road_shoulder_margin", p->soft_road_shoulder_margin);
    updateParam<double>(parameters, ns + "hard_road_shoulder_margin", p->hard_road_shoulder_margin);
  }

  {
    const std::string ns = "avoidance.avoidance.longitudinal.";
    updateParam<double>(parameters, ns + "min_prepare_time", p->min_prepare_time);
    updateParam<double>(parameters, ns + "max_prepare_time", p->max_prepare_time);
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

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}
}  // namespace behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_path_planner::AvoidanceModuleManager, behavior_path_planner::SceneModuleManagerInterface)
