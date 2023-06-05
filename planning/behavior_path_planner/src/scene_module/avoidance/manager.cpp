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

AvoidanceModuleManager::AvoidanceModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config,
  const std::shared_ptr<AvoidanceParameters> & parameters)
: SceneModuleManagerInterface(node, name, config, {"left", "right"}), parameters_{parameters}
{
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
    updateParam<bool>(parameters, ns + "enable", config.enable);
    updateParam<double>(parameters, ns + "envelope_buffer_margin", config.envelope_buffer_margin);
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
  }

  {
    const std::string ns = "avoidance.avoidance.lateral.";
    updateParam<double>(
      parameters, ns + "lateral_execution_threshold", p->lateral_execution_threshold);
    updateParam<double>(
      parameters, ns + "lateral_small_shift_threshold", p->lateral_small_shift_threshold);
    updateParam<double>(parameters, ns + "lateral_collision_margin", p->lateral_collision_margin);
    updateParam<double>(
      parameters, ns + "road_shoulder_safety_margin", p->road_shoulder_safety_margin);
  }

  {
    const std::string ns = "avoidance.avoidance.longitudinal.";
    updateParam<double>(parameters, ns + "prepare_time", p->prepare_time);
  }

  {
    const std::string ns = "avoidance.stop.";
    updateParam<double>(parameters, ns + "max_distance", p->stop_max_distance);
    updateParam<double>(parameters, ns + "min_distance", p->stop_min_distance);
  }

  {
    const std::string ns = "avoidance.constrains.lateral.";
    updateParam<double>(parameters, ns + "nominal_lateral_jerk", p->nominal_lateral_jerk);
    updateParam<double>(parameters, ns + "max_lateral_jerk", p->max_lateral_jerk);
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
