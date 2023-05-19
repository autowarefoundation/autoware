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

#include "behavior_path_planner/scene_module/dynamic_avoidance/manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

DynamicAvoidanceModuleManager::DynamicAvoidanceModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config,
  const std::shared_ptr<DynamicAvoidanceParameters> & parameters)
: SceneModuleManagerInterface(node, name, config, {""}), parameters_{parameters}
{
}

void DynamicAvoidanceModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;
  auto & p = parameters_;

  {  // target object
    const std::string ns = "dynamic_avoidance.target_object.";

    updateParam<bool>(parameters, ns + "car", p->avoid_car);
    updateParam<bool>(parameters, ns + "truck", p->avoid_truck);
    updateParam<bool>(parameters, ns + "bus", p->avoid_bus);
    updateParam<bool>(parameters, ns + "trailer", p->avoid_trailer);
    updateParam<bool>(parameters, ns + "unknown", p->avoid_unknown);
    updateParam<bool>(parameters, ns + "bicycle", p->avoid_bicycle);
    updateParam<bool>(parameters, ns + "motorcycle", p->avoid_motorcycle);
    updateParam<bool>(parameters, ns + "pedestrian", p->avoid_pedestrian);

    updateParam<double>(parameters, ns + "min_obstacle_vel", p->min_obstacle_vel);
  }

  {  // drivable_area_generation
    const std::string ns = "dynamic_avoidance.drivable_area_generation.";

    updateParam<double>(parameters, ns + "lat_offset_from_obstacle", p->lat_offset_from_obstacle);
    updateParam<double>(parameters, ns + "max_lat_offset_to_avoid", p->max_lat_offset_to_avoid);

    updateParam<double>(
      parameters, ns + "overtaking_object.max_time_to_collision",
      p->max_time_to_collision_overtaking_object);
    updateParam<double>(
      parameters, ns + "overtaking_object.start_duration_to_avoid",
      p->start_duration_to_avoid_overtaking_object);
    updateParam<double>(
      parameters, ns + "overtaking_object.end_duration_to_avoid",
      p->end_duration_to_avoid_overtaking_object);
    updateParam<double>(
      parameters, ns + "overtaking_object.duration_to_hold_avoidance",
      p->duration_to_hold_avoidance_overtaking_object);

    updateParam<double>(
      parameters, ns + "oncoming_object.max_time_to_collision",
      p->max_time_to_collision_oncoming_object);
    updateParam<double>(
      parameters, ns + "oncoming_object.start_duration_to_avoid",
      p->start_duration_to_avoid_oncoming_object);
    updateParam<double>(
      parameters, ns + "oncoming_object.end_duration_to_avoid",
      p->end_duration_to_avoid_oncoming_object);
  }

  std::for_each(registered_modules_.begin(), registered_modules_.end(), [&p](const auto & m) {
    m->updateModuleParams(p);
  });
}
}  // namespace behavior_path_planner
