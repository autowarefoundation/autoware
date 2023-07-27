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
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config)
: SceneModuleManagerInterface(node, name, config, {""})
{
  DynamicAvoidanceParameters p{};

  {  // common
    std::string ns = "dynamic_avoidance.common.";
    p.enable_debug_info = node->declare_parameter<bool>(ns + "enable_debug_info");
  }

  {  // target object
    std::string ns = "dynamic_avoidance.target_object.";
    p.avoid_car = node->declare_parameter<bool>(ns + "car");
    p.avoid_truck = node->declare_parameter<bool>(ns + "truck");
    p.avoid_bus = node->declare_parameter<bool>(ns + "bus");
    p.avoid_trailer = node->declare_parameter<bool>(ns + "trailer");
    p.avoid_unknown = node->declare_parameter<bool>(ns + "unknown");
    p.avoid_bicycle = node->declare_parameter<bool>(ns + "bicycle");
    p.avoid_motorcycle = node->declare_parameter<bool>(ns + "motorcycle");
    p.avoid_pedestrian = node->declare_parameter<bool>(ns + "pedestrian");
    p.min_obstacle_vel = node->declare_parameter<double>(ns + "min_obstacle_vel");
    p.successive_num_to_entry_dynamic_avoidance_condition =
      node->declare_parameter<int>(ns + "successive_num_to_entry_dynamic_avoidance_condition");

    p.min_obj_lat_offset_to_ego_path =
      node->declare_parameter<double>(ns + "min_obj_lat_offset_to_ego_path");
    p.max_obj_lat_offset_to_ego_path =
      node->declare_parameter<double>(ns + "max_obj_lat_offset_to_ego_path");

    p.max_front_object_angle =
      node->declare_parameter<double>(ns + "front_object.max_object_angle");

    p.min_crossing_object_vel =
      node->declare_parameter<double>(ns + "crossing_object.min_object_vel");
    p.max_crossing_object_angle =
      node->declare_parameter<double>(ns + "crossing_object.max_object_angle");
  }

  {  // drivable_area_generation
    std::string ns = "dynamic_avoidance.drivable_area_generation.";
    p.lat_offset_from_obstacle = node->declare_parameter<double>(ns + "lat_offset_from_obstacle");
    p.max_lat_offset_to_avoid = node->declare_parameter<double>(ns + "max_lat_offset_to_avoid");

    p.max_time_to_collision_overtaking_object =
      node->declare_parameter<double>(ns + "overtaking_object.max_time_to_collision");
    p.start_duration_to_avoid_overtaking_object =
      node->declare_parameter<double>(ns + "overtaking_object.start_duration_to_avoid");
    p.end_duration_to_avoid_overtaking_object =
      node->declare_parameter<double>(ns + "overtaking_object.end_duration_to_avoid");
    p.duration_to_hold_avoidance_overtaking_object =
      node->declare_parameter<double>(ns + "overtaking_object.duration_to_hold_avoidance");

    p.max_time_to_collision_oncoming_object =
      node->declare_parameter<double>(ns + "oncoming_object.max_time_to_collision");
    p.start_duration_to_avoid_oncoming_object =
      node->declare_parameter<double>(ns + "oncoming_object.start_duration_to_avoid");
    p.end_duration_to_avoid_oncoming_object =
      node->declare_parameter<double>(ns + "oncoming_object.end_duration_to_avoid");
  }

  parameters_ = std::make_shared<DynamicAvoidanceParameters>(p);
}

void DynamicAvoidanceModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;
  auto & p = parameters_;

  {  // common
    const std::string ns = "dynamic_avoidance.common.";
    updateParam<bool>(parameters, ns + "enable_debug_info", p->enable_debug_info);
  }

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

    updateParam<int>(
      parameters, ns + "successive_num_to_entry_dynamic_avoidance_condition",
      p->successive_num_to_entry_dynamic_avoidance_condition);

    updateParam<double>(
      parameters, ns + "min_obj_lat_offset_to_ego_path", p->min_obj_lat_offset_to_ego_path);
    updateParam<double>(
      parameters, ns + "max_obj_lat_offset_to_ego_path", p->max_obj_lat_offset_to_ego_path);

    updateParam<double>(
      parameters, ns + "front_object.max_object_angle", p->max_front_object_angle);

    updateParam<double>(
      parameters, ns + "crossing_object.min_object_vel", p->min_crossing_object_vel);
    updateParam<double>(
      parameters, ns + "crossing_object.max_object_angle", p->max_crossing_object_angle);
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
