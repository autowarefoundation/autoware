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

#include "manager.hpp"

#include "scene_dynamic_obstacle_stop.hpp"

#include <tier4_autoware_utils/ros/parameter.hpp>

#include <algorithm>

namespace behavior_velocity_planner
{
using tier4_autoware_utils::getOrDeclareParameter;

DynamicObstacleStopModuleManager::DynamicObstacleStopModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName()), module_id_(0UL)
{
  const std::string ns(getModuleName());
  auto & pp = planner_param_;

  pp.extra_object_width = getOrDeclareParameter<double>(node, ns + ".extra_object_width");
  pp.minimum_object_velocity = getOrDeclareParameter<double>(node, ns + ".minimum_object_velocity");
  pp.stop_distance_buffer = getOrDeclareParameter<double>(node, ns + ".stop_distance_buffer");
  pp.time_horizon = getOrDeclareParameter<double>(node, ns + ".time_horizon");
  pp.hysteresis = getOrDeclareParameter<double>(node, ns + ".hysteresis");
  pp.decision_duration_buffer =
    getOrDeclareParameter<double>(node, ns + ".decision_duration_buffer");
  pp.minimum_object_distance_from_ego_path =
    getOrDeclareParameter<double>(node, ns + ".minimum_object_distance_from_ego_path");
  pp.ignore_unavoidable_collisions =
    getOrDeclareParameter<bool>(node, ns + ".ignore_unavoidable_collisions");

  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  pp.ego_lateral_offset =
    std::max(std::abs(vehicle_info.min_lateral_offset_m), vehicle_info.max_lateral_offset_m);
  pp.ego_longitudinal_offset = vehicle_info.max_longitudinal_offset_m;
}

void DynamicObstacleStopModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  if (path.points.empty()) return;
  // general
  if (!isModuleRegistered(module_id_)) {
    registerModule(std::make_shared<dynamic_obstacle_stop::DynamicObstacleStopModule>(
      module_id_, planner_param_, logger_.get_child("dynamic_obstacle_stop_module"), clock_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
DynamicObstacleStopModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  return [path]([[maybe_unused]] const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return false;
  };
}
}  // namespace behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_velocity_planner::DynamicObstacleStopModulePlugin,
  behavior_velocity_planner::PluginInterface)
