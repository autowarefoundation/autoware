// Copyright 2021 Tier IV, Inc.
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

#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <tier4_autoware_utils/ros/parameter.hpp>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>

namespace behavior_velocity_planner
{
using lanelet::autoware::VirtualTrafficLight;
using tier4_autoware_utils::getOrDeclareParameter;

VirtualTrafficLightModuleManager::VirtualTrafficLightModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());

  {
    auto & p = planner_param_;
    p.max_delay_sec = getOrDeclareParameter<double>(node, ns + ".max_delay_sec");
    p.near_line_distance = getOrDeclareParameter<double>(node, ns + ".near_line_distance");
    p.dead_line_margin = getOrDeclareParameter<double>(node, ns + ".dead_line_margin");
    p.hold_stop_margin_distance =
      getOrDeclareParameter<double>(node, ns + ".hold_stop_margin_distance");
    p.max_yaw_deviation_rad = tier4_autoware_utils::deg2rad(
      getOrDeclareParameter<double>(node, ns + ".max_yaw_deviation_deg"));
    p.check_timeout_after_stop_line =
      getOrDeclareParameter<bool>(node, ns + ".check_timeout_after_stop_line");
  }
}

void VirtualTrafficLightModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & m : planning_utils::getRegElemMapOnPath<VirtualTrafficLight>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    // Use lanelet_id to unregister module when the route is changed
    const auto lane_id = m.second.id();
    const auto module_id = lane_id;
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<VirtualTrafficLightModule>(
        module_id, lane_id, *m.first, m.second, planner_param_,
        logger_.get_child("virtual_traffic_light_module"), clock_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
VirtualTrafficLightModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto id_set = planning_utils::getLaneletIdSetOnPath<VirtualTrafficLight>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return id_set.count(scene_module->getModuleId()) == 0;
  };
}
}  // namespace behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_velocity_planner::VirtualTrafficLightModulePlugin,
  behavior_velocity_planner::PluginInterface)
