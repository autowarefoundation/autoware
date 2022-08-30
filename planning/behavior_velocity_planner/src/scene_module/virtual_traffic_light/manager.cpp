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

#include <scene_module/virtual_traffic_light/manager.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>

namespace behavior_velocity_planner
{
using lanelet::autoware::VirtualTrafficLight;

VirtualTrafficLightModuleManager::VirtualTrafficLightModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());

  {
    auto & p = planner_param_;
    p.max_delay_sec = node.declare_parameter(ns + ".max_delay_sec", 3.0);
    p.near_line_distance = node.declare_parameter(ns + ".near_line_distance", 1.0);
    p.dead_line_margin = node.declare_parameter(ns + ".dead_line_margin", 1.0);
    p.hold_stop_margin_distance = node.declare_parameter(ns + ".hold_stop_margin_distance", 0.0);
    p.max_yaw_deviation_rad =
      tier4_autoware_utils::deg2rad(node.declare_parameter(ns + ".max_yaw_deviation_deg", 90.0));
    p.check_timeout_after_stop_line =
      node.declare_parameter(ns + ".check_timeout_after_stop_line", true);
  }
}

void VirtualTrafficLightModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & m : planning_utils::getRegElemMapOnPath<VirtualTrafficLight>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_pose.pose)) {
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
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_pose.pose);

  return [id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return id_set.count(scene_module->getModuleId()) == 0;
  };
}
}  // namespace behavior_velocity_planner
