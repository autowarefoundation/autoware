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
#include <lanelet2_extension/utility/query.hpp>
#include <tier4_autoware_utils/ros/parameter.hpp>

#include <tf2/utils.h>

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using lanelet::autoware::NoStoppingArea;
using tier4_autoware_utils::getOrDeclareParameter;

NoStoppingAreaModuleManager::NoStoppingAreaModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(
    node, getModuleName(),
    getOrDeclareParameter<bool>(node, std::string(getModuleName()) + ".enable_rtc"))
{
  const std::string ns(getModuleName());
  auto & pp = planner_param_;
  const auto & vi = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  pp.state_clear_time = getOrDeclareParameter<double>(node, ns + ".state_clear_time");
  pp.stuck_vehicle_vel_thr = getOrDeclareParameter<double>(node, ns + ".stuck_vehicle_vel_thr");
  pp.stop_margin = getOrDeclareParameter<double>(node, ns + ".stop_margin");
  pp.dead_line_margin = getOrDeclareParameter<double>(node, ns + ".dead_line_margin");
  pp.stop_line_margin = getOrDeclareParameter<double>(node, ns + ".stop_line_margin");
  pp.detection_area_length = getOrDeclareParameter<double>(node, ns + ".detection_area_length");
  pp.stuck_vehicle_front_margin =
    getOrDeclareParameter<double>(node, ns + ".stuck_vehicle_front_margin");
  pp.path_expand_width = vi.vehicle_width_m * 0.5;
}

void NoStoppingAreaModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & m : planning_utils::getRegElemMapOnPath<NoStoppingArea>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    // Use lanelet_id to unregister module when the route is changed
    const int64_t module_id = m.first->id();
    const int64_t lane_id = m.second.id();

    if (!isModuleRegistered(module_id)) {
      // assign 1 no stopping area for each module
      registerModule(std::make_shared<NoStoppingAreaModule>(
        module_id, lane_id, *m.first, planner_param_, logger_.get_child("no_stopping_area_module"),
        clock_));
      generateUUID(module_id);
      updateRTCStatus(
        getUUID(module_id), true, std::numeric_limits<double>::lowest(), path.header.stamp);
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
NoStoppingAreaModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto no_stopping_area_id_set = planning_utils::getRegElemIdSetOnPath<NoStoppingArea>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [no_stopping_area_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return no_stopping_area_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_velocity_planner::NoStoppingAreaModulePlugin, behavior_velocity_planner::PluginInterface)
