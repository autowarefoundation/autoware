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

#include "scene_module/no_stopping_area/manager.hpp"

#include <lanelet2_extension/utility/query.hpp>

#include <tf2/utils.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using lanelet::autoware::NoStoppingArea;

NoStoppingAreaModuleManager::NoStoppingAreaModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(node, getModuleName())
{
  const std::string ns(getModuleName());
  auto & pp = planner_param_;
  const auto & vi = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  pp.state_clear_time = node.declare_parameter(ns + ".state_clear_time", 2.0);
  pp.stuck_vehicle_vel_thr = node.declare_parameter(ns + ".stuck_vehicle_vel_thr", 0.8333);
  pp.stop_margin = node.declare_parameter(ns + ".stop_margin", 0.0);
  pp.dead_line_margin = node.declare_parameter(ns + ".dead_line_margin", 1.0);
  pp.stop_line_margin = node.declare_parameter(ns + ".stop_line_margin", 1.0);
  pp.detection_area_length = node.declare_parameter(ns + ".detection_area_length", 200.0);
  pp.stuck_vehicle_front_margin = node.declare_parameter(ns + ".stuck_vehicle_front_margin", 6.0);
  pp.path_expand_width = vi.vehicle_width_m * 0.5;
}

void NoStoppingAreaModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & m : planning_utils::getRegElemMapOnPath<NoStoppingArea>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_pose.pose)) {
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
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_pose.pose);

  return [no_stopping_area_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return no_stopping_area_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace behavior_velocity_planner
