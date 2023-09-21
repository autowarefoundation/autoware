// Copyright 2020 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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
#include <lanelet2_extension/regulatory_elements/speed_bump.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <tier4_autoware_utils/ros/parameter.hpp>

#include <tf2/utils.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using lanelet::autoware::SpeedBump;
using tier4_autoware_utils::getOrDeclareParameter;

SpeedBumpModuleManager::SpeedBumpModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  std::string ns(getModuleName());
  planner_param_.slow_start_margin = getOrDeclareParameter<double>(node, ns + ".slow_start_margin");
  planner_param_.slow_end_margin = getOrDeclareParameter<double>(node, ns + ".slow_end_margin");
  planner_param_.print_debug_info = getOrDeclareParameter<bool>(node, ns + ".print_debug_info");

  // limits for speed bump height and slow down speed
  ns += ".speed_calculation";
  planner_param_.speed_calculation_min_height =
    static_cast<float>(getOrDeclareParameter<double>(node, ns + ".min_height"));
  planner_param_.speed_calculation_max_height =
    static_cast<float>(getOrDeclareParameter<double>(node, ns + ".max_height"));
  planner_param_.speed_calculation_min_speed =
    static_cast<float>(getOrDeclareParameter<double>(node, ns + ".min_speed"));
  planner_param_.speed_calculation_max_speed =
    static_cast<float>(getOrDeclareParameter<double>(node, ns + ".max_speed"));
}

void SpeedBumpModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & speed_bump_with_lane_id : planning_utils::getRegElemMapOnPath<SpeedBump>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    const auto lane_id = speed_bump_with_lane_id.second.id();
    const auto module_id = speed_bump_with_lane_id.first->id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<SpeedBumpModule>(
        module_id, lane_id, *speed_bump_with_lane_id.first, planner_param_,
        logger_.get_child("speed_bump_module"), clock_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
SpeedBumpModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto speed_bump_id_set = planning_utils::getRegElemIdSetOnPath<SpeedBump>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [speed_bump_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return speed_bump_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_velocity_planner::SpeedBumpModulePlugin, behavior_velocity_planner::PluginInterface)
