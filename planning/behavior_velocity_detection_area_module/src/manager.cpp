// Copyright 2020 Tier IV, Inc.
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
using lanelet::autoware::DetectionArea;
using tier4_autoware_utils::getOrDeclareParameter;

DetectionAreaModuleManager::DetectionAreaModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(
    node, getModuleName(),
    getOrDeclareParameter<bool>(node, std::string(getModuleName()) + ".enable_rtc"))
{
  const std::string ns(getModuleName());
  planner_param_.stop_margin = getOrDeclareParameter<double>(node, ns + ".stop_margin");
  planner_param_.use_dead_line = getOrDeclareParameter<bool>(node, ns + ".use_dead_line");
  planner_param_.dead_line_margin = getOrDeclareParameter<double>(node, ns + ".dead_line_margin");
  planner_param_.use_pass_judge_line =
    getOrDeclareParameter<bool>(node, ns + ".use_pass_judge_line");
  planner_param_.state_clear_time = getOrDeclareParameter<double>(node, ns + ".state_clear_time");
  planner_param_.hold_stop_margin_distance =
    getOrDeclareParameter<double>(node, ns + ".hold_stop_margin_distance");
  planner_param_.distance_to_judge_over_stop_line =
    getOrDeclareParameter<double>(node, ns + ".distance_to_judge_over_stop_line");
}

void DetectionAreaModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & detection_area_with_lane_id :
       planning_utils::getRegElemMapOnPath<DetectionArea>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    // Use lanelet_id to unregister module when the route is changed
    const auto lane_id = detection_area_with_lane_id.second.id();
    const auto module_id = detection_area_with_lane_id.first->id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<DetectionAreaModule>(
        module_id, lane_id, *detection_area_with_lane_id.first, planner_param_,
        logger_.get_child("detection_area_module"), clock_));
      generateUUID(module_id);
      updateRTCStatus(
        getUUID(module_id), true, std::numeric_limits<double>::lowest(), path.header.stamp);
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
DetectionAreaModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto detection_area_id_set = planning_utils::getRegElemIdSetOnPath<DetectionArea>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [detection_area_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return detection_area_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_velocity_planner::DetectionAreaModulePlugin, behavior_velocity_planner::PluginInterface)
