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

#include <lanelet2_extension/utility/query.hpp>
#include <scene_module/detection_area/manager.hpp>

#include <tf2/utils.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using lanelet::autoware::DetectionArea;

DetectionAreaModuleManager::DetectionAreaModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(node, getModuleName())
{
  const std::string ns(getModuleName());
  planner_param_.stop_margin = node.declare_parameter(ns + ".stop_margin", 0.0);
  planner_param_.use_dead_line = node.declare_parameter(ns + ".use_dead_line", false);
  planner_param_.dead_line_margin = node.declare_parameter(ns + ".dead_line_margin", 5.0);
  planner_param_.use_pass_judge_line = node.declare_parameter(ns + ".use_pass_judge_line", false);
  planner_param_.state_clear_time = node.declare_parameter(ns + ".state_clear_time", 2.0);
  planner_param_.hold_stop_margin_distance =
    node.declare_parameter(ns + ".hold_stop_margin_distance", 0.0);
}

void DetectionAreaModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & detection_area_with_lane_id :
       planning_utils::getRegElemMapOnPath<DetectionArea>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_pose.pose)) {
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
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_pose.pose);

  return [detection_area_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return detection_area_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace behavior_velocity_planner
