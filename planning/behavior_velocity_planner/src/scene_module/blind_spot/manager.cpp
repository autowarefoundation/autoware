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

#include <scene_module/blind_spot/manager.hpp>
#include <utilization/boost_geometry_helper.hpp>
#include <utilization/util.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
BlindSpotModuleManager::BlindSpotModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(node, getModuleName())
{
  const std::string ns(getModuleName());
  planner_param_.use_pass_judge_line = node.declare_parameter(ns + ".use_pass_judge_line", false);
  planner_param_.stop_line_margin = node.declare_parameter(ns + ".stop_line_margin", 1.0);
  planner_param_.backward_length = node.declare_parameter(ns + ".backward_length", 15.0);
  planner_param_.ignore_width_from_center_line =
    node.declare_parameter(ns + ".ignore_width_from_center_line", 1.0);
  planner_param_.max_future_movement_time =
    node.declare_parameter(ns + ".max_future_movement_time", 10.0);
}

void BlindSpotModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & ll : planning_utils::getLaneletsOnPath(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_pose.pose)) {
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    if (isModuleRegistered(module_id)) {
      continue;
    }

    // Is turning lane?
    const std::string turn_direction = ll.attributeOr("turn_direction", "else");
    if (turn_direction != "left" && turn_direction != "right") {
      continue;
    }

    registerModule(std::make_shared<BlindSpotModule>(
      module_id, lane_id, planner_data_, planner_param_, logger_.get_child("blind_spot_module"),
      clock_));
    generateUUID(module_id);
    updateRTCStatus(
      getUUID(module_id), true, std::numeric_limits<double>::lowest(), path.header.stamp);
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
BlindSpotModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lane_id_set = planning_utils::getLaneIdSetOnPath(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_pose.pose);

  return [lane_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return lane_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace behavior_velocity_planner
