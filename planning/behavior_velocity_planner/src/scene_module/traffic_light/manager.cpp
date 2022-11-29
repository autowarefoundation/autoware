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

#include <scene_module/traffic_light/manager.hpp>

#include <tf2/utils.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>

namespace behavior_velocity_planner
{
using lanelet::TrafficLight;

TrafficLightModuleManager::TrafficLightModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(node, getModuleName())
{
  const std::string ns(getModuleName());
  planner_param_.stop_margin = node.declare_parameter(ns + ".stop_margin", 0.0);
  planner_param_.tl_state_timeout = node.declare_parameter(ns + ".tl_state_timeout", 1.0);
  planner_param_.external_tl_state_timeout =
    node.declare_parameter(ns + ".external_tl_state_timeout", 1.0);
  planner_param_.enable_pass_judge = node.declare_parameter(ns + ".enable_pass_judge", true);
  planner_param_.yellow_lamp_period = node.declare_parameter(ns + ".yellow_lamp_period", 2.75);
  pub_tl_state_ = node.create_publisher<autoware_auto_perception_msgs::msg::LookingTrafficSignal>(
    "~/output/traffic_signal", 1);
}

void TrafficLightModuleManager::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path)
{
  visualization_msgs::msg::MarkerArray debug_marker_array;
  visualization_msgs::msg::MarkerArray virtual_wall_marker_array;

  autoware_auto_perception_msgs::msg::LookingTrafficSignal tl_state;
  tl_state.header.stamp = path->header.stamp;
  tl_state.is_module_running = false;

  tier4_planning_msgs::msg::StopReasonArray stop_reason_array;
  stop_reason_array.header.frame_id = "map";
  stop_reason_array.header.stamp = path->header.stamp;

  first_stop_path_point_index_ = static_cast<int>(path->points.size() - 1);
  first_ref_stop_path_point_index_ = static_cast<int>(path->points.size() - 1);
  for (const auto & scene_module : scene_modules_) {
    tier4_planning_msgs::msg::StopReason stop_reason;
    std::shared_ptr<TrafficLightModule> traffic_light_scene_module(
      std::dynamic_pointer_cast<TrafficLightModule>(scene_module));
    traffic_light_scene_module->setPlannerData(planner_data_);
    traffic_light_scene_module->modifyPathVelocity(path, &stop_reason);

    if (stop_reason.reason != "") {
      stop_reason_array.stop_reasons.emplace_back(stop_reason);
    }

    if (traffic_light_scene_module->getFirstStopPathPointIndex() < first_stop_path_point_index_) {
      first_stop_path_point_index_ = traffic_light_scene_module->getFirstStopPathPointIndex();
    }
    if (
      traffic_light_scene_module->getFirstRefStopPathPointIndex() <
      first_ref_stop_path_point_index_) {
      first_ref_stop_path_point_index_ =
        traffic_light_scene_module->getFirstRefStopPathPointIndex();
      if (
        traffic_light_scene_module->getTrafficLightModuleState() !=
        TrafficLightModule::State::GO_OUT) {
        tl_state = traffic_light_scene_module->getTrafficSignal();
      }
    }
    for (const auto & marker : traffic_light_scene_module->createDebugMarkerArray().markers) {
      debug_marker_array.markers.push_back(marker);
    }
    for (const auto & marker : traffic_light_scene_module->createVirtualWallMarkerArray().markers) {
      virtual_wall_marker_array.markers.push_back(marker);
    }
  }
  if (!stop_reason_array.stop_reasons.empty()) {
    pub_stop_reason_->publish(stop_reason_array);
  }
  pub_debug_->publish(debug_marker_array);
  pub_virtual_wall_->publish(virtual_wall_marker_array);
  pub_tl_state_->publish(tl_state);
}

void TrafficLightModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & traffic_light_reg_elem : planning_utils::getRegElemMapOnPath<TrafficLight>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_pose.pose)) {
    const auto stop_line = traffic_light_reg_elem.first->stopLine();

    if (!stop_line) {
      RCLCPP_FATAL(
        logger_, "No stop line at traffic_light_reg_elem_id = %ld, please fix the map!",
        traffic_light_reg_elem.first->id());
      continue;
    }

    // Use lanelet_id to unregister module when the route is changed
    const auto lane_id = traffic_light_reg_elem.second.id();
    const auto module_id = lane_id;
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<TrafficLightModule>(
        module_id, lane_id, *(traffic_light_reg_elem.first), traffic_light_reg_elem.second,
        planner_param_, logger_.get_child("traffic_light_module"), clock_));
      generateUUID(module_id);
      updateRTCStatus(
        getUUID(module_id), true, std::numeric_limits<double>::lowest(), path.header.stamp);
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
TrafficLightModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lanelet_id_set = planning_utils::getLaneletIdSetOnPath<TrafficLight>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_pose.pose);

  return [lanelet_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return lanelet_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace behavior_velocity_planner
