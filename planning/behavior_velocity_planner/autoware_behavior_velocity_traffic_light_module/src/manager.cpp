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

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>

#include <tf2/utils.h>

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
namespace autoware::behavior_velocity_planner
{
using autoware::universe_utils::getOrDeclareParameter;
using lanelet::TrafficLight;

TrafficLightModuleManager::TrafficLightModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(
    node, getModuleName(), getEnableRTC(node, std::string(getModuleName()) + ".enable_rtc"))
{
  const std::string ns(TrafficLightModuleManager::getModuleName());
  planner_param_.stop_margin = getOrDeclareParameter<double>(node, ns + ".stop_margin");
  planner_param_.tl_state_timeout = getOrDeclareParameter<double>(node, ns + ".tl_state_timeout");
  planner_param_.stop_time_hysteresis =
    getOrDeclareParameter<double>(node, ns + ".stop_time_hysteresis");
  planner_param_.enable_pass_judge = getOrDeclareParameter<bool>(node, ns + ".enable_pass_judge");
  planner_param_.yellow_lamp_period =
    getOrDeclareParameter<double>(node, ns + ".yellow_lamp_period");
  pub_tl_state_ = node.create_publisher<autoware_perception_msgs::msg::TrafficLightGroup>(
    "~/output/traffic_signal", 1);
}

void TrafficLightModuleManager::modifyPathVelocity(tier4_planning_msgs::msg::PathWithLaneId * path)
{
  visualization_msgs::msg::MarkerArray debug_marker_array;
  visualization_msgs::msg::MarkerArray virtual_wall_marker_array;

  autoware_perception_msgs::msg::TrafficLightGroup tl_state;

  autoware_adapi_v1_msgs::msg::VelocityFactorArray velocity_factor_array;
  velocity_factor_array.header.frame_id = "map";
  velocity_factor_array.header.stamp = clock_->now();

  tier4_planning_msgs::msg::StopReasonArray stop_reason_array;
  stop_reason_array.header.frame_id = "map";
  stop_reason_array.header.stamp = path->header.stamp;

  first_stop_path_point_index_ = static_cast<int>(path->points.size() - 1);
  nearest_ref_stop_path_point_index_ = static_cast<int>(path->points.size() - 1);
  for (const auto & scene_module : scene_modules_) {
    tier4_planning_msgs::msg::StopReason stop_reason;
    std::shared_ptr<TrafficLightModule> traffic_light_scene_module(
      std::dynamic_pointer_cast<TrafficLightModule>(scene_module));
    traffic_light_scene_module->resetVelocityFactor();
    traffic_light_scene_module->setPlannerData(planner_data_);
    traffic_light_scene_module->modifyPathVelocity(path, &stop_reason);

    // The velocity factor must be called after modifyPathVelocity.
    const auto velocity_factor = traffic_light_scene_module->getVelocityFactor();
    if (velocity_factor.behavior != PlanningBehavior::UNKNOWN) {
      velocity_factor_array.factors.emplace_back(velocity_factor);
    }
    if (stop_reason.reason != "") {
      stop_reason_array.stop_reasons.emplace_back(stop_reason);
    }

    if (traffic_light_scene_module->getFirstStopPathPointIndex() < first_stop_path_point_index_) {
      first_stop_path_point_index_ = traffic_light_scene_module->getFirstStopPathPointIndex();
    }
    if (
      traffic_light_scene_module->getFirstRefStopPathPointIndex() <
      nearest_ref_stop_path_point_index_) {
      nearest_ref_stop_path_point_index_ =
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
    virtual_wall_marker_creator_.add_virtual_walls(
      traffic_light_scene_module->createVirtualWalls());
  }
  if (!stop_reason_array.stop_reasons.empty()) {
    pub_stop_reason_->publish(stop_reason_array);
  }
  pub_velocity_factor_->publish(velocity_factor_array);
  pub_debug_->publish(debug_marker_array);
  pub_virtual_wall_->publish(virtual_wall_marker_creator_.create_markers(clock_->now()));
  pub_tl_state_->publish(tl_state);
}

void TrafficLightModuleManager::launchNewModules(
  const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & traffic_light_reg_elem : planning_utils::getRegElemMapOnPath<TrafficLight>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    const auto stop_line = traffic_light_reg_elem.first->stopLine();

    if (!stop_line) {
      RCLCPP_FATAL(
        logger_, "No stop line at traffic_light_reg_elem_id = %ld, please fix the map!",
        traffic_light_reg_elem.first->id());
      continue;
    }

    // Use lanelet_id to unregister module when the route is changed
    const auto lane_id = traffic_light_reg_elem.second.id();
    if (!isModuleRegisteredFromExistingAssociatedModule(lane_id)) {
      registerModule(std::make_shared<TrafficLightModule>(
        lane_id, *(traffic_light_reg_elem.first), traffic_light_reg_elem.second, planner_param_,
        logger_.get_child("traffic_light_module"), clock_));
      generateUUID(lane_id);
      updateRTCStatus(
        getUUID(lane_id), true, State::WAITING_FOR_EXECUTION, std::numeric_limits<double>::lowest(),
        path.header.stamp);
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
TrafficLightModuleManager::getModuleExpiredFunction(
  const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lanelet_id_set = planning_utils::getLaneletIdSetOnPath<TrafficLight>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [this, lanelet_id_set](
           [[maybe_unused]] const std::shared_ptr<SceneModuleInterface> & scene_module) {
    for (const auto & id : lanelet_id_set) {
      if (isModuleRegisteredFromExistingAssociatedModule(id)) {
        return false;
      }
    }
    return true;
  };
}

bool TrafficLightModuleManager::isModuleRegisteredFromExistingAssociatedModule(
  const lanelet::Id & id) const
{
  const auto lane = planner_data_->route_handler_->getLaneletMapPtr()->laneletLayer.get(id);

  for (const auto & registered_id : registered_module_id_set_) {
    const auto registered_lane =
      planner_data_->route_handler_->getLaneletMapPtr()->laneletLayer.get(registered_id);
    for (const auto & registered_element : registered_lane.regulatoryElementsAs<TrafficLight>()) {
      for (const auto & element : lane.regulatoryElementsAs<TrafficLight>()) {
        if (hasSameTrafficLight(element, registered_element)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool TrafficLightModuleManager::hasSameTrafficLight(
  const lanelet::TrafficLightConstPtr element,
  const lanelet::TrafficLightConstPtr registered_element) const
{
  for (const auto & traffic_light : element->trafficLights()) {
    for (const auto & registered_traffic_light : registered_element->trafficLights()) {
      if (traffic_light.id() == registered_traffic_light.id()) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::TrafficLightModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
