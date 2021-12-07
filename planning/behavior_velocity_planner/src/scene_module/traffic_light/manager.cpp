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
namespace
{
std::unordered_map<lanelet::TrafficLightConstPtr, lanelet::ConstLanelet>
getTrafficLightRegElemsOnPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map)
{
  std::unordered_map<lanelet::TrafficLightConstPtr, lanelet::ConstLanelet> traffic_light_reg_elems;

  for (const auto & p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    const auto ll = lanelet_map->laneletLayer.get(lane_id);

    const auto tls = ll.regulatoryElementsAs<const lanelet::TrafficLight>();
    for (const auto & tl : tls) {
      traffic_light_reg_elems.insert(std::make_pair(tl, ll));
    }
  }

  return traffic_light_reg_elems;
}

std::set<int64_t> getLaneletIdSetOnPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map)
{
  std::set<int64_t> lanelet_id_set;
  for (const auto & traffic_light_reg_elem : getTrafficLightRegElemsOnPath(path, lanelet_map)) {
    lanelet_id_set.insert(traffic_light_reg_elem.second.id());
  }
  return lanelet_id_set;
}

}  // namespace

TrafficLightModuleManager::TrafficLightModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
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
  autoware_planning_msgs::msg::StopReasonArray stop_reason_array;
  autoware_auto_perception_msgs::msg::LookingTrafficSignal tl_state;

  tl_state.header.stamp = path->header.stamp;
  tl_state.is_module_running = false;

  stop_reason_array.header.frame_id = "map";
  stop_reason_array.header.stamp = path->header.stamp;
  first_stop_path_point_index_ = static_cast<int>(path->points.size() - 1);
  first_ref_stop_path_point_index_ = static_cast<int>(path->points.size() - 1);
  for (const auto & scene_module : scene_modules_) {
    autoware_planning_msgs::msg::StopReason stop_reason;
    std::shared_ptr<TrafficLightModule> traffic_light_scene_module(
      std::dynamic_pointer_cast<TrafficLightModule>(scene_module));
    traffic_light_scene_module->setPlannerData(planner_data_);
    traffic_light_scene_module->modifyPathVelocity(path, &stop_reason);
    stop_reason_array.stop_reasons.emplace_back(stop_reason);
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
  }
  if (!stop_reason_array.stop_reasons.empty()) {
    pub_stop_reason_->publish(stop_reason_array);
  }
  pub_debug_->publish(debug_marker_array);
  pub_tl_state_->publish(tl_state);
}

void TrafficLightModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & traffic_light_reg_elem :
       getTrafficLightRegElemsOnPath(path, planner_data_->lanelet_map)) {
    const auto stop_line = traffic_light_reg_elem.first->stopLine();

    if (!stop_line) {
      RCLCPP_FATAL(
        logger_, "No stop line at traffic_light_reg_elem_id = %ld, please fix the map!",
        traffic_light_reg_elem.first->id());
      continue;
    }

    // Use lanelet_id to unregister module when the route is changed
    const auto module_id = traffic_light_reg_elem.second.id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<TrafficLightModule>(
        module_id, *(traffic_light_reg_elem.first), traffic_light_reg_elem.second, planner_param_,
        logger_.get_child("traffic_light_module"), clock_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
TrafficLightModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lanelet_id_set = getLaneletIdSetOnPath(path, planner_data_->lanelet_map);

  return [lanelet_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return lanelet_id_set.count(scene_module->getModuleId()) == 0;
  };
}
}  // namespace behavior_velocity_planner
