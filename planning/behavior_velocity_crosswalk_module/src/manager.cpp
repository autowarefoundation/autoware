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
#include <tier4_autoware_utils/ros/parameter.hpp>

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{

using lanelet::autoware::Crosswalk;
using tier4_autoware_utils::getOrDeclareParameter;

CrosswalkModuleManager::CrosswalkModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(
    node, getModuleName(),
    getOrDeclareParameter<bool>(node, std::string(getModuleName()) + ".common.enable_rtc"))
{
  const std::string ns(getModuleName());

  // for crosswalk parameters
  auto & cp = crosswalk_planner_param_;
  cp.show_processing_time = getOrDeclareParameter<bool>(node, ns + ".common.show_processing_time");

  // param for input data
  cp.traffic_light_state_timeout =
    getOrDeclareParameter<double>(node, ns + ".common.traffic_light_state_timeout");

  // param for stop position
  cp.stop_distance_from_crosswalk =
    getOrDeclareParameter<double>(node, ns + ".stop_position.stop_distance_from_crosswalk");
  cp.stop_distance_from_object =
    getOrDeclareParameter<double>(node, ns + ".stop_position.stop_distance_from_object");
  cp.far_object_threshold =
    getOrDeclareParameter<double>(node, ns + ".stop_position.far_object_threshold");
  cp.stop_position_threshold =
    getOrDeclareParameter<double>(node, ns + ".stop_position.stop_position_threshold");

  // param for ego velocity
  cp.min_slow_down_velocity =
    getOrDeclareParameter<double>(node, ns + ".slow_down.min_slow_down_velocity");
  cp.max_slow_down_jerk = getOrDeclareParameter<double>(node, ns + ".slow_down.max_slow_down_jerk");
  cp.max_slow_down_accel =
    getOrDeclareParameter<double>(node, ns + ".slow_down.max_slow_down_accel");
  cp.no_relax_velocity = getOrDeclareParameter<double>(node, ns + ".slow_down.no_relax_velocity");

  // param for stuck vehicle
  cp.enable_stuck_check_in_intersection =
    getOrDeclareParameter<bool>(node, ns + ".stuck_vehicle.enable_stuck_check_in_intersection");
  cp.stuck_vehicle_velocity =
    getOrDeclareParameter<double>(node, ns + ".stuck_vehicle.stuck_vehicle_velocity");
  cp.max_stuck_vehicle_lateral_offset =
    getOrDeclareParameter<double>(node, ns + ".stuck_vehicle.max_stuck_vehicle_lateral_offset");
  cp.stuck_vehicle_attention_range =
    getOrDeclareParameter<double>(node, ns + ".stuck_vehicle.stuck_vehicle_attention_range");
  cp.min_acc_for_stuck_vehicle = getOrDeclareParameter<double>(node, ns + ".stuck_vehicle.min_acc");
  cp.max_jerk_for_stuck_vehicle =
    getOrDeclareParameter<double>(node, ns + ".stuck_vehicle.max_jerk");
  cp.min_jerk_for_stuck_vehicle =
    getOrDeclareParameter<double>(node, ns + ".stuck_vehicle.min_jerk");

  // param for pass judge logic
  cp.ego_pass_first_margin_x =
    getOrDeclareParameter<std::vector<double>>(node, ns + ".pass_judge.ego_pass_first_margin_x");
  cp.ego_pass_first_margin_y =
    getOrDeclareParameter<std::vector<double>>(node, ns + ".pass_judge.ego_pass_first_margin_y");
  cp.ego_pass_first_additional_margin =
    getOrDeclareParameter<double>(node, ns + ".pass_judge.ego_pass_first_additional_margin");
  cp.ego_pass_later_margin_x =
    getOrDeclareParameter<std::vector<double>>(node, ns + ".pass_judge.ego_pass_later_margin_x");
  cp.ego_pass_later_margin_y =
    getOrDeclareParameter<std::vector<double>>(node, ns + ".pass_judge.ego_pass_later_margin_y");
  cp.ego_pass_later_additional_margin =
    getOrDeclareParameter<double>(node, ns + ".pass_judge.ego_pass_later_additional_margin");
  cp.max_offset_to_crosswalk_for_yield = getOrDeclareParameter<double>(
    node, ns + ".pass_judge.no_stop_decision.max_offset_to_crosswalk_for_yield");
  cp.min_acc_for_no_stop_decision =
    getOrDeclareParameter<double>(node, ns + ".pass_judge.no_stop_decision.min_acc");
  cp.max_jerk_for_no_stop_decision =
    getOrDeclareParameter<double>(node, ns + ".pass_judge.no_stop_decision.max_jerk");
  cp.min_jerk_for_no_stop_decision =
    getOrDeclareParameter<double>(node, ns + ".pass_judge.no_stop_decision.min_jerk");
  cp.stop_object_velocity =
    getOrDeclareParameter<double>(node, ns + ".pass_judge.stop_object_velocity_threshold");
  cp.min_object_velocity =
    getOrDeclareParameter<double>(node, ns + ".pass_judge.min_object_velocity");
  cp.disable_stop_for_yield_cancel =
    getOrDeclareParameter<bool>(node, ns + ".pass_judge.disable_stop_for_yield_cancel");
  cp.disable_yield_for_new_stopped_object =
    getOrDeclareParameter<bool>(node, ns + ".pass_judge.disable_yield_for_new_stopped_object");
  cp.distance_map_for_no_intention_to_walk = getOrDeclareParameter<std::vector<double>>(
    node, ns + ".pass_judge.distance_map_for_no_intention_to_walk");
  cp.timeout_map_for_no_intention_to_walk = getOrDeclareParameter<std::vector<double>>(
    node, ns + ".pass_judge.timeout_map_for_no_intention_to_walk");
  cp.timeout_ego_stop_for_yield =
    getOrDeclareParameter<double>(node, ns + ".pass_judge.timeout_ego_stop_for_yield");

  // param for target area & object
  cp.crosswalk_attention_range =
    getOrDeclareParameter<double>(node, ns + ".object_filtering.crosswalk_attention_range");
  cp.vehicle_object_cross_angle_threshold = getOrDeclareParameter<double>(
    node, ns + ".object_filtering.vehicle_object_cross_angle_threshold");
  cp.look_unknown =
    getOrDeclareParameter<bool>(node, ns + ".object_filtering.target_object.unknown");
  cp.look_bicycle =
    getOrDeclareParameter<bool>(node, ns + ".object_filtering.target_object.bicycle");
  cp.look_motorcycle =
    getOrDeclareParameter<bool>(node, ns + ".object_filtering.target_object.motorcycle");
  cp.look_pedestrian =
    getOrDeclareParameter<bool>(node, ns + ".object_filtering.target_object.pedestrian");
}

void CrosswalkModuleManager::launchNewModules(const PathWithLaneId & path)
{
  const auto rh = planner_data_->route_handler_;

  const auto launch = [this, &path](
                        const auto road_lanelet_id, const auto crosswalk_lanelet_id,
                        const std::optional<int64_t> & reg_elem_id) {
    if (isModuleRegistered(crosswalk_lanelet_id)) {
      return;
    }

    const auto & p = crosswalk_planner_param_;
    const auto logger = logger_.get_child("crosswalk_module");
    const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();

    // NOTE: module_id is always a lane id so that isModuleRegistered works correctly in the case
    //       where both regulatory element and non-regulatory element crosswalks exist.
    registerModule(std::make_shared<CrosswalkModule>(
      node_, road_lanelet_id, crosswalk_lanelet_id, reg_elem_id, lanelet_map_ptr, p, logger,
      clock_));
    generateUUID(crosswalk_lanelet_id);
    updateRTCStatus(
      getUUID(crosswalk_lanelet_id), true, std::numeric_limits<double>::lowest(),
      path.header.stamp);
  };

  const auto crosswalk_reg_elem_map = planning_utils::getRegElemMapOnPath<Crosswalk>(
    path, rh->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  for (const auto & crosswalk : crosswalk_reg_elem_map) {
    // NOTE: The former id is a lane id, and the latter one is a regulatory element's id.
    launch(crosswalk.second.id(), crosswalk.first->crosswalkLanelet().id(), crosswalk.first->id());
  }

  const auto crosswalk_lanelets = getCrosswalksOnPath(
    planner_data_->current_odometry->pose, path, rh->getLaneletMapPtr(), rh->getOverallGraphPtr());

  for (const auto & crosswalk : crosswalk_lanelets) {
    launch(crosswalk.first, crosswalk.second.id(), std::nullopt);
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
CrosswalkModuleManager::getModuleExpiredFunction(const PathWithLaneId & path)
{
  const auto rh = planner_data_->route_handler_;

  std::set<int64_t> crosswalk_id_set;

  crosswalk_id_set = getCrosswalkIdSetOnPath(
    planner_data_->current_odometry->pose, path, rh->getLaneletMapPtr(), rh->getOverallGraphPtr());

  const auto crosswalk_reg_elem_map = planning_utils::getRegElemMapOnPath<Crosswalk>(
    path, rh->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  for (const auto & crosswalk : crosswalk_reg_elem_map) {
    crosswalk_id_set.insert(crosswalk.first->crosswalkLanelet().id());
  }

  return [crosswalk_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return crosswalk_id_set.count(scene_module->getModuleId()) == 0;
  };
}
}  // namespace behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_velocity_planner::CrosswalkModulePlugin, behavior_velocity_planner::PluginInterface)
