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

#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/ros/parameter.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using tier4_autoware_utils::getOrDeclareParameter;

IntersectionModuleManager::IntersectionModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(
    node, getModuleName(),
    getOrDeclareParameter<bool>(node, std::string(getModuleName()) + ".enable_rtc.intersection")),
  occlusion_rtc_interface_(
    &node, "intersection_occlusion",
    getOrDeclareParameter<bool>(
      node, std::string(getModuleName()) + ".enable_rtc.intersection_to_occlusion"))
{
  const std::string ns(getModuleName());
  auto & ip = intersection_param_;
  ip.common.attention_area_length =
    getOrDeclareParameter<double>(node, ns + ".common.attention_area_length");
  ip.common.attention_area_margin =
    getOrDeclareParameter<double>(node, ns + ".common.attention_area_margin");
  ip.common.attention_area_angle_threshold =
    getOrDeclareParameter<double>(node, ns + ".common.attention_area_angle_threshold");
  ip.common.use_intersection_area =
    getOrDeclareParameter<bool>(node, ns + ".common.use_intersection_area");
  ip.common.default_stopline_margin =
    getOrDeclareParameter<double>(node, ns + ".common.default_stopline_margin");
  ip.common.stopline_overshoot_margin =
    getOrDeclareParameter<double>(node, ns + ".common.stopline_overshoot_margin");
  ip.common.path_interpolation_ds =
    getOrDeclareParameter<double>(node, ns + ".common.path_interpolation_ds");
  ip.common.max_accel = getOrDeclareParameter<double>(node, ns + ".common.max_accel");
  ip.common.max_jerk = getOrDeclareParameter<double>(node, ns + ".common.max_jerk");
  ip.common.delay_response_time =
    getOrDeclareParameter<double>(node, ns + ".common.delay_response_time");

  ip.stuck_vehicle.turn_direction.left =
    getOrDeclareParameter<bool>(node, ns + ".stuck_vehicle.turn_direction.left");
  ip.stuck_vehicle.turn_direction.right =
    getOrDeclareParameter<bool>(node, ns + ".stuck_vehicle.turn_direction.right");
  ip.stuck_vehicle.turn_direction.straight =
    getOrDeclareParameter<bool>(node, ns + ".stuck_vehicle.turn_direction.straight");
  ip.stuck_vehicle.use_stuck_stopline =
    getOrDeclareParameter<bool>(node, ns + ".stuck_vehicle.use_stuck_stopline");
  ip.stuck_vehicle.stuck_vehicle_detect_dist =
    getOrDeclareParameter<double>(node, ns + ".stuck_vehicle.stuck_vehicle_detect_dist");
  ip.stuck_vehicle.stuck_vehicle_velocity_threshold =
    getOrDeclareParameter<double>(node, ns + ".stuck_vehicle.stuck_vehicle_velocity_threshold");
  ip.stuck_vehicle.disable_against_private_lane =
    getOrDeclareParameter<bool>(node, ns + ".stuck_vehicle.disable_against_private_lane");

  ip.yield_stuck.turn_direction.left =
    getOrDeclareParameter<bool>(node, ns + ".yield_stuck.turn_direction.left");
  ip.yield_stuck.turn_direction.right =
    getOrDeclareParameter<bool>(node, ns + ".yield_stuck.turn_direction.right");
  ip.yield_stuck.turn_direction.straight =
    getOrDeclareParameter<bool>(node, ns + ".yield_stuck.turn_direction.straight");
  ip.yield_stuck.distance_threshold =
    getOrDeclareParameter<double>(node, ns + ".yield_stuck.distance_threshold");

  ip.collision_detection.consider_wrong_direction_vehicle =
    getOrDeclareParameter<bool>(node, ns + ".collision_detection.consider_wrong_direction_vehicle");
  ip.collision_detection.collision_detection_hold_time =
    getOrDeclareParameter<double>(node, ns + ".collision_detection.collision_detection_hold_time");
  ip.collision_detection.min_predicted_path_confidence =
    getOrDeclareParameter<double>(node, ns + ".collision_detection.min_predicted_path_confidence");
  ip.collision_detection.keep_detection_velocity_threshold = getOrDeclareParameter<double>(
    node, ns + ".collision_detection.keep_detection_velocity_threshold");
  ip.collision_detection.velocity_profile.use_upstream =
    getOrDeclareParameter<bool>(node, ns + ".collision_detection.velocity_profile.use_upstream");
  ip.collision_detection.velocity_profile.minimum_upstream_velocity = getOrDeclareParameter<double>(
    node, ns + ".collision_detection.velocity_profile.minimum_upstream_velocity");
  ip.collision_detection.velocity_profile.default_velocity = getOrDeclareParameter<double>(
    node, ns + ".collision_detection.velocity_profile.default_velocity");
  ip.collision_detection.velocity_profile.minimum_default_velocity = getOrDeclareParameter<double>(
    node, ns + ".collision_detection.velocity_profile.minimum_default_velocity");
  ip.collision_detection.fully_prioritized.collision_start_margin_time =
    getOrDeclareParameter<double>(
      node, ns + ".collision_detection.fully_prioritized.collision_start_margin_time");
  ip.collision_detection.fully_prioritized.collision_end_margin_time =
    getOrDeclareParameter<double>(
      node, ns + ".collision_detection.fully_prioritized.collision_end_margin_time");
  ip.collision_detection.partially_prioritized.collision_start_margin_time =
    getOrDeclareParameter<double>(
      node, ns + ".collision_detection.partially_prioritized.collision_start_margin_time");
  ip.collision_detection.partially_prioritized.collision_end_margin_time =
    getOrDeclareParameter<double>(
      node, ns + ".collision_detection.partially_prioritized.collision_end_margin_time");
  ip.collision_detection.not_prioritized.collision_start_margin_time =
    getOrDeclareParameter<double>(
      node, ns + ".collision_detection.not_prioritized.collision_start_margin_time");
  ip.collision_detection.not_prioritized.collision_end_margin_time = getOrDeclareParameter<double>(
    node, ns + ".collision_detection.not_prioritized.collision_end_margin_time");
  ip.collision_detection.yield_on_green_traffic_light.distance_to_assigned_lanelet_start =
    getOrDeclareParameter<double>(
      node,
      ns + ".collision_detection.yield_on_green_traffic_light.distance_to_assigned_lanelet_start");
  ip.collision_detection.yield_on_green_traffic_light.duration = getOrDeclareParameter<double>(
    node, ns + ".collision_detection.yield_on_green_traffic_light.duration");
  ip.collision_detection.yield_on_green_traffic_light.object_dist_to_stopline =
    getOrDeclareParameter<double>(
      node, ns + ".collision_detection.yield_on_green_traffic_light.object_dist_to_stopline");
  ip.collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration =
    getOrDeclareParameter<double>(
      node, ns + ".collision_detection.ignore_on_amber_traffic_light.object_expected_deceleration");
  ip.collision_detection.ignore_on_red_traffic_light.object_margin_to_path =
    getOrDeclareParameter<double>(
      node, ns + ".collision_detection.ignore_on_red_traffic_light.object_margin_to_path");

  ip.occlusion.enable = getOrDeclareParameter<bool>(node, ns + ".occlusion.enable");
  ip.occlusion.occlusion_attention_area_length =
    getOrDeclareParameter<double>(node, ns + ".occlusion.occlusion_attention_area_length");
  ip.occlusion.free_space_max = getOrDeclareParameter<int>(node, ns + ".occlusion.free_space_max");
  ip.occlusion.occupied_min = getOrDeclareParameter<int>(node, ns + ".occlusion.occupied_min");
  ip.occlusion.denoise_kernel =
    getOrDeclareParameter<double>(node, ns + ".occlusion.denoise_kernel");
  ip.occlusion.attention_lane_crop_curvature_threshold =
    getOrDeclareParameter<double>(node, ns + ".occlusion.attention_lane_crop_curvature_threshold");
  ip.occlusion.attention_lane_curvature_calculation_ds =
    getOrDeclareParameter<double>(node, ns + ".occlusion.attention_lane_curvature_calculation_ds");
  ip.occlusion.creep_during_peeking.enable =
    getOrDeclareParameter<bool>(node, ns + ".occlusion.creep_during_peeking.enable");
  ip.occlusion.creep_during_peeking.creep_velocity =
    getOrDeclareParameter<double>(node, ns + ".occlusion.creep_during_peeking.creep_velocity");
  ip.occlusion.peeking_offset =
    getOrDeclareParameter<double>(node, ns + ".occlusion.peeking_offset");
  ip.occlusion.possible_object_bbox =
    getOrDeclareParameter<std::vector<double>>(node, ns + ".occlusion.possible_object_bbox");
  ip.occlusion.ignore_parked_vehicle_speed_threshold =
    getOrDeclareParameter<double>(node, ns + ".occlusion.ignore_parked_vehicle_speed_threshold");
  ip.occlusion.occlusion_detection_hold_time =
    getOrDeclareParameter<double>(node, ns + ".occlusion.occlusion_detection_hold_time");
  ip.occlusion.temporal_stop_time_before_peeking =
    getOrDeclareParameter<double>(node, ns + ".occlusion.temporal_stop_time_before_peeking");
  ip.occlusion.temporal_stop_before_attention_area =
    getOrDeclareParameter<bool>(node, ns + ".occlusion.temporal_stop_before_attention_area");
  ip.occlusion.creep_velocity_without_traffic_light =
    getOrDeclareParameter<double>(node, ns + ".occlusion.creep_velocity_without_traffic_light");
  ip.occlusion.static_occlusion_with_traffic_light_timeout = getOrDeclareParameter<double>(
    node, ns + ".occlusion.static_occlusion_with_traffic_light_timeout");

  ip.debug.ttc = getOrDeclareParameter<std::vector<int64_t>>(node, ns + ".debug.ttc");
}

void IntersectionModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto routing_graph = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto lanelet_map = planner_data_->route_handler_->getLaneletMapPtr();

  const auto lanelets =
    planning_utils::getLaneletsOnPath(path, lanelet_map, planner_data_->current_odometry->pose);
  // run occlusion detection only in the first intersection
  // TODO(Mamoru Sobue): remove `enable_occlusion_detection` variable
  const bool enable_occlusion_detection = intersection_param_.occlusion.enable;
  for (size_t i = 0; i < lanelets.size(); i++) {
    const auto ll = lanelets.at(i);
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    // Is intersection?
    const std::string turn_direction = ll.attributeOr("turn_direction", "else");
    const auto is_intersection =
      turn_direction == "right" || turn_direction == "left" || turn_direction == "straight";
    if (!is_intersection) {
      continue;
    }

    if (hasSameParentLaneletAndTurnDirectionWithRegistered(ll)) {
      continue;
    }

    const std::string location = ll.attributeOr("location", "else");
    const auto associative_ids =
      planning_utils::getAssociativeIntersectionLanelets(ll, lanelet_map, routing_graph);
    bool has_traffic_light = false;
    if (const auto tl_reg_elems = ll.regulatoryElementsAs<lanelet::TrafficLight>();
        tl_reg_elems.size() != 0) {
      const auto tl_reg_elem = tl_reg_elems.front();
      const auto stopline_opt = tl_reg_elem->stopLine();
      if (!!stopline_opt) has_traffic_light = true;
    }
    const auto new_module = std::make_shared<IntersectionModule>(
      module_id, lane_id, planner_data_, intersection_param_, associative_ids, turn_direction,
      has_traffic_light, enable_occlusion_detection, node_,
      logger_.get_child("intersection_module"), clock_);
    generateUUID(module_id);
    /* set RTC status as non_occluded status initially */
    const UUID uuid = getUUID(new_module->getModuleId());
    const auto occlusion_uuid = new_module->getOcclusionUUID();
    rtc_interface_.updateCooperateStatus(
      uuid, true, std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
      clock_->now());
    occlusion_rtc_interface_.updateCooperateStatus(
      occlusion_uuid, true, std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::lowest(), clock_->now());
    registerModule(std::move(new_module));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
IntersectionModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lane_set = planning_utils::getLaneletsOnPath(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [this, lane_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    const auto intersection_module = std::dynamic_pointer_cast<IntersectionModule>(scene_module);
    const auto & associative_ids = intersection_module->getAssociativeIds();
    for (const auto & lane : lane_set) {
      const std::string turn_direction = lane.attributeOr("turn_direction", "else");
      const auto is_intersection =
        turn_direction == "right" || turn_direction == "left" || turn_direction == "straight";
      if (!is_intersection) {
        continue;
      }

      if (associative_ids.find(lane.id()) != associative_ids.end() /* contains */) {
        return false;
      }
    }
    return true;
  };
}

bool IntersectionModuleManager::hasSameParentLaneletAndTurnDirectionWithRegistered(
  const lanelet::ConstLanelet & lane) const
{
  for (const auto & scene_module : scene_modules_) {
    const auto intersection_module = std::dynamic_pointer_cast<IntersectionModule>(scene_module);
    const auto & associative_ids = intersection_module->getAssociativeIds();
    if (associative_ids.find(lane.id()) != associative_ids.end()) {
      return true;
    }
  }
  return false;
}

void IntersectionModuleManager::sendRTC(const Time & stamp)
{
  for (const auto & scene_module : scene_modules_) {
    const auto intersection_module = std::dynamic_pointer_cast<IntersectionModule>(scene_module);
    const UUID uuid = getUUID(scene_module->getModuleId());
    const bool safety =
      scene_module->isSafe() && (!intersection_module->isOcclusionFirstStopRequired());
    updateRTCStatus(uuid, safety, scene_module->getDistance(), stamp);
    const auto occlusion_uuid = intersection_module->getOcclusionUUID();
    const auto occlusion_distance = intersection_module->getOcclusionDistance();
    const auto occlusion_safety = intersection_module->getOcclusionSafety();
    occlusion_rtc_interface_.updateCooperateStatus(
      occlusion_uuid, occlusion_safety, occlusion_distance, occlusion_distance, stamp);
  }
  rtc_interface_.publishCooperateStatus(stamp);  // publishRTCStatus()
  occlusion_rtc_interface_.publishCooperateStatus(stamp);
}

void IntersectionModuleManager::setActivation()
{
  for (const auto & scene_module : scene_modules_) {
    const auto intersection_module = std::dynamic_pointer_cast<IntersectionModule>(scene_module);
    const auto occlusion_uuid = intersection_module->getOcclusionUUID();
    scene_module->setActivation(rtc_interface_.isActivated(getUUID(scene_module->getModuleId())));
    intersection_module->setOcclusionActivation(
      occlusion_rtc_interface_.isActivated(occlusion_uuid));
  }
}

void IntersectionModuleManager::deleteExpiredModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto isModuleExpired = getModuleExpiredFunction(path);

  // Copy container to avoid iterator corruption
  // due to scene_modules_.erase() in unregisterModule()
  const auto copied_scene_modules = scene_modules_;

  for (const auto & scene_module : copied_scene_modules) {
    if (isModuleExpired(scene_module)) {
      // default
      removeRTCStatus(getUUID(scene_module->getModuleId()));
      removeUUID(scene_module->getModuleId());
      // occlusion
      const auto intersection_module = std::dynamic_pointer_cast<IntersectionModule>(scene_module);
      const auto occlusion_uuid = intersection_module->getOcclusionUUID();
      occlusion_rtc_interface_.removeCooperateStatus(occlusion_uuid);
      unregisterModule(scene_module);
    }
  }
}

MergeFromPrivateModuleManager::MergeFromPrivateModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());
  auto & mp = merge_from_private_area_param_;
  mp.stop_duration_sec = getOrDeclareParameter<double>(node, ns + ".stop_duration_sec");
  mp.attention_area_length =
    node.get_parameter("intersection.common.attention_area_length").as_double();
  mp.stopline_margin = getOrDeclareParameter<double>(node, ns + ".stopline_margin");
  mp.path_interpolation_ds =
    node.get_parameter("intersection.common.path_interpolation_ds").as_double();
  mp.stop_distance_threshold = getOrDeclareParameter<double>(node, ns + ".stop_distance_threshold");
}

void MergeFromPrivateModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto routing_graph = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto lanelet_map = planner_data_->route_handler_->getLaneletMapPtr();

  const auto lanelets =
    planning_utils::getLaneletsOnPath(path, lanelet_map, planner_data_->current_odometry->pose);
  for (size_t i = 0; i < lanelets.size(); i++) {
    const auto ll = lanelets.at(i);
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    if (isModuleRegistered(module_id)) {
      continue;
    }

    // Is intersection?
    const std::string turn_direction = ll.attributeOr("turn_direction", "else");
    const auto is_intersection =
      turn_direction == "right" || turn_direction == "left" || turn_direction == "straight";
    if (!is_intersection) {
      continue;
    }

    // Is merging from private road?
    // In case the goal is in private road, check if this lanelet is conflicting with urban lanelet
    const std::string lane_location = ll.attributeOr("location", "else");
    if (lane_location != "private") {
      continue;
    }

    if (hasSameParentLaneletAndTurnDirectionWithRegistered(ll)) {
      continue;
    }

    if (i + 1 < lanelets.size()) {
      const auto next_lane = lanelets.at(i + 1);
      const std::string next_lane_location = next_lane.attributeOr("location", "else");
      if (next_lane_location != "private") {
        const auto associative_ids =
          planning_utils::getAssociativeIntersectionLanelets(ll, lanelet_map, routing_graph);
        registerModule(std::make_shared<MergeFromPrivateRoadModule>(
          module_id, lane_id, planner_data_, merge_from_private_area_param_, associative_ids,
          logger_.get_child("merge_from_private_road_module"), clock_));
        continue;
      }
    } else {
      const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();
      const auto conflicting_lanelets =
        lanelet::utils::getConflictingLanelets(routing_graph_ptr, ll);
      for (auto && conflicting_lanelet : conflicting_lanelets) {
        const std::string conflicting_attr = conflicting_lanelet.attributeOr("location", "else");
        if (conflicting_attr == "urban") {
          const auto associative_ids =
            planning_utils::getAssociativeIntersectionLanelets(ll, lanelet_map, routing_graph);
          registerModule(std::make_shared<MergeFromPrivateRoadModule>(
            module_id, lane_id, planner_data_, merge_from_private_area_param_, associative_ids,
            logger_.get_child("merge_from_private_road_module"), clock_));
          continue;
        }
      }
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
MergeFromPrivateModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lane_set = planning_utils::getLaneletsOnPath(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [this, lane_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    const auto merge_from_private_module =
      std::dynamic_pointer_cast<MergeFromPrivateRoadModule>(scene_module);
    const auto & associative_ids = merge_from_private_module->getAssociativeIds();
    for (const auto & lane : lane_set) {
      const std::string turn_direction = lane.attributeOr("turn_direction", "else");
      const auto is_intersection =
        turn_direction == "right" || turn_direction == "left" || turn_direction == "straight";
      if (!is_intersection) {
        continue;
      }

      if (associative_ids.find(lane.id()) != associative_ids.end() /* contains */) {
        return false;
      }
    }
    return true;
  };
}

bool MergeFromPrivateModuleManager::hasSameParentLaneletAndTurnDirectionWithRegistered(
  const lanelet::ConstLanelet & lane) const
{
  for (const auto & scene_module : scene_modules_) {
    const auto merge_from_private_module =
      std::dynamic_pointer_cast<MergeFromPrivateRoadModule>(scene_module);
    const auto & associative_ids = merge_from_private_module->getAssociativeIds();
    if (associative_ids.find(lane.id()) != associative_ids.end()) {
      return true;
    }
  }
  return false;
}

}  // namespace behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_velocity_planner::IntersectionModulePlugin, behavior_velocity_planner::PluginInterface)
PLUGINLIB_EXPORT_CLASS(
  behavior_velocity_planner::MergeFromPrivateModulePlugin,
  behavior_velocity_planner::PluginInterface)
