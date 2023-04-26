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

#include <lanelet2_extension/utility/utilities.hpp>
#include <scene_module/intersection/manager.hpp>
#include <utilization/boost_geometry_helper.hpp>
#include <utilization/util.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
IntersectionModuleManager::IntersectionModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(node, getModuleName()),
  occlusion_rtc_interface_(&node, "intersection_occlusion")
{
  const std::string ns(getModuleName());
  auto & ip = intersection_param_;
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  ip.common.detection_area_margin =
    node.declare_parameter<double>(ns + ".common.detection_area_margin");
  ip.common.detection_area_right_margin =
    node.declare_parameter<double>(ns + ".common.detection_area_right_margin");
  ip.common.detection_area_left_margin =
    node.declare_parameter<double>(ns + ".common.detection_area_left_margin");
  ip.common.detection_area_length =
    node.declare_parameter<double>(ns + ".common.detection_area_length");
  ip.common.detection_area_angle_thr =
    node.declare_parameter<double>(ns + ".common.detection_area_angle_threshold");
  ip.common.stop_line_margin = node.declare_parameter<double>(ns + ".common.stop_line_margin");
  ip.common.intersection_velocity =
    node.declare_parameter<double>(ns + ".common.intersection_velocity");
  ip.common.intersection_max_acc =
    node.declare_parameter<double>(ns + ".common.intersection_max_accel");
  ip.common.stop_overshoot_margin =
    node.declare_parameter<double>(ns + ".common.stop_overshoot_margin");

  ip.stuck_vehicle.use_stuck_stopline =
    node.declare_parameter<bool>(ns + ".stuck_vehicle.use_stuck_stopline");
  ip.stuck_vehicle.stuck_vehicle_detect_dist =
    node.declare_parameter<double>(ns + ".stuck_vehicle.stuck_vehicle_detect_dist");
  ip.stuck_vehicle.stuck_vehicle_ignore_dist =
    node.declare_parameter<double>(ns + ".stuck_vehicle.stuck_vehicle_ignore_dist") +
    vehicle_info.max_longitudinal_offset_m;
  ip.stuck_vehicle.stuck_vehicle_vel_thr =
    node.declare_parameter<double>(ns + ".stuck_vehicle.stuck_vehicle_vel_thr");
  ip.stuck_vehicle.assumed_front_car_decel =
    node.declare_parameter<double>(ns + ".stuck_vehicle.assumed_front_car_decel");
  ip.stuck_vehicle.enable_front_car_decel_prediction =
    node.declare_parameter<bool>(ns + ".stuck_vehicle.enable_front_car_decel_prediction");

  ip.collision_detection.state_transit_margin_time =
    node.declare_parameter<double>(ns + ".collision_detection.state_transit_margin_time");
  ip.collision_detection.min_predicted_path_confidence =
    node.declare_parameter<double>(ns + ".collision_detection.min_predicted_path_confidence");
  ip.collision_detection.minimum_ego_predicted_velocity =
    node.declare_parameter<double>(ns + ".collision_detection.minimum_ego_predicted_velocity");
  ip.collision_detection.collision_start_margin_time =
    node.declare_parameter<double>(ns + ".collision_detection.collision_start_margin_time");
  ip.collision_detection.collision_end_margin_time =
    node.declare_parameter<double>(ns + ".collision_detection.collision_end_margin_time");
  ip.collision_detection.keep_detection_vel_thr =
    node.declare_parameter<double>(ns + ".collision_detection.keep_detection_vel_thr");

  ip.occlusion.enable = node.declare_parameter<bool>(ns + ".occlusion.enable");
  ip.occlusion.occlusion_detection_area_length =
    node.declare_parameter<double>(ns + ".occlusion.occlusion_detection_area_length");
  ip.occlusion.enable_creeping = node.declare_parameter<bool>(ns + ".occlusion.enable_creeping");
  ip.occlusion.occlusion_creep_velocity =
    node.declare_parameter<double>(ns + ".occlusion.occlusion_creep_velocity");
  ip.occlusion.peeking_offset = node.declare_parameter<double>(ns + ".occlusion.peeking_offset");
  ip.occlusion.free_space_max = node.declare_parameter<int>(ns + ".occlusion.free_space_max");
  ip.occlusion.occupied_min = node.declare_parameter<int>(ns + ".occlusion.occupied_min");
  ip.occlusion.do_dp = node.declare_parameter<bool>(ns + ".occlusion.do_dp");
  ip.occlusion.before_creep_stop_time =
    node.declare_parameter<double>(ns + ".occlusion.before_creep_stop_time");
  ip.occlusion.min_vehicle_brake_for_rss =
    node.declare_parameter<double>(ns + ".occlusion.min_vehicle_brake_for_rss");
  ip.occlusion.max_vehicle_velocity_for_rss =
    node.declare_parameter<double>(ns + ".occlusion.max_vehicle_velocity_for_rss");
}

void IntersectionModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto routing_graph = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto lanelet_map = planner_data_->route_handler_->getLaneletMapPtr();

  const auto lanelets =
    planning_utils::getLaneletsOnPath(path, lanelet_map, planner_data_->current_odometry->pose);
  // run occlusion detection only in the first intersection
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

    const auto assoc_ids =
      planning_utils::getAssociativeIntersectionLanelets(ll, lanelet_map, routing_graph);
    const auto new_module = std::make_shared<IntersectionModule>(
      module_id, lane_id, planner_data_, intersection_param_, assoc_ids, enable_occlusion_detection,
      node_, logger_.get_child("intersection_module"), clock_);
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
    const auto & assoc_ids = intersection_module->getAssocIds();
    for (const auto & lane : lane_set) {
      const std::string turn_direction = lane.attributeOr("turn_direction", "else");
      const auto is_intersection =
        turn_direction == "right" || turn_direction == "left" || turn_direction == "straight";
      if (!is_intersection) {
        continue;
      }

      if (assoc_ids.find(lane.id()) != assoc_ids.end() /* contains */) {
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
    const auto & assoc_ids = intersection_module->getAssocIds();
    if (assoc_ids.find(lane.id()) != assoc_ids.end()) {
      return true;
    }
  }
  return false;
}

void IntersectionModuleManager::sendRTC(const Time & stamp)
{
  rtc_interface_.clearCooperateStatus();
  for (const auto & scene_module : scene_modules_) {
    const auto intersection_module = std::dynamic_pointer_cast<IntersectionModule>(scene_module);
    const bool is_occluded = intersection_module->isOccluded();
    const UUID uuid = getUUID(scene_module->getModuleId());
    const auto occlusion_uuid = intersection_module->getOcclusionUUID();
    const auto occlusion_first_stop_uuid = intersection_module->getOcclusionFirstStopUUID();
    if (!is_occluded) {
      // default
      updateRTCStatus(uuid, scene_module->isSafe(), scene_module->getDistance(), stamp);
      occlusion_rtc_interface_.updateCooperateStatus(
        occlusion_uuid, true, std::numeric_limits<double>::lowest(),
        std::numeric_limits<double>::lowest(), stamp);
    } else {
      // occlusion
      const auto occlusion_safety = intersection_module->getOcclusionSafety();
      const auto occlusion_distance = intersection_module->getOcclusionDistance();
      occlusion_rtc_interface_.updateCooperateStatus(
        occlusion_uuid, occlusion_safety, occlusion_distance, occlusion_distance, stamp);

      const auto occlusion_first_stop_safety = intersection_module->getOcclusionFirstStopSafety();
      const auto occlusion_first_stop_distance =
        intersection_module->getOcclusionFirstStopDistance();
      rtc_interface_.updateCooperateStatus(
        occlusion_first_stop_uuid, occlusion_first_stop_safety, occlusion_first_stop_distance,
        occlusion_first_stop_distance, stamp);
    }
  }
  rtc_interface_.publishCooperateStatus(stamp);  // publishRTCStatus()
  occlusion_rtc_interface_.publishCooperateStatus(stamp);
}

void IntersectionModuleManager::setActivation()
{
  for (const auto & scene_module : scene_modules_) {
    const auto intersection_module = std::dynamic_pointer_cast<IntersectionModule>(scene_module);
    const auto occlusion_uuid = intersection_module->getOcclusionUUID();
    const auto occlusion_first_stop_uuid = intersection_module->getOcclusionFirstStopUUID();
    const bool is_occluded = intersection_module->isOccluded();
    if (!is_occluded) {
      // default
      scene_module->setActivation(rtc_interface_.isActivated(getUUID(scene_module->getModuleId())));
      intersection_module->setOcclusionActivation(
        occlusion_rtc_interface_.isActivated(occlusion_uuid));
    } else {
      // occlusion
      intersection_module->setOcclusionActivation(
        occlusion_rtc_interface_.isActivated(occlusion_uuid));
      intersection_module->setOcclusionFirstStopActivation(
        rtc_interface_.isActivated(occlusion_first_stop_uuid));
    }
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
      const auto occlusion_first_uuid = intersection_module->getOcclusionFirstStopUUID();
      occlusion_rtc_interface_.removeCooperateStatus(occlusion_uuid);
      rtc_interface_.removeCooperateStatus(occlusion_first_uuid);
      unregisterModule(scene_module);
    }
  }
}

MergeFromPrivateModuleManager::MergeFromPrivateModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());
  auto & mp = merge_from_private_area_param_;
  mp.stop_duration_sec = node.declare_parameter<double>(ns + ".stop_duration_sec");
  mp.detection_area_length =
    node.get_parameter("intersection.common.detection_area_length").as_double();
  mp.detection_area_right_margin =
    node.get_parameter("intersection.common.detection_area_right_margin").as_double();
  mp.detection_area_left_margin =
    node.get_parameter("intersection.common.detection_area_left_margin").as_double();
  mp.stop_line_margin = node.get_parameter("intersection.common.stop_line_margin").as_double();
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
        const auto assoc_ids =
          planning_utils::getAssociativeIntersectionLanelets(ll, lanelet_map, routing_graph);
        registerModule(std::make_shared<MergeFromPrivateRoadModule>(
          module_id, lane_id, planner_data_, merge_from_private_area_param_, assoc_ids,
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
          const auto assoc_ids =
            planning_utils::getAssociativeIntersectionLanelets(ll, lanelet_map, routing_graph);
          registerModule(std::make_shared<MergeFromPrivateRoadModule>(
            module_id, lane_id, planner_data_, merge_from_private_area_param_, assoc_ids,
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
    const auto & assoc_ids = merge_from_private_module->getAssocIds();
    for (const auto & lane : lane_set) {
      const std::string turn_direction = lane.attributeOr("turn_direction", "else");
      const auto is_intersection =
        turn_direction == "right" || turn_direction == "left" || turn_direction == "straight";
      if (!is_intersection) {
        continue;
      }

      if (assoc_ids.find(lane.id()) != assoc_ids.end() /* contains */) {
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
    const auto & assoc_ids = merge_from_private_module->getAssocIds();
    if (assoc_ids.find(lane.id()) != assoc_ids.end()) {
      return true;
    }
  }
  return false;
}

}  // namespace behavior_velocity_planner
