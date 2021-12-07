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
namespace
{
std::vector<lanelet::ConstLanelet> getLaneletsOnPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapPtr lanelet_map)
{
  std::vector<lanelet::ConstLanelet> lanelets;

  for (const auto & p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    const auto lane = lanelet_map->laneletLayer.get(lane_id);
    if (!lanelet::utils::contains(lanelets, lane)) {
      lanelets.push_back(lane);
    }
  }

  return lanelets;
}

std::set<int64_t> getLaneIdSetOnPath(const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  std::set<int64_t> lane_id_set;

  for (const auto & p : path.points) {
    for (const auto & lane_id : p.lane_ids) {
      lane_id_set.insert(lane_id);
    }
  }

  return lane_id_set;
}

}  // namespace

IntersectionModuleManager::IntersectionModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(getModuleName());
  auto & ip = intersection_param_;
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  ip.state_transit_margin_time = node.declare_parameter(ns + ".state_transit_margin_time", 2.0);
  ip.decel_velocity = node.declare_parameter(ns + ".decel_velocity", 30.0 / 3.6);
  ip.stop_line_margin = node.declare_parameter(ns + ".stop_line_margin", 1.0);
  ip.stuck_vehicle_detect_dist = node.declare_parameter(ns + ".stuck_vehicle_detect_dist", 3.0);
  ip.stuck_vehicle_ignore_dist = node.declare_parameter(ns + ".stuck_vehicle_ignore_dist", 5.0) +
                                 vehicle_info.max_longitudinal_offset_m;
  ip.stuck_vehicle_vel_thr = node.declare_parameter(ns + ".stuck_vehicle_vel_thr", 3.0 / 3.6);
  ip.intersection_velocity = node.declare_parameter(ns + ".intersection_velocity", 10.0 / 3.6);
  ip.intersection_max_acc = node.declare_parameter(ns + ".intersection_max_accel", 0.5);
  ip.detection_area_margin = node.declare_parameter(ns + ".detection_area_margin", 0.5);
  ip.detection_area_length = node.declare_parameter(ns + ".detection_area_length", 200.0);
  ip.detection_area_angle_thr =
    node.declare_parameter(ns + ".detection_area_angle_threshold", M_PI / 4.0);
  ip.min_predicted_path_confidence =
    node.declare_parameter(ns + ".min_predicted_path_confidence", 0.05);
  ip.external_input_timeout = node.declare_parameter(ns + ".walkway.external_input_timeout", 1.0);
  ip.collision_start_margin_time = node.declare_parameter(ns + ".collision_start_margin_time", 5.0);
  ip.collision_end_margin_time = node.declare_parameter(ns + ".collision_end_margin_time", 2.0);
  auto & mp = merge_from_private_area_param_;
  mp.stop_duration_sec =
    node.declare_parameter(ns + ".merge_from_private_area.stop_duration_sec", 1.0);
  mp.intersection_param = intersection_param_;
}

void IntersectionModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lanelets = getLaneletsOnPath(path, planner_data_->lanelet_map);
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
    if (i + 1 < lanelets.size()) {
      const auto next_lane = lanelets.at(i + 1);
      const std::string lane_location = ll.attributeOr("location", "else");
      const std::string next_lane_location = next_lane.attributeOr("location", "else");
      if (lane_location == "private" && next_lane_location != "private") {
        registerModule(std::make_shared<MergeFromPrivateRoadModule>(
          module_id, lane_id, planner_data_, merge_from_private_area_param_,
          logger_.get_child("merge_from_private_road_module"), clock_));
      }
    }

    registerModule(std::make_shared<IntersectionModule>(
      module_id, lane_id, planner_data_, intersection_param_,
      logger_.get_child("intersection_module"), clock_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
IntersectionModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lane_id_set = getLaneIdSetOnPath(path);

  return [lane_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return lane_id_set.count(scene_module->getModuleId()) == 0;
  };
}
}  // namespace behavior_velocity_planner
