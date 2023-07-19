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

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{

using lanelet::autoware::Crosswalk;

CrosswalkModuleManager::CrosswalkModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(node, getModuleName())
{
  const std::string ns(getModuleName());

  // for crosswalk parameters
  auto & cp = crosswalk_planner_param_;
  cp.show_processing_time = node.declare_parameter<bool>(ns + ".show_processing_time");

  // param for stop position
  cp.stop_line_distance = node.declare_parameter<double>(ns + ".stop_line_distance");
  cp.stop_margin = node.declare_parameter<double>(ns + ".stop_margin");
  cp.stop_line_margin = node.declare_parameter<double>(ns + ".stop_line_margin");
  cp.stop_position_threshold = node.declare_parameter<double>(ns + ".stop_position_threshold");

  // param for ego velocity
  cp.min_slow_down_velocity = node.declare_parameter<double>(ns + ".min_slow_down_velocity");
  cp.max_slow_down_jerk = node.declare_parameter<double>(ns + ".max_slow_down_jerk");
  cp.max_slow_down_accel = node.declare_parameter<double>(ns + ".max_slow_down_accel");
  cp.no_relax_velocity = node.declare_parameter<double>(ns + ".no_relax_velocity");

  // param for stuck vehicle
  cp.stuck_vehicle_velocity = node.declare_parameter<double>(ns + ".stuck_vehicle_velocity");
  cp.max_lateral_offset = node.declare_parameter<double>(ns + ".max_lateral_offset");
  cp.stuck_vehicle_attention_range =
    node.declare_parameter<double>(ns + ".stuck_vehicle_attention_range");

  // param for pass judge logic
  cp.ego_pass_first_margin = node.declare_parameter<double>(ns + ".ego_pass_first_margin");
  cp.ego_pass_later_margin = node.declare_parameter<double>(ns + ".ego_pass_later_margin");
  cp.stop_object_velocity = node.declare_parameter<double>(ns + ".stop_object_velocity_threshold");
  cp.min_object_velocity = node.declare_parameter<double>(ns + ".min_object_velocity");
  cp.disable_stop_for_yield_cancel =
    node.declare_parameter<bool>(ns + ".disable_stop_for_yield_cancel");
  cp.max_yield_timeout = node.declare_parameter<double>(ns + ".max_yield_timeout");
  cp.ego_yield_query_stop_duration =
    node.declare_parameter<double>(ns + ".ego_yield_query_stop_duration");

  // param for input data
  cp.tl_state_timeout = node.declare_parameter<double>(ns + ".tl_state_timeout");

  // param for target area & object
  cp.crosswalk_attention_range = node.declare_parameter<double>(ns + ".crosswalk_attention_range");
  cp.look_unknown = node.declare_parameter<bool>(ns + ".target_object.unknown");
  cp.look_bicycle = node.declare_parameter<bool>(ns + ".target_object.bicycle");
  cp.look_motorcycle = node.declare_parameter<bool>(ns + ".target_object.motorcycle");
  cp.look_pedestrian = node.declare_parameter<bool>(ns + ".target_object.pedestrian");
}

void CrosswalkModuleManager::launchNewModules(const PathWithLaneId & path)
{
  const auto rh = planner_data_->route_handler_;
  if (!opt_use_regulatory_element_) {
    opt_use_regulatory_element_ = checkRegulatoryElementExistence(rh->getLaneletMapPtr());
    std::ostringstream string_stream;
    string_stream << "use crosswalk regulatory element: ";
    string_stream << std::boolalpha << *opt_use_regulatory_element_;
    RCLCPP_INFO_STREAM(logger_, string_stream.str());
  }

  const auto launch = [this, &path](const auto id) {
    if (isModuleRegistered(id)) {
      return;
    }

    const auto & p = crosswalk_planner_param_;
    const auto logger = logger_.get_child("crosswalk_module");
    const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();

    registerModule(std::make_shared<CrosswalkModule>(
      id, lanelet_map_ptr, p, *opt_use_regulatory_element_, logger, clock_));
    generateUUID(id);
    updateRTCStatus(getUUID(id), true, std::numeric_limits<double>::lowest(), path.header.stamp);
  };

  if (*opt_use_regulatory_element_) {
    const auto crosswalk_leg_elem_map = planning_utils::getRegElemMapOnPath<Crosswalk>(
      path, rh->getLaneletMapPtr(), planner_data_->current_odometry->pose);

    for (const auto & crosswalk : crosswalk_leg_elem_map) {
      launch(crosswalk.first->id());
    }
  } else {
    const auto crosswalk_lanelets = getCrosswalksOnPath(
      planner_data_->current_odometry->pose, path, rh->getLaneletMapPtr(),
      rh->getOverallGraphPtr());

    for (const auto & crosswalk : crosswalk_lanelets) {
      launch(crosswalk.id());
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
CrosswalkModuleManager::getModuleExpiredFunction(const PathWithLaneId & path)
{
  const auto rh = planner_data_->route_handler_;

  std::set<int64_t> crosswalk_id_set;

  if (*opt_use_regulatory_element_) {
    const auto crosswalk_leg_elem_map = planning_utils::getRegElemMapOnPath<Crosswalk>(
      path, rh->getLaneletMapPtr(), planner_data_->current_odometry->pose);

    for (const auto & crosswalk : crosswalk_leg_elem_map) {
      crosswalk_id_set.insert(crosswalk.first->id());
    }
  } else {
    crosswalk_id_set = getCrosswalkIdSetOnPath(
      planner_data_->current_odometry->pose, path, rh->getLaneletMapPtr(),
      rh->getOverallGraphPtr());
  }

  return [crosswalk_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return crosswalk_id_set.count(scene_module->getModuleId()) == 0;
  };
}
}  // namespace behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_velocity_planner::CrosswalkModulePlugin, behavior_velocity_planner::PluginInterface)
