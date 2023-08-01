// Copyright 2023 TIER IV, Inc.
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

#include "scene_out_of_lane.hpp"

#include <behavior_velocity_planner_common/utilization/util.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
OutOfLaneModuleManager::OutOfLaneModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName()), module_id_(0UL)
{
  const std::string ns(getModuleName());
  auto & pp = planner_param_;

  pp.mode = node.declare_parameter<std::string>(ns + ".mode");
  pp.skip_if_already_overlapping =
    node.declare_parameter<bool>(ns + ".skip_if_already_overlapping");

  pp.time_threshold = node.declare_parameter<double>(ns + ".threshold.time_threshold");
  pp.intervals_ego_buffer = node.declare_parameter<double>(ns + ".intervals.ego_time_buffer");
  pp.intervals_obj_buffer = node.declare_parameter<double>(ns + ".intervals.objects_time_buffer");
  pp.ttc_threshold = node.declare_parameter<double>(ns + ".ttc.threshold");

  pp.objects_min_vel = node.declare_parameter<double>(ns + ".objects.minimum_velocity");
  pp.objects_use_predicted_paths =
    node.declare_parameter<bool>(ns + ".objects.use_predicted_paths");
  pp.objects_min_confidence =
    node.declare_parameter<double>(ns + ".objects.predicted_path_min_confidence");

  pp.overlap_min_dist = node.declare_parameter<double>(ns + ".overlap.minimum_distance");
  pp.overlap_extra_length = node.declare_parameter<double>(ns + ".overlap.extra_length");

  pp.skip_if_over_max_decel = node.declare_parameter<bool>(ns + ".action.skip_if_over_max_decel");
  pp.strict = node.declare_parameter<bool>(ns + ".action.strict");
  pp.dist_buffer = node.declare_parameter<double>(ns + ".action.distance_buffer");
  pp.slow_velocity = node.declare_parameter<double>(ns + ".action.slowdown.velocity");
  pp.slow_dist_threshold =
    node.declare_parameter<double>(ns + ".action.slowdown.distance_threshold");
  pp.stop_dist_threshold = node.declare_parameter<double>(ns + ".action.stop.distance_threshold");

  pp.extra_front_offset = node.declare_parameter<double>(ns + ".ego.extra_front_offset");
  pp.extra_rear_offset = node.declare_parameter<double>(ns + ".ego.extra_rear_offset");
  pp.extra_left_offset = node.declare_parameter<double>(ns + ".ego.extra_left_offset");
  pp.extra_right_offset = node.declare_parameter<double>(ns + ".ego.extra_right_offset");
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  pp.front_offset = vehicle_info.max_longitudinal_offset_m;
  pp.rear_offset = vehicle_info.min_longitudinal_offset_m;
  pp.left_offset = vehicle_info.max_lateral_offset_m;
  pp.right_offset = vehicle_info.min_lateral_offset_m;
}

void OutOfLaneModuleManager::launchNewModules(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  if (path.points.empty()) return;
  // general
  if (!isModuleRegistered(module_id_)) {
    registerModule(std::make_shared<out_of_lane::OutOfLaneModule>(
      module_id_, planner_param_, logger_.get_child("out_of_lane_module"), clock_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
OutOfLaneModuleManager::getModuleExpiredFunction(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  return [path]([[maybe_unused]] const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return false;
  };
}
}  // namespace behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_velocity_planner::OutOfLaneModulePlugin, behavior_velocity_planner::PluginInterface)
