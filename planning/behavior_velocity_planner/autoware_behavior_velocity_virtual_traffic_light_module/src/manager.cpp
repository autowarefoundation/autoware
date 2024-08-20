// Copyright 2021 Tier IV, Inc.
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

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/LineString.h>

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>

namespace autoware::behavior_velocity_planner
{
using autoware::universe_utils::getOrDeclareParameter;
using lanelet::autoware::VirtualTrafficLight;

VirtualTrafficLightModuleManager::VirtualTrafficLightModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(VirtualTrafficLightModuleManager::getModuleName());

  {
    auto & p = planner_param_;
    p.max_delay_sec = getOrDeclareParameter<double>(node, ns + ".max_delay_sec");
    p.near_line_distance = getOrDeclareParameter<double>(node, ns + ".near_line_distance");
    p.dead_line_margin = getOrDeclareParameter<double>(node, ns + ".dead_line_margin");
    p.hold_stop_margin_distance =
      getOrDeclareParameter<double>(node, ns + ".hold_stop_margin_distance");
    p.max_yaw_deviation_rad = autoware::universe_utils::deg2rad(
      getOrDeclareParameter<double>(node, ns + ".max_yaw_deviation_deg"));
    p.check_timeout_after_stop_line =
      getOrDeclareParameter<bool>(node, ns + ".check_timeout_after_stop_line");
  }
}

void VirtualTrafficLightModuleManager::launchNewModules(
  const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  autoware::universe_utils::LineString2d ego_path_linestring;
  for (const auto & path_point : path.points) {
    ego_path_linestring.push_back(
      autoware::universe_utils::fromMsg(path_point.point.pose.position).to_2d());
  }

  for (const auto & m : planning_utils::getRegElemMapOnPath<VirtualTrafficLight>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    const auto stop_line_opt = m.first->getStopLine();
    if (!stop_line_opt) {
      RCLCPP_FATAL(
        logger_, "No stop line at virtual_traffic_light_reg_elem_id = %ld, please fix the map!",
        m.first->id());
      continue;
    }

    // Use lanelet_id to unregister module when the route is changed
    const auto lane_id = m.second.id();
    const auto module_id = lane_id;
    if (
      !isModuleRegistered(module_id) &&
      boost::geometry::intersects(
        ego_path_linestring, lanelet::utils::to2D(stop_line_opt.value()).basicLineString())) {
      registerModule(std::make_shared<VirtualTrafficLightModule>(
        module_id, lane_id, *m.first, m.second, planner_param_,
        logger_.get_child("virtual_traffic_light_module"), clock_));
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
VirtualTrafficLightModuleManager::getModuleExpiredFunction(
  const tier4_planning_msgs::msg::PathWithLaneId & path)
{
  const auto id_set = planning_utils::getLaneletIdSetOnPath<VirtualTrafficLight>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return id_set.count(scene_module->getModuleId()) == 0;
  };
}
}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::VirtualTrafficLightModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
