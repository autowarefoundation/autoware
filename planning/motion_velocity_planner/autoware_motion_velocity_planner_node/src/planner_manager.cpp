// Copyright 2024 Tier IV, Inc.
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

#include "planner_manager.hpp"

#include <boost/format.hpp>

#include <memory>
#include <string>

namespace autoware::motion_velocity_planner
{

MotionVelocityPlannerManager::MotionVelocityPlannerManager()
: plugin_loader_(
    "autoware_motion_velocity_planner_node",
    "autoware::motion_velocity_planner::PluginModuleInterface")
{
}

void MotionVelocityPlannerManager::load_module_plugin(rclcpp::Node & node, const std::string & name)
{
  // Check if the plugin is already loaded.
  if (plugin_loader_.isClassLoaded(name)) {
    RCLCPP_WARN_STREAM(node.get_logger(), "The plugin '" << name << "' is already loaded.");
    return;
  }
  if (plugin_loader_.isClassAvailable(name)) {
    const auto plugin = plugin_loader_.createSharedInstance(name);
    plugin->init(node, name);

    // register
    loaded_plugins_.push_back(plugin);
    RCLCPP_DEBUG_STREAM(node.get_logger(), "The scene plugin '" << name << "' is loaded.");
  } else {
    RCLCPP_ERROR_STREAM(node.get_logger(), "The scene plugin '" << name << "' is not available.");
  }
}

void MotionVelocityPlannerManager::unload_module_plugin(
  rclcpp::Node & node, const std::string & name)
{
  auto it = std::remove_if(loaded_plugins_.begin(), loaded_plugins_.end(), [&](const auto plugin) {
    return plugin->get_module_name() == name;
  });

  if (it == loaded_plugins_.end()) {
    RCLCPP_WARN_STREAM(
      node.get_logger(),
      "The scene plugin '" << name << "' is not found in the registered modules.");
  } else {
    loaded_plugins_.erase(it, loaded_plugins_.end());
    RCLCPP_INFO_STREAM(node.get_logger(), "The scene plugin '" << name << "' is unloaded.");
  }
}

void MotionVelocityPlannerManager::update_module_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (auto & plugin : loaded_plugins_) plugin->update_parameters(parameters);
}

std::vector<VelocityPlanningResult> MotionVelocityPlannerManager::plan_velocities(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  std::vector<VelocityPlanningResult> results;
  for (auto & plugin : loaded_plugins_)
    results.push_back(plugin->plan(ego_trajectory_points, planner_data));
  return results;
}
}  // namespace autoware::motion_velocity_planner
