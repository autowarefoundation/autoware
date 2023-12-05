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

#include "planner_manager.hpp"

#include <boost/format.hpp>

#include <memory>
#include <string>

namespace behavior_velocity_planner
{
namespace
{
std::string jsonDumpsPose(const geometry_msgs::msg::Pose & pose)
{
  const std::string json_dumps_pose =
    (boost::format(
       R"({"position":{"x":%lf,"y":%lf,"z":%lf},"orientation":{"w":%lf,"x":%lf,"y":%lf,"z":%lf}})") %
     pose.position.x % pose.position.y % pose.position.z % pose.orientation.w % pose.orientation.x %
     pose.orientation.y % pose.orientation.z)
      .str();
  return json_dumps_pose;
}

diagnostic_msgs::msg::DiagnosticStatus makeStopReasonDiag(
  const std::string stop_reason, const geometry_msgs::msg::Pose & stop_pose)
{
  diagnostic_msgs::msg::DiagnosticStatus stop_reason_diag;
  diagnostic_msgs::msg::KeyValue stop_reason_diag_kv;
  stop_reason_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stop_reason_diag.name = "stop_reason";
  stop_reason_diag.message = stop_reason;
  stop_reason_diag_kv.key = "stop_pose";
  stop_reason_diag_kv.value = jsonDumpsPose(stop_pose);
  stop_reason_diag.values.push_back(stop_reason_diag_kv);
  return stop_reason_diag;
}
}  // namespace

BehaviorVelocityPlannerManager::BehaviorVelocityPlannerManager()
: plugin_loader_("behavior_velocity_planner", "behavior_velocity_planner::PluginInterface")
{
}

void BehaviorVelocityPlannerManager::launchScenePlugin(
  rclcpp::Node & node, const std::string & name)
{
  if (plugin_loader_.isClassAvailable(name)) {
    const auto plugin = plugin_loader_.createSharedInstance(name);
    plugin->init(node);

    // Check if the plugin is already registered.
    for (const auto & running_plugin : scene_manager_plugins_) {
      if (plugin->getModuleName() == running_plugin->getModuleName()) {
        RCLCPP_WARN_STREAM(node.get_logger(), "The plugin '" << name << "' is already loaded.");
        return;
      }
    }

    // register
    scene_manager_plugins_.push_back(plugin);
    RCLCPP_INFO_STREAM(node.get_logger(), "The scene plugin '" << name << "' is loaded.");
  } else {
    RCLCPP_ERROR_STREAM(node.get_logger(), "The scene plugin '" << name << "' is not available.");
  }
}

void BehaviorVelocityPlannerManager::removeScenePlugin(
  rclcpp::Node & node, const std::string & name)
{
  auto it = std::remove_if(
    scene_manager_plugins_.begin(), scene_manager_plugins_.end(),
    [&](const std::shared_ptr<behavior_velocity_planner::PluginInterface> plugin) {
      return plugin->getModuleName() == name;
    });

  if (it == scene_manager_plugins_.end()) {
    RCLCPP_WARN_STREAM(
      node.get_logger(),
      "The scene plugin '" << name << "' is not found in the registered modules.");
  } else {
    scene_manager_plugins_.erase(it, scene_manager_plugins_.end());
    RCLCPP_INFO_STREAM(node.get_logger(), "The scene plugin '" << name << "' is unloaded.");
  }
}

autoware_auto_planning_msgs::msg::PathWithLaneId BehaviorVelocityPlannerManager::planPathVelocity(
  const std::shared_ptr<const PlannerData> & planner_data,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path_msg)
{
  autoware_auto_planning_msgs::msg::PathWithLaneId output_path_msg = input_path_msg;

  int first_stop_path_point_index = static_cast<int>(output_path_msg.points.size() - 1);
  std::string stop_reason_msg("path_end");

  for (const auto & plugin : scene_manager_plugins_) {
    plugin->updateSceneModuleInstances(planner_data, input_path_msg);
    plugin->plan(&output_path_msg);
    const auto firstStopPathPointIndex = plugin->getFirstStopPathPointIndex();

    if (firstStopPathPointIndex) {
      if (firstStopPathPointIndex.value() < first_stop_path_point_index) {
        first_stop_path_point_index = firstStopPathPointIndex.value();
        stop_reason_msg = plugin->getModuleName();
      }
    }
  }

  stop_reason_diag_ = makeStopReasonDiag(
    stop_reason_msg, output_path_msg.points[first_stop_path_point_index].point.pose);

  return output_path_msg;
}

diagnostic_msgs::msg::DiagnosticStatus BehaviorVelocityPlannerManager::getStopReasonDiag() const
{
  return stop_reason_diag_;
}
}  // namespace behavior_velocity_planner
