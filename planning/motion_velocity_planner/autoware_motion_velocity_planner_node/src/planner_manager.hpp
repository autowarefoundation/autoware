// Copyright 2024 Autoware Foundation
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

#ifndef PLANNER_MANAGER_HPP_
#define PLANNER_MANAGER_HPP_

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{
class MotionVelocityPlannerManager
{
public:
  MotionVelocityPlannerManager();
  void load_module_plugin(rclcpp::Node & node, const std::string & name);
  void unload_module_plugin(rclcpp::Node & node, const std::string & name);
  void update_module_parameters(const std::vector<rclcpp::Parameter> & parameters);
  std::vector<VelocityPlanningResult> plan_velocities(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data);

private:
  pluginlib::ClassLoader<PluginModuleInterface> plugin_loader_;
  std::vector<std::shared_ptr<PluginModuleInterface>> loaded_plugins_;
};
}  // namespace autoware::motion_velocity_planner

#endif  // PLANNER_MANAGER_HPP_
