// Copyright 2024 The Autoware Contributors
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

#ifndef AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__PLUGIN_MODULE_INTERFACE_HPP_
#define AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__PLUGIN_MODULE_INTERFACE_HPP_

#include "planner_data.hpp"
#include "velocity_planning_result.hpp"

#include <autoware/motion_utils/factor/velocity_factor_interface.hpp>
#include <autoware/universe_utils/ros/processing_time_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{

class PluginModuleInterface
{
public:
  virtual ~PluginModuleInterface() = default;
  virtual void init(rclcpp::Node & node, const std::string & module_name) = 0;
  virtual void update_parameters(const std::vector<rclcpp::Parameter> & parameters) = 0;
  virtual VelocityPlanningResult plan(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data) = 0;
  virtual std::string get_module_name() const = 0;
  autoware::motion_utils::VelocityFactorInterface velocity_factor_interface_;
  rclcpp::Logger logger_ = rclcpp::get_logger("");
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr virtual_wall_publisher_;
  std::shared_ptr<autoware::universe_utils::ProcessingTimePublisher> processing_diag_publisher_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr processing_time_publisher_;
  autoware::motion_utils::VirtualWallMarkerCreator virtual_wall_marker_creator{};
};

}  // namespace autoware::motion_velocity_planner

#endif  // AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__PLUGIN_MODULE_INTERFACE_HPP_
