// Copyright 2022-2024 TIER IV, Inc.
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

#ifndef OBSTACLE_VELOCITY_LIMITER_MODULE_HPP_
#define OBSTACLE_VELOCITY_LIMITER_MODULE_HPP_

#include "parameters.hpp"

#include <autoware/motion_velocity_planner_common/plugin_module_interface.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware/universe_utils/ros/published_time_publisher.hpp>
#include <autoware/universe_utils/ros/self_pose_listener.hpp>
#include <autoware/universe_utils/ros/transform_listener.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner
{

class ObstacleVelocityLimiterModule : public PluginModuleInterface
{
public:
  void init(rclcpp::Node & node, const std::string & module_name) override;
  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) override;
  VelocityPlanningResult plan(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data) override;
  std::string get_module_name() const override { return module_name_; }

private:
  inline static const std::string ns_ = "obstacle_velocity_limiter";
  std::string module_name_;
  rclcpp::Clock::SharedPtr clock_;
  std::optional<geometry_msgs::msg::Point> prev_inserted_point_;

  // parameters
  obstacle_velocity_limiter::PreprocessingParameters preprocessing_params_;
  obstacle_velocity_limiter::ProjectionParameters projection_params_;
  obstacle_velocity_limiter::ObstacleParameters obstacle_params_;
  obstacle_velocity_limiter::VelocityParameters velocity_params_;
  double distance_buffer_{};
  double vehicle_lateral_offset_{};
  double vehicle_front_offset_{};
};
}  // namespace autoware::motion_velocity_planner

#endif  // OBSTACLE_VELOCITY_LIMITER_MODULE_HPP_
