// Copyright 2022 TIER IV, Inc.
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

#ifndef OBSTACLE_VELOCITY_LIMITER__OBSTACLE_VELOCITY_LIMITER_NODE_HPP_
#define OBSTACLE_VELOCITY_LIMITER__OBSTACLE_VELOCITY_LIMITER_NODE_HPP_

#include "obstacle_velocity_limiter/obstacles.hpp"
#include "obstacle_velocity_limiter/parameters.hpp"
#include "obstacle_velocity_limiter/types.hpp"
#include "tier4_autoware_utils/ros/logger_level_configure.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/self_pose_listener.hpp>
#include <tier4_autoware_utils/ros/transform_listener.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace obstacle_velocity_limiter
{

class ObstacleVelocityLimiterNode : public rclcpp::Node
{
public:
  explicit ObstacleVelocityLimiterNode(const rclcpp::NodeOptions & node_options);

private:
  tier4_autoware_utils::TransformListener transform_listener_{this};
  rclcpp::Publisher<Trajectory>::SharedPtr
    pub_trajectory_;  //!< @brief publisher for output trajectory
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pub_debug_markers_;  //!< @brief publisher for debug markers
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr
    pub_runtime_;  //!< @brief publisher for callback runtime
  rclcpp::Subscription<Trajectory>::SharedPtr
    sub_trajectory_;  //!< @brief subscriber for reference trajectory
  rclcpp::Subscription<PredictedObjects>::SharedPtr
    sub_objects_;  //!< @brief subscribe for dynamic objects
  rclcpp::Subscription<OccupancyGrid>::SharedPtr
    sub_occupancy_grid_;  //!< @brief subscriber for occupancy grid
  rclcpp::Subscription<PointCloud>::SharedPtr
    sub_pointcloud_;  //!< @brief subscriber for obstacle pointcloud
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    sub_odom_;  //!< @brief subscriber for the current velocity
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;

  // cached inputs
  PredictedObjects::ConstSharedPtr dynamic_obstacles_ptr_;
  OccupancyGrid::ConstSharedPtr occupancy_grid_ptr_;
  PointCloud::ConstSharedPtr pointcloud_ptr_;
  lanelet::LaneletMapPtr lanelet_map_ptr_{new lanelet::LaneletMap};
  multi_linestring_t static_map_obstacles_;
  nav_msgs::msg::Odometry::ConstSharedPtr current_odometry_ptr_;

  // parameters
  PreprocessingParameters preprocessing_params_;
  ProjectionParameters projection_params_;
  ObstacleParameters obstacle_params_;
  VelocityParameters velocity_params_;
  Float distance_buffer_ = static_cast<Float>(declare_parameter<Float>("distance_buffer"));
  Float vehicle_lateral_offset_;
  Float vehicle_front_offset_;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /// @brief callback for parameter updates
  /// @param[in] parameters updated parameters and their new values
  /// @return result of parameter update
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  /// @brief callback for input trajectories. Publishes a trajectory with updated velocities
  /// @param[in] msg input trajectory message
  void onTrajectory(const Trajectory::ConstSharedPtr msg);

  /// @brief validate the inputs of the node
  /// @return true if the inputs are valid
  bool validInputs();

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;
};
}  // namespace obstacle_velocity_limiter

#endif  // OBSTACLE_VELOCITY_LIMITER__OBSTACLE_VELOCITY_LIMITER_NODE_HPP_
