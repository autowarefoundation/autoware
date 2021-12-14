// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#ifndef OBSTACLE_COLLISION_CHECKER__OBSTACLE_COLLISION_CHECKER_NODE_HPP_
#define OBSTACLE_COLLISION_CHECKER__OBSTACLE_COLLISION_CHECKER_NODE_HPP_

#include "obstacle_collision_checker/obstacle_collision_checker.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <tier4_autoware_utils/ros/processing_time_publisher.hpp>
#include <tier4_autoware_utils/ros/self_pose_listener.hpp>
#include <tier4_autoware_utils/ros/transform_listener.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <vector>

namespace obstacle_collision_checker
{
struct NodeParam
{
  double update_rate;
};

class ObstacleCollisionCheckerNode : public rclcpp::Node
{
public:
  explicit ObstacleCollisionCheckerNode(const rclcpp::NodeOptions & node_options);

private:
  // Subscriber
  std::shared_ptr<tier4_autoware_utils::SelfPoseListener> self_pose_listener_;
  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_obstacle_pointcloud_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    sub_reference_trajectory_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    sub_predicted_trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  // Data Buffer
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;
  geometry_msgs::msg::Twist::ConstSharedPtr current_twist_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr obstacle_pointcloud_;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr obstacle_transform_;
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr reference_trajectory_;
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_trajectory_;

  // Callback
  void onObstaclePointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onReferenceTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);
  void onPredictedTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Publisher
  std::shared_ptr<tier4_autoware_utils::DebugPublisher> debug_publisher_;
  std::shared_ptr<tier4_autoware_utils::ProcessingTimePublisher> time_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void initTimer(double period_s);

  bool isDataReady();
  bool isDataTimeout();
  void onTimer();

  // Parameter
  NodeParam node_param_;
  Param param_;

  // Dynamic Reconfigure
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // Core
  Input input_;
  Output output_;
  std::unique_ptr<ObstacleCollisionChecker> obstacle_collision_checker_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  void checkLaneDeparture(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Visualization
  visualization_msgs::msg::MarkerArray createMarkerArray() const;
};
}  // namespace obstacle_collision_checker

#endif  // OBSTACLE_COLLISION_CHECKER__OBSTACLE_COLLISION_CHECKER_NODE_HPP_
