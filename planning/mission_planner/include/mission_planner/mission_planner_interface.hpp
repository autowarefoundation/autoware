// Copyright 2019 Autoware Foundation
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

#ifndef MISSION_PLANNER__MISSION_PLANNER_INTERFACE_HPP_
#define MISSION_PLANNER__MISSION_PLANNER_INTERFACE_HPP_

#include <boost/optional.hpp>

#include <string>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Autoware
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace mission_planner
{
class MissionPlannerInterface : public rclcpp::Node
{
protected:
  MissionPlannerInterface(const std::string & node_name, const rclcpp::NodeOptions & node_options);

  geometry_msgs::msg::Pose::ConstSharedPtr ego_pose_;
  std::vector<geometry_msgs::msg::Pose> check_points_;

  std::string map_frame_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  virtual bool is_routing_graph_ready() const = 0;
  virtual autoware_auto_planning_msgs::msg::HADMapRoute plan_route(
    const std::vector<geometry_msgs::msg::Pose> & check_points) = 0;
  virtual void visualize_route(
    const autoware_auto_planning_msgs::msg::HADMapRoute & route) const = 0;
  virtual void publish_route(const autoware_auto_planning_msgs::msg::HADMapRoute & route) const;

private:
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::HADMapRoute>::SharedPtr route_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr check_point_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_msg_ptr);
  void check_point_callback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr check_point_msg_ptr);
  boost::optional<geometry_msgs::msg::PoseStamped> transform_pose(
    const geometry_msgs::msg::PoseStamped & input_pose, const std::string & target_frame);
};

}  // namespace mission_planner
#endif  // MISSION_PLANNER__MISSION_PLANNER_INTERFACE_HPP_
