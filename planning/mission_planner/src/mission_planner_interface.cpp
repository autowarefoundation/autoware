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

#include "mission_planner/mission_planner_interface.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>

#include <lanelet2_routing/Route.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <visualization_msgs/msg/marker_array.h>

#include <memory>
#include <string>

namespace mission_planner
{
MissionPlannerInterface::MissionPlannerInterface(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  map_frame_ = declare_parameter("map_frame", "map");

  using std::placeholders::_1;

  odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{1},
    std::bind(&MissionPlannerInterface::odometry_callback, this, _1));
  goal_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "input/goal_pose", 10, std::bind(&MissionPlannerInterface::goal_pose_callback, this, _1));
  check_point_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "input/checkpoint", 10, std::bind(&MissionPlannerInterface::check_point_callback, this, _1));

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  route_publisher_ =
    create_publisher<autoware_auto_planning_msgs::msg::HADMapRoute>("output/route", durable_qos);
  marker_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("debug/route_marker", durable_qos);
}

void MissionPlannerInterface::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  ego_pose_ = std::make_shared<const geometry_msgs::msg::Pose>(msg->pose.pose);
}

boost::optional<geometry_msgs::msg::PoseStamped> MissionPlannerInterface::transform_pose(
  const geometry_msgs::msg::PoseStamped & input_pose, const std::string & target_frame)
{
  try {
    geometry_msgs::msg::PoseStamped output_pose;
    const auto transform =
      tf_buffer_.lookupTransform(target_frame, input_pose.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(input_pose, output_pose, transform);
    return output_pose;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
  }

  return {};
}

void MissionPlannerInterface::goal_pose_callback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_msg_ptr)
{
  // set start pose
  if (!ego_pose_) {
    RCLCPP_ERROR(get_logger(), "Ego pose has not been subscribed. Aborting mission planning");
    return;
  }

  // set goal pose
  const auto opt_transformed_goal_pose = transform_pose(*goal_msg_ptr, map_frame_);
  if (!opt_transformed_goal_pose) {
    RCLCPP_ERROR(get_logger(), "Failed to get goal pose in map frame. Aborting mission planning");
    return;
  }
  const auto transformed_goal_pose = opt_transformed_goal_pose.get();

  RCLCPP_INFO(get_logger(), "New goal pose is set. Reset check_points.");
  check_points_.clear();
  check_points_.push_back(*ego_pose_);
  check_points_.push_back(transformed_goal_pose.pose);

  if (!is_routing_graph_ready()) {
    RCLCPP_ERROR(get_logger(), "RoutingGraph is not ready. Aborting mission planning");
    return;
  }

  const auto route = plan_route(check_points_);
  publish_route(route);
}  // namespace mission_planner

void MissionPlannerInterface::check_point_callback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr check_point_msg_ptr)
{
  if (check_points_.size() < 2) {
    RCLCPP_ERROR(
      get_logger(),
      "You must set start and goal before setting check_points. Aborting mission planning");
    return;
  }

  const auto opt_transformed_check_point = transform_pose(*check_point_msg_ptr, map_frame_);
  if (!opt_transformed_check_point) {
    RCLCPP_ERROR(
      get_logger(), "Failed to get check_point pose in map frame. Aborting mission planning");
    return;
  }
  const auto transformed_check_point = opt_transformed_check_point.get();

  // insert check_point before goal
  check_points_.insert(check_points_.end() - 1, transformed_check_point.pose);

  const auto route = plan_route(check_points_);
  publish_route(route);
}

void MissionPlannerInterface::publish_route(
  const autoware_auto_planning_msgs::msg::HADMapRoute & route) const
{
  if (route.segments.empty()) {
    RCLCPP_ERROR(get_logger(), "Calculated route is empty!");
    return;
  }

  RCLCPP_INFO(get_logger(), "Route successfully planned. Publishing...");
  route_publisher_->publish(route);
  visualize_route(route);
}

}  // namespace mission_planner
