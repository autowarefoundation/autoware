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

#include "surround_obstacle_checker/debug_marker.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>

SurroundObstacleCheckerDebugNode::SurroundObstacleCheckerDebugNode(
  const double base_link2front, const rclcpp::Clock::SharedPtr clock, rclcpp::Node & node)
: base_link2front_(base_link2front), clock_(clock)
{
  debug_viz_pub_ = node.create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/marker", 1);
  stop_reason_pub_ =
    node.create_publisher<autoware_planning_msgs::msg::StopReasonArray>("~/output/stop_reasons", 1);
}

bool SurroundObstacleCheckerDebugNode::pushPose(
  const geometry_msgs::msg::Pose & pose, const PoseType & type)
{
  switch (type) {
    case PoseType::NoStart:
      stop_pose_ptr_ = std::make_shared<geometry_msgs::msg::Pose>(pose);
      return true;
    default:
      return false;
  }
}

bool SurroundObstacleCheckerDebugNode::pushObstaclePoint(
  const geometry_msgs::msg::Point & obstacle_point, const PointType & type)
{
  switch (type) {
    case PointType::NoStart:
      stop_obstacle_point_ptr_ = std::make_shared<geometry_msgs::msg::Point>(obstacle_point);
      return true;
    default:
      return false;
  }
}

void SurroundObstacleCheckerDebugNode::publish()
{
  /* publish debug marker for rviz */
  const auto visualization_msg = makeVisualizationMarker();
  debug_viz_pub_->publish(visualization_msg);

  /* publish stop reason for autoware api */
  const auto stop_reason_msg = makeStopReasonArray();
  stop_reason_pub_->publish(stop_reason_msg);

  /* reset variables */
  stop_pose_ptr_ = nullptr;
  stop_obstacle_point_ptr_ = nullptr;
}

visualization_msgs::msg::MarkerArray SurroundObstacleCheckerDebugNode::makeVisualizationMarker()
{
  visualization_msgs::msg::MarkerArray msg;
  rclcpp::Time current_time = this->clock_->now();
  tf2::Transform tf_base_link2front(
    tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(base_link2front_, 0.0, 0.0));

  // visualize stop line
  if (stop_pose_ptr_ != nullptr) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "virtual_wall/no_start";
    marker.id = 0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(*stop_pose_ptr_, tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 5.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    msg.markers.push_back(marker);
  }

  // visualize stop reason
  if (stop_pose_ptr_ != nullptr) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "factor_text/no_start";
    marker.id = 0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(*stop_pose_ptr_, tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 2.0;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = "surround obstacle";
    msg.markers.push_back(marker);
  }

  // visualize surround object
  if (stop_obstacle_point_ptr_ != nullptr) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "no_start_obstacle_text";
    marker.id = 0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = *stop_obstacle_point_ptr_;
    marker.pose.position.z += 2.0;  // add half of the heights of obj roughly
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = "!";
    msg.markers.push_back(marker);
  }

  return msg;
}

autoware_planning_msgs::msg::StopReasonArray SurroundObstacleCheckerDebugNode::makeStopReasonArray()
{
  // create header
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = this->clock_->now();

  // create stop reason stamped
  autoware_planning_msgs::msg::StopReason stop_reason_msg;
  stop_reason_msg.reason = autoware_planning_msgs::msg::StopReason::SURROUND_OBSTACLE_CHECK;
  autoware_planning_msgs::msg::StopFactor stop_factor;

  if (stop_pose_ptr_ != nullptr) {
    stop_factor.stop_pose = *stop_pose_ptr_;
    if (stop_obstacle_point_ptr_ != nullptr) {
      stop_factor.stop_factor_points.emplace_back(*stop_obstacle_point_ptr_);
    }
    stop_reason_msg.stop_factors.emplace_back(stop_factor);
  }

  // create stop reason array
  autoware_planning_msgs::msg::StopReasonArray stop_reason_array;
  stop_reason_array.header = header;
  stop_reason_array.stop_reasons.emplace_back(stop_reason_msg);
  return stop_reason_array;
}
