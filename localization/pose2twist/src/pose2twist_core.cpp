// Copyright 2015-2019 Autoware Foundation
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

#include "pose2twist/pose2twist_core.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <cmath>
#include <cstddef>
#include <functional>

Pose2Twist::Pose2Twist() : Node("pose2twist_core")
{
  using std::placeholders::_1;

  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);
  durable_qos.transient_local();

  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist", durable_qos);
  linear_x_pub_ = create_publisher<tier4_debug_msgs::msg::Float32Stamped>("linear_x", durable_qos);
  angular_z_pub_ =
    create_publisher<tier4_debug_msgs::msg::Float32Stamped>("angular_z", durable_qos);
  // Note: this callback publishes topics above
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "pose", queue_size, std::bind(&Pose2Twist::callbackPose, this, _1));
}

double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad > M_PI) {
    diff_rad = diff_rad - 2 * M_PI;
  } else if (diff_rad < -M_PI) {
    diff_rad = diff_rad + 2 * M_PI;
  }
  return diff_rad;
}

// x: roll, y: pitch, z: yaw
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Vector3 rpy;
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

geometry_msgs::msg::Vector3 getRPY(geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  return getRPY(pose->pose);
}

geometry_msgs::msg::TwistStamped calcTwist(
  geometry_msgs::msg::PoseStamped::SharedPtr pose_a,
  geometry_msgs::msg::PoseStamped::SharedPtr pose_b)
{
  const double dt =
    (rclcpp::Time(pose_b->header.stamp) - rclcpp::Time(pose_a->header.stamp)).seconds();

  if (dt == 0) {
    geometry_msgs::msg::TwistStamped twist;
    twist.header = pose_b->header;
    return twist;
  }

  const auto pose_a_rpy = getRPY(pose_a);
  const auto pose_b_rpy = getRPY(pose_b);

  geometry_msgs::msg::Vector3 diff_xyz;
  geometry_msgs::msg::Vector3 diff_rpy;

  diff_xyz.x = pose_b->pose.position.x - pose_a->pose.position.x;
  diff_xyz.y = pose_b->pose.position.y - pose_a->pose.position.y;
  diff_xyz.z = pose_b->pose.position.z - pose_a->pose.position.z;
  diff_rpy.x = calcDiffForRadian(pose_b_rpy.x, pose_a_rpy.x);
  diff_rpy.y = calcDiffForRadian(pose_b_rpy.y, pose_a_rpy.y);
  diff_rpy.z = calcDiffForRadian(pose_b_rpy.z, pose_a_rpy.z);

  geometry_msgs::msg::TwistStamped twist;
  twist.header = pose_b->header;
  twist.twist.linear.x =
    std::sqrt(std::pow(diff_xyz.x, 2.0) + std::pow(diff_xyz.y, 2.0) + std::pow(diff_xyz.z, 2.0)) /
    dt;
  twist.twist.linear.y = 0;
  twist.twist.linear.z = 0;
  twist.twist.angular.x = diff_rpy.x / dt;
  twist.twist.angular.y = diff_rpy.y / dt;
  twist.twist.angular.z = diff_rpy.z / dt;

  return twist;
}

void Pose2Twist::callbackPose(geometry_msgs::msg::PoseStamped::SharedPtr pose_msg_ptr)
{
  // TODO(YamatoAndo) check time stamp diff
  // TODO(YamatoAndo) check suddenly move
  // TODO(YamatoAndo) apply low pass filter

  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_msg = pose_msg_ptr;
  static geometry_msgs::msg::PoseStamped::SharedPtr prev_pose_msg = current_pose_msg;
  geometry_msgs::msg::TwistStamped twist_msg = calcTwist(prev_pose_msg, current_pose_msg);
  prev_pose_msg = current_pose_msg;
  twist_msg.header.frame_id = "base_link";
  twist_pub_->publish(twist_msg);

  tier4_debug_msgs::msg::Float32Stamped linear_x_msg;
  linear_x_msg.stamp = this->now();
  linear_x_msg.data = twist_msg.twist.linear.x;
  linear_x_pub_->publish(linear_x_msg);

  tier4_debug_msgs::msg::Float32Stamped angular_z_msg;
  angular_z_msg.stamp = this->now();
  angular_z_msg.data = twist_msg.twist.angular.z;
  angular_z_pub_->publish(angular_z_msg);
}
