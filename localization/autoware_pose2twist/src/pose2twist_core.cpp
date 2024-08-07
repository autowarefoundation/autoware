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

#include "pose2twist_core.hpp"

#include <cmath>
#include <cstddef>
#include <functional>

namespace autoware::pose2twist
{
Pose2Twist::Pose2Twist(const rclcpp::NodeOptions & options) : rclcpp::Node("pose2twist", options)
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
    "pose", queue_size, std::bind(&Pose2Twist::callback_pose, this, _1));
}

tf2::Quaternion get_quaternion(const geometry_msgs::msg::PoseStamped::SharedPtr & pose_stamped_ptr)
{
  const auto & orientation = pose_stamped_ptr->pose.orientation;
  return tf2::Quaternion{orientation.x, orientation.y, orientation.z, orientation.w};
}

geometry_msgs::msg::Vector3 compute_relative_rotation_vector(
  const tf2::Quaternion & q1, const tf2::Quaternion & q2)
{
  // If we define q2 as the rotation obtained by applying dq after applying q1,
  // then q2 = q1 * dq .
  // Therefore, dq = q1.inverse() * q2 .
  const tf2::Quaternion diff_quaternion = q1.inverse() * q2;
  const tf2::Vector3 axis = diff_quaternion.getAxis() * diff_quaternion.getAngle();
  return geometry_msgs::msg::Vector3{}.set__x(axis.x()).set__y(axis.y()).set__z(axis.z());
}

geometry_msgs::msg::TwistStamped calc_twist(
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

  const auto pose_a_quaternion = get_quaternion(pose_a);
  const auto pose_b_quaternion = get_quaternion(pose_b);

  geometry_msgs::msg::Vector3 diff_xyz;
  const geometry_msgs::msg::Vector3 relative_rotation_vector =
    compute_relative_rotation_vector(pose_a_quaternion, pose_b_quaternion);

  diff_xyz.x = pose_b->pose.position.x - pose_a->pose.position.x;
  diff_xyz.y = pose_b->pose.position.y - pose_a->pose.position.y;
  diff_xyz.z = pose_b->pose.position.z - pose_a->pose.position.z;

  geometry_msgs::msg::TwistStamped twist;
  twist.header = pose_b->header;
  twist.twist.linear.x =
    std::sqrt(std::pow(diff_xyz.x, 2.0) + std::pow(diff_xyz.y, 2.0) + std::pow(diff_xyz.z, 2.0)) /
    dt;
  twist.twist.linear.y = 0;
  twist.twist.linear.z = 0;
  twist.twist.angular.x = relative_rotation_vector.x / dt;
  twist.twist.angular.y = relative_rotation_vector.y / dt;
  twist.twist.angular.z = relative_rotation_vector.z / dt;

  return twist;
}

void Pose2Twist::callback_pose(geometry_msgs::msg::PoseStamped::SharedPtr pose_msg_ptr)
{
  // TODO(YamatoAndo) check time stamp diff
  // TODO(YamatoAndo) check suddenly move
  // TODO(YamatoAndo) apply low pass filter

  const geometry_msgs::msg::PoseStamped::SharedPtr & current_pose_msg = pose_msg_ptr;
  static geometry_msgs::msg::PoseStamped::SharedPtr prev_pose_msg = current_pose_msg;
  geometry_msgs::msg::TwistStamped twist_msg = calc_twist(prev_pose_msg, current_pose_msg);
  prev_pose_msg = current_pose_msg;
  twist_msg.header.frame_id = "base_link";
  twist_pub_->publish(twist_msg);

  tier4_debug_msgs::msg::Float32Stamped linear_x_msg;
  linear_x_msg.stamp = this->now();
  linear_x_msg.data = static_cast<float>(twist_msg.twist.linear.x);
  linear_x_pub_->publish(linear_x_msg);

  tier4_debug_msgs::msg::Float32Stamped angular_z_msg;
  angular_z_msg.stamp = this->now();
  angular_z_msg.data = static_cast<float>(twist_msg.twist.angular.z);
  angular_z_pub_->publish(angular_z_msg);
}
}  // namespace autoware::pose2twist

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pose2twist::Pose2Twist)
