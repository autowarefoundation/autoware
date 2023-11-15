// Copyright 2023- Autoware Foundation
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

#ifndef TEST_MESSAGE_HELPER_NODE_HPP_
#define TEST_MESSAGE_HELPER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class TestMessageHelperNode : public rclcpp::Node
{
  using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
  using Odometry = nav_msgs::msg::Odometry;
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

public:
  TestMessageHelperNode() : Node("test_message_helper_node")
  {
    odometry_publisher_ =
      this->create_publisher<Odometry>("/pose_instability_detector/input/odometry", 10);
    twist_publisher_ = this->create_publisher<TwistWithCovarianceStamped>(
      "/pose_instability_detector/input/twist", 10);
    diagnostic_subscriber_ = this->create_subscription<DiagnosticArray>(
      "/diagnostics", 10,
      std::bind(&TestMessageHelperNode::callback_diagnostics, this, std::placeholders::_1));
  }

  void send_odometry_message(
    const builtin_interfaces::msg::Time timestamp, const double x, const double y, const double z)
  {
    Odometry message{};
    message.header.stamp = timestamp;
    message.pose.pose.position.x = x;
    message.pose.pose.position.y = y;
    message.pose.pose.position.z = z;
    odometry_publisher_->publish(message);
  }

  void send_twist_message(
    const builtin_interfaces::msg::Time timestamp, const double x, const double y, const double z)
  {
    TwistWithCovarianceStamped message{};
    message.header.stamp = timestamp;
    message.twist.twist.linear.x = x;
    message.twist.twist.linear.y = y;
    message.twist.twist.linear.z = z;
    twist_publisher_->publish(message);
  }

  void callback_diagnostics(const DiagnosticArray::ConstSharedPtr msg)
  {
    received_diagnostic_array = *msg;
    received_diagnostic_array_flag = true;
  }

  DiagnosticArray received_diagnostic_array;
  bool received_diagnostic_array_flag = false;

private:
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr twist_publisher_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostic_subscriber_;
};

#endif  // TEST_MESSAGE_HELPER_NODE_HPP_
