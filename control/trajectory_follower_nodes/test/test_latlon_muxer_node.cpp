// Copyright 2021 The Autoware Foundation
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


#include <trajectory_follower_nodes/latlon_muxer_node.hpp>

#include <memory>
#include <string>
#include <vector>

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_control_msgs/msg/longitudinal_command.hpp"
#include "fake_test_node/fake_test_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "gtest/gtest.h"
#include "trajectory_follower_test_utils.hpp"


using LatLonMuxer = autoware::motion::control::trajectory_follower_nodes::LatLonMuxer;
using LateralCommand = autoware_auto_control_msgs::msg::AckermannLateralCommand;
using LongitudinalCommand = autoware_auto_control_msgs::msg::LongitudinalCommand;
using ControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

const rclcpp::Duration one_second(1, 0);

TEST_F(FakeNodeFixture, TestCorrectOutput)
{
  // Data to test
  ControlCommand::SharedPtr cmd_msg;
  bool received_combined_command = false;
  // Node
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("timeout_thr_sec", 0.5);
  std::shared_ptr<LatLonMuxer> node = std::make_shared<LatLonMuxer>(node_options);
  // Publisher/Subscribers
  rclcpp::Publisher<LateralCommand>::SharedPtr lat_pub =
    this->create_publisher<LateralCommand>(
    "latlon_muxer/input/lateral/control_cmd");
  rclcpp::Publisher<LongitudinalCommand>::SharedPtr lon_pub =
    this->create_publisher<LongitudinalCommand>(
    "latlon_muxer/input/longitudinal/control_cmd");
  rclcpp::Subscription<ControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<ControlCommand>(
    "latlon_muxer/output/control_cmd", *node,
    [&cmd_msg, &received_combined_command](const ControlCommand::SharedPtr msg) {
      cmd_msg = msg; received_combined_command = true;
    });
  // Publish messages
  LateralCommand lat_msg;
  lat_msg.steering_tire_angle = 1.5;
  lat_msg.steering_tire_rotation_rate = 0.2f;
  LongitudinalCommand lon_msg;
  lon_msg.speed = 5.0;
  lon_msg.acceleration = -1.0;
  lon_msg.jerk = 0.25;
  lat_msg.stamp = node->now();
  lon_msg.stamp = node->now();
  lat_pub->publish(lat_msg);
  lon_pub->publish(lon_msg);

  test_utils::waitForMessage(node, this, received_combined_command);
  // Ensure the combined control command was published and contains correct data
  ASSERT_TRUE(received_combined_command);
  EXPECT_EQ(cmd_msg->lateral.steering_tire_angle, lat_msg.steering_tire_angle);
  EXPECT_EQ(cmd_msg->lateral.steering_tire_rotation_rate, lat_msg.steering_tire_rotation_rate);
  EXPECT_EQ(cmd_msg->longitudinal.speed, lon_msg.speed);
  EXPECT_EQ(cmd_msg->longitudinal.acceleration, lon_msg.acceleration);
  EXPECT_EQ(cmd_msg->longitudinal.jerk, lon_msg.jerk);
  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(lat_msg.stamp));
  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(lon_msg.stamp));
}

TEST_F(FakeNodeFixture, TestLateralTimeout)
{
  // Data to test
  ControlCommand::SharedPtr cmd_msg;
  bool received_combined_command = false;
  // Node
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("timeout_thr_sec", 0.5);
  std::shared_ptr<LatLonMuxer> node = std::make_shared<LatLonMuxer>(node_options);
  // Publisher/Subscribers
  rclcpp::Publisher<LateralCommand>::SharedPtr lat_pub =
    this->create_publisher<LateralCommand>(
    "latlon_muxer/input/lateral/control_cmd");
  rclcpp::Publisher<LongitudinalCommand>::SharedPtr lon_pub =
    this->create_publisher<LongitudinalCommand>(
    "latlon_muxer/input/longitudinal/control_cmd");
  rclcpp::Subscription<ControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<ControlCommand>(
    "latlon_muxer/output/control_cmd", *node,
    [&cmd_msg, &received_combined_command](const ControlCommand::SharedPtr msg) {
      cmd_msg = msg; received_combined_command = true;
    });
  // Publish messages
  LateralCommand lat_msg;
  LongitudinalCommand lon_msg;
  // Generate a timeout of the lateral message
  lat_msg.stamp = node->now() - one_second;
  lon_msg.stamp = node->now();
  lat_pub->publish(lat_msg);
  lon_pub->publish(lon_msg);

  test_utils::waitForMessage(
    node, this, received_combined_command, std::chrono::seconds{1LL},
    false);
  // Ensure combined command was not published
  ASSERT_FALSE(received_combined_command);
}

TEST_F(FakeNodeFixture, TestLongitudinalTimeout)
{
  // Data to test
  ControlCommand::SharedPtr cmd_msg;
  bool received_combined_command = false;
  // Node
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("timeout_thr_sec", 0.5);
  std::shared_ptr<LatLonMuxer> node = std::make_shared<LatLonMuxer>(node_options);
  // Publisher/Subscribers
  rclcpp::Publisher<LateralCommand>::SharedPtr lat_pub =
    this->create_publisher<LateralCommand>(
    "latlon_muxer/input/lateral/control_cmd");
  rclcpp::Publisher<LongitudinalCommand>::SharedPtr lon_pub =
    this->create_publisher<LongitudinalCommand>(
    "latlon_muxer/input/longitudinal/control_cmd");
  rclcpp::Subscription<ControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<ControlCommand>(
    "latlon_muxer/output/control_cmd", *node,
    [&cmd_msg, &received_combined_command](const ControlCommand::SharedPtr msg) {
      cmd_msg = msg; received_combined_command = true;
    });
  // Publish messages
  LateralCommand lat_msg;
  LongitudinalCommand lon_msg;
  // Generate a timeout of the longitudinal message
  lat_msg.stamp = node->now();
  lon_msg.stamp = node->now() - one_second;
  lat_pub->publish(lat_msg);
  lon_pub->publish(lon_msg);

  test_utils::waitForMessage(
    node, this, received_combined_command, std::chrono::seconds{1LL},
    false);
  // Ensure combined command was not published
  ASSERT_FALSE(received_combined_command);
}

TEST_F(FakeNodeFixture, TestLatlonTimeout)
{
  // Data to test
  ControlCommand::SharedPtr cmd_msg;
  bool received_combined_command = false;
  // Node
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("timeout_thr_sec", 0.5);
  std::shared_ptr<LatLonMuxer> node = std::make_shared<LatLonMuxer>(node_options);
  // Publisher/Subscribers
  rclcpp::Publisher<LateralCommand>::SharedPtr lat_pub =
    this->create_publisher<LateralCommand>(
    "latlon_muxer/input/lateral/control_cmd");
  rclcpp::Publisher<LongitudinalCommand>::SharedPtr lon_pub =
    this->create_publisher<LongitudinalCommand>(
    "latlon_muxer/input/longitudinal/control_cmd");
  rclcpp::Subscription<ControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<ControlCommand>(
    "latlon_muxer/output/control_cmd", *node,
    [&cmd_msg, &received_combined_command](const ControlCommand::SharedPtr msg) {
      cmd_msg = msg; received_combined_command = true;
    });
  // Publish messages
  LateralCommand lat_msg;
  LongitudinalCommand lon_msg;
  // Generate a timeout of the longitudinal message
  lat_msg.stamp = node->now() - one_second;
  lon_msg.stamp = node->now() - one_second;
  lat_pub->publish(lat_msg);
  lon_pub->publish(lon_msg);

  test_utils::waitForMessage(
    node, this, received_combined_command, std::chrono::seconds{1LL},
    false);
  // Ensure combined command was not published
  ASSERT_FALSE(received_combined_command);
}
