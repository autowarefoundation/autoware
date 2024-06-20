// Copyright 2024 TIER IV
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

#include "autoware_test_utils/autoware_test_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <gtest/gtest.h>

class RelayNode : public rclcpp::Node
{
public:
  RelayNode() : Node("relay_node")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "input_topic", 10, [this](const std_msgs::msg::String::ConstSharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->data.c_str());
        auto new_msg = std::make_shared<std_msgs::msg::String>();
        new_msg->data = msg->data;
        publisher_->publish(*new_msg);
      });

    publisher_ = this->create_publisher<std_msgs::msg::String>("output_topic", 10);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

TEST(AutowareTestManagerTest, TestRelayNode)
{
  rclcpp::init(0, nullptr);

  const std::string input_topic_name = "input_topic";
  const std::string output_topic_name = "output_topic";

  // Setup target node and its test manager
  auto target_node = std::make_shared<RelayNode>();
  auto manager = std::make_shared<autoware::test_utils::AutowareTestManager>();

  // Setup subscriber for test manager
  std::string received_msg;
  manager->set_subscriber<std_msgs::msg::String>(
    "output_topic",
    [&received_msg](const std_msgs::msg::String::ConstSharedPtr msg) { received_msg = msg->data; });

  // Publish a message to the relay node
  std_msgs::msg::String msg;
  msg.data = "Hello, Relay!";
  manager->test_pub_msg<std_msgs::msg::String>(target_node, input_topic_name, msg);

  // Spin to process callbacks
  rclcpp::spin_some(target_node);

  // Check that the message was relayed and received by the test node
  EXPECT_EQ(received_msg, "Hello, Relay!");
}
