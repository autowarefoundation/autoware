// Copyright 2023 Tier IV, Inc.
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

#include "diagnostic_converter/converter_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "tier4_simulation_msgs/msg/user_defined_value.hpp"
#include "tier4_simulation_msgs/msg/user_defined_value_type.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using ConverterNode = diagnostic_converter::DiagnosticConverter;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using tier4_simulation_msgs::msg::UserDefinedValue;

void waitForMsg(
  bool & flag, const rclcpp::Node::SharedPtr node1, const rclcpp::Node::SharedPtr node2)
{
  while (!flag) {
    rclcpp::spin_some(node1);
    rclcpp::spin_some(node2);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  flag = false;
}

std::function<void(UserDefinedValue::ConstSharedPtr)> generateCallback(
  bool & flag, UserDefinedValue & msg)
{
  return [&](UserDefinedValue::ConstSharedPtr received_msg) {
    flag = true;
    msg = *received_msg;
  };
}

TEST(ConverterNode, ConvertDiagnostics)
{
  const std::vector<std::string> input_topics = {"/test1/diag", "/test2/diag"};

  rclcpp::init(0, nullptr);
  rclcpp::Node::SharedPtr dummy_node = std::make_shared<rclcpp::Node>("converter_test_node");

  rclcpp::NodeOptions options;
  options.append_parameter_override("diagnostic_topics", rclcpp::ParameterValue(input_topics));
  auto node = std::make_shared<ConverterNode>(options);

  {  // Simple case with 1 resulting UserDefinedValue
    bool msg_received = false;
    UserDefinedValue param;
    // DiagnosticArray publishers
    const auto diag_pub = dummy_node->create_publisher<DiagnosticArray>(input_topics[0], 1);
    // UserDefinedValue subscribers
    const auto param_sub_a = dummy_node->create_subscription<UserDefinedValue>(
      input_topics[0] + "_a", 1, generateCallback(msg_received, param));
    DiagnosticArray diag;
    DiagnosticStatus status;
    status.name = "";
    KeyValue key_value = KeyValue().set__key("a").set__value("1");
    status.values.push_back(key_value);
    diag.status.push_back(status);
    diag_pub->publish(diag);
    waitForMsg(msg_received, node, dummy_node);
    EXPECT_EQ(param.value, "1");
  }
  {  // Case with multiple UserDefinedValue converted from one DiagnosticArray
    bool msg_received_xa = false;
    bool msg_received_xb = false;
    bool msg_received_ya = false;
    bool msg_received_yb = false;
    UserDefinedValue param_xa;
    UserDefinedValue param_xb;
    UserDefinedValue param_ya;
    UserDefinedValue param_yb;
    // DiagnosticArray publishers
    const auto diag_pub = dummy_node->create_publisher<DiagnosticArray>(input_topics[1], 1);
    // UserDefinedValue subscribers
    const auto param_sub_xa = dummy_node->create_subscription<UserDefinedValue>(
      input_topics[1] + "_x_a", 1, generateCallback(msg_received_xa, param_xa));
    const auto param_sub_xb = dummy_node->create_subscription<UserDefinedValue>(
      input_topics[1] + "_x_b", 1, generateCallback(msg_received_xb, param_xb));
    const auto param_sub_ya = dummy_node->create_subscription<UserDefinedValue>(
      input_topics[1] + "_y_a", 1, generateCallback(msg_received_ya, param_ya));
    const auto param_sub_yb = dummy_node->create_subscription<UserDefinedValue>(
      input_topics[1] + "_y_b", 1, generateCallback(msg_received_yb, param_yb));
    DiagnosticArray diag;
    DiagnosticStatus status_x;
    status_x.name = "x";
    status_x.values.push_back(KeyValue().set__key("a").set__value("1"));
    status_x.values.push_back(KeyValue().set__key("b").set__value("10"));
    diag.status.push_back(status_x);
    DiagnosticStatus status_y;
    status_y.name = "y";
    status_y.values.push_back(KeyValue().set__key("a").set__value("9"));
    status_y.values.push_back(KeyValue().set__key("b").set__value("6"));
    diag.status.push_back(status_y);
    diag_pub->publish(diag);
    waitForMsg(msg_received_xa, node, dummy_node);
    EXPECT_EQ(param_xa.value, "1");
    waitForMsg(msg_received_xb, node, dummy_node);
    EXPECT_EQ(param_xb.value, "10");
    waitForMsg(msg_received_ya, node, dummy_node);
    EXPECT_EQ(param_ya.value, "9");
    waitForMsg(msg_received_yb, node, dummy_node);
    EXPECT_EQ(param_yb.value, "6");
  }

  rclcpp::shutdown();
}
