// Copyright 2023 The Autoware Foundation
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

#include <autoware_auto_msgs_adapter_core.hpp>

#include <gtest/gtest.h>

#include <random>

autoware_map_msgs::msg::LaneletMapBin generate_map_msg()
{
  // generate deterministic random int
  std::mt19937 gen(0);
  std::uniform_int_distribution<> dis_int(0, 1000000);
  auto rand_int = [&dis_int, &gen]() { return dis_int(gen); };

  autoware_map_msgs::msg::LaneletMapBin msg_map;
  msg_map.header.stamp = rclcpp::Time(rand_int());
  msg_map.header.frame_id = "test_frame";

  msg_map.version_map_format = "1.1.1";
  msg_map.version_map = "1.0.0";
  msg_map.name_map = "florence-prato-city-center";
  msg_map.data.push_back(rand_int());

  return msg_map;
}

TEST(AutowareAutoMsgsAdapter, TestHADMapBin)  // NOLINT for gtest
{
  const std::string msg_type_target = "autoware_auto_mapping_msgs/msg/HADMapBin";
  const std::string topic_name_source = "topic_name_source";
  const std::string topic_name_target = "topic_name_target";

  std::cout << "Creating the adapter node..." << std::endl;

  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("msg_type_target", msg_type_target);
  node_options.append_parameter_override("topic_name_source", topic_name_source);
  node_options.append_parameter_override("topic_name_target", topic_name_target);

  using autoware_auto_msgs_adapter::AutowareAutoMsgsAdapterNode;
  AutowareAutoMsgsAdapterNode::SharedPtr node_adapter;
  node_adapter = std::make_shared<AutowareAutoMsgsAdapterNode>(node_options);

  std::cout << "Creating the subscriber node..." << std::endl;

  auto node_subscriber = std::make_shared<rclcpp::Node>("node_subscriber", rclcpp::NodeOptions{});

  bool test_completed = false;

  const auto msg_map = generate_map_msg();
  auto sub = node_subscriber->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    topic_name_target, rclcpp::QoS{1},
    [&msg_map, &test_completed](const autoware_auto_mapping_msgs::msg::HADMapBin::SharedPtr msg) {
      EXPECT_EQ(msg->header.stamp, msg_map.header.stamp);
      EXPECT_EQ(msg->header.frame_id, msg_map.header.frame_id);

      EXPECT_EQ(msg->map_format, 0);
      EXPECT_EQ(msg->format_version, msg_map.version_map_format);
      EXPECT_EQ(msg->map_version, msg_map.version_map);
      EXPECT_EQ(msg->data, msg_map.data);

      test_completed = true;
    });

  std::cout << "Creating the publisher node..." << std::endl;

  auto node_publisher = std::make_shared<rclcpp::Node>("node_publisher", rclcpp::NodeOptions{});
  auto pub = node_publisher->create_publisher<autoware_map_msgs::msg::LaneletMapBin>(
    topic_name_source, rclcpp::QoS{1});
  pub->publish(msg_map);

  auto start_time = std::chrono::system_clock::now();
  auto max_test_dur = std::chrono::seconds(5);
  auto timed_out = false;

  while (rclcpp::ok() && !test_completed) {
    rclcpp::spin_some(node_subscriber);
    rclcpp::spin_some(node_adapter);
    rclcpp::spin_some(node_publisher);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    if (std::chrono::system_clock::now() - start_time > max_test_dur) {
      timed_out = true;
      break;
    }
  }

  EXPECT_TRUE(test_completed);
  EXPECT_FALSE(timed_out);

  // rclcpp::shutdown();
}
