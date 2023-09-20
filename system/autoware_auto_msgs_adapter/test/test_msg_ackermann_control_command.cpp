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

autoware_control_msgs::msg::Control generate_control_msg()
{
  // generate deterministic random float
  std::mt19937 gen(0);
  std::uniform_real_distribution<> dis(-100.0, 100.0);
  auto rand_float = [&dis, &gen]() { return static_cast<float>(dis(gen)); };

  // generate deterministic random int
  std::uniform_int_distribution<> dis_int(0, 1000000);
  auto rand_int = [&dis_int, &gen]() { return dis_int(gen); };

  autoware_control_msgs::msg::Control msg_control;
  msg_control.stamp = rclcpp::Time(rand_int());

  msg_control.lateral.stamp = rclcpp::Time(rand_int());
  msg_control.lateral.steering_tire_angle = rand_float();
  msg_control.lateral.steering_tire_rotation_rate = rand_float();

  msg_control.longitudinal.stamp = rclcpp::Time(rand_int());
  msg_control.longitudinal.velocity = rand_float();
  msg_control.longitudinal.jerk = rand_float();
  msg_control.longitudinal.acceleration = rand_float();
  return msg_control;
}

TEST(AutowareAutoMsgsAdapter, TestMsgAckermannControlCommand)  // NOLINT for gtest
{
  const std::string msg_type_target = "autoware_auto_control_msgs/msg/AckermannControlCommand";
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

  const auto msg_control = generate_control_msg();
  auto sub =
    node_subscriber->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      topic_name_target, rclcpp::QoS{1},
      [&msg_control, &test_completed](
        const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg) {
        EXPECT_EQ(msg->stamp, msg_control.stamp);

        EXPECT_EQ(msg->lateral.stamp, msg_control.lateral.stamp);
        EXPECT_FLOAT_EQ(msg->lateral.steering_tire_angle, msg_control.lateral.steering_tire_angle);
        EXPECT_FLOAT_EQ(
          msg->lateral.steering_tire_rotation_rate,
          msg_control.lateral.steering_tire_rotation_rate);

        EXPECT_EQ(msg->longitudinal.stamp, msg_control.longitudinal.stamp);
        EXPECT_FLOAT_EQ(msg->longitudinal.speed, msg_control.longitudinal.velocity);
        EXPECT_FLOAT_EQ(msg->longitudinal.acceleration, msg_control.longitudinal.acceleration);
        EXPECT_FLOAT_EQ(msg->longitudinal.jerk, msg_control.longitudinal.jerk);
        test_completed = true;
      });

  std::cout << "Creating the publisher node..." << std::endl;

  auto node_publisher = std::make_shared<rclcpp::Node>("node_publisher", rclcpp::NodeOptions{});
  auto pub = node_publisher->create_publisher<autoware_control_msgs::msg::Control>(
    topic_name_source, rclcpp::QoS{1});
  pub->publish(msg_control);

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
