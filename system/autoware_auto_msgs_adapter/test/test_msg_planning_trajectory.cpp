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

autoware_planning_msgs::msg::Trajectory generate_planning_msg()
{
  using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;
  // generate deterministic random int
  std::mt19937 gen(0);
  std::uniform_int_distribution<> dis_int(0, 1000000);
  auto rand_int = [&dis_int, &gen]() { return dis_int(gen); };

  autoware_planning_msgs::msg::Trajectory msg_planning;
  msg_planning.header.stamp = rclcpp::Time(rand_int());
  msg_planning.header.frame_id = "test_frame";

  TrajectoryPoint point;
  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  for (size_t i = 0; i < 100; i++) {
    point.longitudinal_velocity_mps = 1.0;
    point.time_from_start = rclcpp::Duration::from_seconds(0.0);
    point.pose = pose;
    point.longitudinal_velocity_mps = 20.0;
    point.lateral_velocity_mps = 0.0;
    point.acceleration_mps2 = 1.0;
    point.heading_rate_rps = 2.0;
    point.front_wheel_angle_rad = 8.0;
    point.rear_wheel_angle_rad = 10.0;

    msg_planning.points.push_back(point);
  }

  return msg_planning;
}

TEST(AutowareAutoMsgsAdapter, TestTrajectory)  // NOLINT for gtest
{
  const std::string msg_type_target = "autoware_auto_planning_msgs/msg/Trajectory";
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

  const auto msg_planning = generate_planning_msg();
  auto sub = node_subscriber->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    topic_name_target, rclcpp::QoS{1},
    [&msg_planning,
     &test_completed](const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) {
      EXPECT_EQ(msg->header.stamp, msg_planning.header.stamp);
      EXPECT_EQ(msg->header.frame_id, msg_planning.header.frame_id);
      for (size_t i = 0; i < msg_planning.points.size(); i++) {
        EXPECT_FLOAT_EQ(
          msg->points.at(i).pose.position.x, msg_planning.points.at(i).pose.position.x);
        EXPECT_FLOAT_EQ(
          msg->points.at(i).pose.position.y, msg_planning.points.at(i).pose.position.y);
        EXPECT_FLOAT_EQ(
          msg->points.at(i).pose.position.z, msg_planning.points.at(i).pose.position.z);
        EXPECT_FLOAT_EQ(
          msg->points.at(i).pose.orientation.x, msg_planning.points.at(i).pose.orientation.x);
        EXPECT_FLOAT_EQ(
          msg->points.at(i).pose.orientation.y, msg_planning.points.at(i).pose.orientation.y);
        EXPECT_FLOAT_EQ(
          msg->points.at(i).pose.orientation.z, msg_planning.points.at(i).pose.orientation.z);
        EXPECT_FLOAT_EQ(
          msg->points.at(i).pose.orientation.w, msg_planning.points.at(i).pose.orientation.w);
        EXPECT_FLOAT_EQ(
          msg->points.at(i).longitudinal_velocity_mps,
          msg_planning.points.at(i).longitudinal_velocity_mps);
        EXPECT_FLOAT_EQ(
          msg->points.at(i).lateral_velocity_mps, msg_planning.points.at(i).lateral_velocity_mps);
        EXPECT_FLOAT_EQ(
          msg->points.at(i).acceleration_mps2, msg_planning.points.at(i).acceleration_mps2);
        EXPECT_FLOAT_EQ(
          msg->points.at(i).heading_rate_rps, msg_planning.points.at(i).heading_rate_rps);
        EXPECT_FLOAT_EQ(
          msg->points.at(i).front_wheel_angle_rad, msg_planning.points.at(i).front_wheel_angle_rad);
        EXPECT_FLOAT_EQ(
          msg->points.at(i).rear_wheel_angle_rad, msg_planning.points.at(i).rear_wheel_angle_rad);
      }

      test_completed = true;
    });

  std::cout << "Creating the publisher node..." << std::endl;

  auto node_publisher = std::make_shared<rclcpp::Node>("node_publisher", rclcpp::NodeOptions{});
  auto pub = node_publisher->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    topic_name_source, rclcpp::QoS{1});
  pub->publish(msg_planning);

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

  //   rclcpp::shutdown();
}
