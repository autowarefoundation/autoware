// Copyright 2021 Tier IV, Inc.
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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "fake_test_node/fake_test_node.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "trajectory_follower_test_utils.hpp"

#include <trajectory_follower_node/longitudinal_controller_node.hpp>

#include "autoware_auto_control_msgs/msg/longitudinal_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <memory>

using LongitudinalController =
  autoware::motion::control::trajectory_follower_node::LongitudinalController;
using LongitudinalCommand = autoware_auto_control_msgs::msg::LongitudinalCommand;
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using VehicleOdometry = nav_msgs::msg::Odometry;

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

std::shared_ptr<LongitudinalController> makeLongitudinalNode()
{
  // Pass default parameter file to the node
  const auto share_dir = ament_index_cpp::get_package_share_directory("trajectory_follower_node");
  rclcpp::NodeOptions node_options;
  node_options.arguments(
    {"--ros-args", "--params-file",
     share_dir + "/param/longitudinal_controller_defaults.param.yaml", "--params-file",
     share_dir + "/param/test_vehicle_info.param.yaml", "--params-file",
     share_dir + "/param/test_nearest_search.param.yaml"});
  std::shared_ptr<LongitudinalController> node =
    std::make_shared<LongitudinalController>(node_options);

  // Enable all logging in the node
  auto ret =
    rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK) {
    std::cout << "Failed to set logging severity to DEBUG\n";
  }
  return node;
}

TEST_F(FakeNodeFixture, longitudinal_keep_velocity)
{
  // Data to test
  LongitudinalCommand::SharedPtr cmd_msg;
  bool received_longitudinal_command = false;

  // Node
  std::shared_ptr<LongitudinalController> node = makeLongitudinalNode();

  // Publisher/Subscribers
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("longitudinal_controller/input/current_odometry");
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("longitudinal_controller/input/current_trajectory");
  rclcpp::Subscription<LongitudinalCommand>::SharedPtr cmd_sub =
    this->create_subscription<LongitudinalCommand>(
      "longitudinal_controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_longitudinal_command](const LongitudinalCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_longitudinal_command = true;
      });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);
  /// Already running at target vel + Non stopping trajectory -> no change in velocity
  // Publish velocity
  VehicleOdometry odom_msg;
  odom_msg.header.stamp = node->now();
  odom_msg.twist.twist.linear.x = 1.0;
  odom_pub->publish(odom_msg);
  // the node needs to receive two velocity msg
  rclcpp::spin_some(node);
  rclcpp::spin_some(this->get_fake_node());
  odom_msg.header.stamp = node->now();
  odom_pub->publish(odom_msg);
  // Publish non stopping trajectory
  Trajectory traj;
  traj.header.stamp = node->now();
  traj.header.frame_id = "map";
  TrajectoryPoint point;
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 50.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 100.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  traj_pub->publish(traj);
  test_utils::waitForMessage(node, this, received_longitudinal_command);

  ASSERT_TRUE(received_longitudinal_command);
  EXPECT_DOUBLE_EQ(cmd_msg->speed, 1.0);
  EXPECT_DOUBLE_EQ(cmd_msg->acceleration, 0.0);

  // Generate another control message
  received_longitudinal_command = false;
  traj_pub->publish(traj);
  test_utils::waitForMessage(node, this, received_longitudinal_command);
  ASSERT_TRUE(received_longitudinal_command);
  EXPECT_DOUBLE_EQ(cmd_msg->speed, 1.0);
  EXPECT_DOUBLE_EQ(cmd_msg->acceleration, 0.0);
}

TEST_F(FakeNodeFixture, longitudinal_slow_down)
{
  // Data to test
  LongitudinalCommand::SharedPtr cmd_msg;
  bool received_longitudinal_command = false;

  // Node
  std::shared_ptr<LongitudinalController> node = makeLongitudinalNode();

  // Publisher/Subscribers
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("longitudinal_controller/input/current_odometry");
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("longitudinal_controller/input/current_trajectory");
  rclcpp::Subscription<LongitudinalCommand>::SharedPtr cmd_sub =
    this->create_subscription<LongitudinalCommand>(
      "longitudinal_controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_longitudinal_command](const LongitudinalCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_longitudinal_command = true;
      });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);
  /// Already running at target vel + Non stopping trajectory -> no change in velocity
  // Publish velocity
  VehicleOdometry odom_msg;
  odom_msg.header.stamp = node->now();
  odom_msg.twist.twist.linear.x = 1.0;
  odom_pub->publish(odom_msg);
  // the node needs to receive two velocity msg
  rclcpp::spin_some(node);
  rclcpp::spin_some(this->get_fake_node());
  odom_msg.header.stamp = node->now();
  odom_pub->publish(odom_msg);
  // Publish non stopping trajectory
  Trajectory traj;
  traj.header.stamp = node->now();
  traj.header.frame_id = "map";
  TrajectoryPoint point;
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 0.5;
  traj.points.push_back(point);
  point.pose.position.x = 50.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 0.5;
  traj.points.push_back(point);
  point.pose.position.x = 100.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 0.5;
  traj.points.push_back(point);
  traj_pub->publish(traj);
  test_utils::waitForMessage(node, this, received_longitudinal_command);

  ASSERT_TRUE(received_longitudinal_command);
  EXPECT_LT(cmd_msg->speed, static_cast<float>(odom_msg.twist.twist.linear.x));
  EXPECT_LT(cmd_msg->acceleration, 0.0f);

  // Generate another control message
  received_longitudinal_command = false;
  traj_pub->publish(traj);
  test_utils::waitForMessage(node, this, received_longitudinal_command);
  ASSERT_TRUE(received_longitudinal_command);
  EXPECT_LT(cmd_msg->speed, static_cast<float>(odom_msg.twist.twist.linear.x));
  EXPECT_LT(cmd_msg->acceleration, 0.0f);
}

TEST_F(FakeNodeFixture, longitudinal_accelerate)
{
  // Data to test
  LongitudinalCommand::SharedPtr cmd_msg;
  bool received_longitudinal_command = false;

  // Node
  std::shared_ptr<LongitudinalController> node = makeLongitudinalNode();

  // Publisher/Subscribers
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("longitudinal_controller/input/current_odometry");
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("longitudinal_controller/input/current_trajectory");
  rclcpp::Subscription<LongitudinalCommand>::SharedPtr cmd_sub =
    this->create_subscription<LongitudinalCommand>(
      "longitudinal_controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_longitudinal_command](const LongitudinalCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_longitudinal_command = true;
      });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);
  /// Below target vel + Non stopping trajectory -> accelerate
  // Publish velocity
  VehicleOdometry odom_msg;
  odom_msg.header.stamp = node->now();
  odom_msg.twist.twist.linear.x = 0.5;
  odom_pub->publish(odom_msg);
  // the node needs to receive two velocity msg
  rclcpp::spin_some(node);
  rclcpp::spin_some(this->get_fake_node());
  odom_msg.header.stamp = node->now();
  odom_pub->publish(odom_msg);
  // Publish non stopping trajectory
  Trajectory traj;
  traj.header.stamp = node->now();
  traj.header.frame_id = "map";
  TrajectoryPoint point;
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 50.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 100.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  traj_pub->publish(traj);
  test_utils::waitForMessage(node, this, received_longitudinal_command);

  ASSERT_TRUE(received_longitudinal_command);
  EXPECT_GT(cmd_msg->speed, static_cast<float>(odom_msg.twist.twist.linear.x));
  EXPECT_GT(cmd_msg->acceleration, 0.0f);

  // Generate another control message
  received_longitudinal_command = false;
  traj_pub->publish(traj);
  test_utils::waitForMessage(node, this, received_longitudinal_command);
  ASSERT_TRUE(received_longitudinal_command);
  EXPECT_GT(cmd_msg->speed, static_cast<float>(odom_msg.twist.twist.linear.x));
  EXPECT_GT(cmd_msg->acceleration, 0.0f);
}

TEST_F(FakeNodeFixture, longitudinal_stopped)
{
  // Data to test
  LongitudinalCommand::SharedPtr cmd_msg;
  bool received_longitudinal_command = false;

  // Node
  std::shared_ptr<LongitudinalController> node = makeLongitudinalNode();

  // Publisher/Subscribers
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("longitudinal_controller/input/current_odometry");
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("longitudinal_controller/input/current_trajectory");
  rclcpp::Subscription<LongitudinalCommand>::SharedPtr cmd_sub =
    this->create_subscription<LongitudinalCommand>(
      "longitudinal_controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_longitudinal_command](const LongitudinalCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_longitudinal_command = true;
      });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);
  /// Below target vel + Non stopping trajectory -> accelerate
  // Publish velocity
  VehicleOdometry odom_msg;
  odom_msg.header.stamp = node->now();
  odom_msg.twist.twist.linear.x = 0.0;
  odom_pub->publish(odom_msg);
  // the node needs to receive two velocity msg
  rclcpp::spin_some(node);
  rclcpp::spin_some(this->get_fake_node());
  odom_msg.header.stamp = node->now();
  odom_pub->publish(odom_msg);
  // Publish stopping trajectory
  Trajectory traj;
  traj.header.stamp = node->now();
  traj.header.frame_id = "map";
  TrajectoryPoint point;
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 0.0;
  traj.points.push_back(point);
  point.pose.position.x = 50.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 0.0;
  traj.points.push_back(point);
  point.pose.position.x = 100.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 0.0;
  traj.points.push_back(point);
  traj_pub->publish(traj);
  test_utils::waitForMessage(node, this, received_longitudinal_command);

  ASSERT_TRUE(received_longitudinal_command);
  EXPECT_DOUBLE_EQ(cmd_msg->speed, 0.0f);
  EXPECT_LT(cmd_msg->acceleration, 0.0f);  // when stopped negative acceleration to brake
}

TEST_F(FakeNodeFixture, longitudinal_reverse)
{
  // Data to test
  LongitudinalCommand::SharedPtr cmd_msg;
  bool received_longitudinal_command = false;

  // Node
  std::shared_ptr<LongitudinalController> node = makeLongitudinalNode();

  // Publisher/Subscribers
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("longitudinal_controller/input/current_odometry");
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("longitudinal_controller/input/current_trajectory");
  rclcpp::Subscription<LongitudinalCommand>::SharedPtr cmd_sub =
    this->create_subscription<LongitudinalCommand>(
      "longitudinal_controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_longitudinal_command](const LongitudinalCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_longitudinal_command = true;
      });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);
  /// Below target vel + Non stopping trajectory -> accelerate
  // Publish velocity
  VehicleOdometry odom_msg;
  odom_msg.header.stamp = node->now();
  odom_msg.twist.twist.linear.x = 0.0;
  odom_pub->publish(odom_msg);
  // the node needs to receive two velocity msg
  rclcpp::spin_some(node);
  rclcpp::spin_some(this->get_fake_node());
  odom_msg.header.stamp = node->now();
  odom_pub->publish(odom_msg);
  // Publish reverse
  Trajectory traj;
  traj.header.stamp = node->now();
  traj.header.frame_id = "map";
  TrajectoryPoint point;
  point.pose.position.x = 0.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = -1.0;
  traj.points.push_back(point);
  point.pose.position.x = 50.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = -1.0;
  traj.points.push_back(point);
  point.pose.position.x = 100.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = -1.0;
  traj.points.push_back(point);
  traj_pub->publish(traj);
  test_utils::waitForMessage(node, this, received_longitudinal_command);

  ASSERT_TRUE(received_longitudinal_command);
  EXPECT_LT(cmd_msg->speed, 0.0f);
  EXPECT_GT(cmd_msg->acceleration, 0.0f);
}

TEST_F(FakeNodeFixture, longitudinal_emergency)
{
  // Data to test
  LongitudinalCommand::SharedPtr cmd_msg;
  bool received_longitudinal_command = false;

  // Node
  std::shared_ptr<LongitudinalController> node = makeLongitudinalNode();

  // Publisher/Subscribers
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("longitudinal_controller/input/current_odometry");
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("longitudinal_controller/input/current_trajectory");
  rclcpp::Subscription<LongitudinalCommand>::SharedPtr cmd_sub =
    this->create_subscription<LongitudinalCommand>(
      "longitudinal_controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_longitudinal_command](const LongitudinalCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_longitudinal_command = true;
      });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);
  /// Below target vel + Non stopping trajectory -> accelerate
  // Publish velocity
  VehicleOdometry odom_msg;
  odom_msg.header.stamp = node->now();
  odom_msg.twist.twist.linear.x = 0.0;
  odom_pub->publish(odom_msg);
  // the node needs to receive two velocity msg
  rclcpp::spin_some(node);
  rclcpp::spin_some(this->get_fake_node());
  odom_msg.header.stamp = node->now();
  odom_pub->publish(odom_msg);
  // Publish trajectory starting away from the current ego pose
  Trajectory traj;
  traj.header.stamp = node->now();
  traj.header.frame_id = "map";
  TrajectoryPoint point;
  point.pose.position.x = 10.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 50.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  point.pose.position.x = 100.0;
  point.pose.position.y = 0.0;
  point.longitudinal_velocity_mps = 1.0;
  traj.points.push_back(point);
  traj_pub->publish(traj);
  test_utils::waitForMessage(node, this, received_longitudinal_command);

  ASSERT_TRUE(received_longitudinal_command);
  // Emergencies (e.g., far from trajectory) produces braking command (0 vel, negative accel)
  EXPECT_DOUBLE_EQ(cmd_msg->speed, 0.0f);
  EXPECT_LT(cmd_msg->acceleration, 0.0f);
}
