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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "fake_test_node/fake_test_node.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "trajectory_follower_nodes/controller_node.hpp"
#include "trajectory_follower_test_utils.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <memory>
#include <vector>

using Controller = autoware::motion::control::trajectory_follower_nodes::Controller;
using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using VehicleOdometry = nav_msgs::msg::Odometry;
using SteeringReport = autoware_auto_vehicle_msgs::msg::SteeringReport;

using FakeNodeFixture = autoware::tools::testing::FakeTestNode;

const rclcpp::Duration one_second(1, 0);

std::shared_ptr<Controller> makeNode()
{
  // Pass default parameter file to the node
  const auto share_dir = ament_index_cpp::get_package_share_directory("trajectory_follower_nodes");
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("ctrl_period", 0.03);
  node_options.append_parameter_override("timeout_thr_sec", 0.5);
  node_options.append_parameter_override(
    "enable_keep_stopped_until_steer_convergence", false);  // longitudinal
  node_options.arguments(
    {"--ros-args", "--params-file", share_dir + "/param/lateral_controller_defaults.param.yaml",
     "--params-file", share_dir + "/param/longitudinal_controller_defaults.param.yaml",
     "--params-file", share_dir + "/param/test_vehicle_info.param.yaml"});
  std::shared_ptr<Controller> node = std::make_shared<Controller>(node_options);

  // Enable all logging in the node
  auto ret =
    rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (ret != RCUTILS_RET_OK) {
    std::cout << "Failed to set logging severerity to DEBUG\n";
  }
  return node;
}

TEST_F(FakeNodeFixture, no_input)
{
  // Data to test
  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;
  // Node
  std::shared_ptr<Controller> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("controller/input/reference_trajectory");
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("controller/input/current_odometry");
  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("controller/input/current_steering");
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_control_command](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
      });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // No published data: expect a stopped command
  test_utils::waitForMessage(
    node, this, received_control_command, std::chrono::seconds{1LL}, false);
  ASSERT_FALSE(received_control_command);
}

TEST_F(FakeNodeFixture, empty_trajectory)
{
  // Data to test
  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;
  // Node
  std::shared_ptr<Controller> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("controller/input/reference_trajectory");
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("controller/input/current_odometry");
  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("controller/input/current_steering");
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_control_command](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
      });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);

  // Spin for transform to be published
  test_utils::spinWhile(node);

  // Empty trajectory: expect a stopped command
  Trajectory traj_msg;
  traj_msg.header.stamp = node->now();
  traj_msg.header.frame_id = "map";
  VehicleOdometry odom_msg;
  SteeringReport steer_msg;
  traj_msg.header.stamp = node->now();
  odom_msg.header.stamp = node->now();
  odom_msg.twist.twist.linear.x = 0.0;
  steer_msg.stamp = node->now();
  steer_msg.steering_tire_angle = 0.0;
  traj_pub->publish(traj_msg);
  odom_pub->publish(odom_msg);
  steer_pub->publish(steer_msg);

  test_utils::waitForMessage(
    node, this, received_control_command, std::chrono::seconds{1LL}, false);
  ASSERT_FALSE(received_control_command);
}

// lateral
TEST_F(FakeNodeFixture, straight_trajectory)
{
  // Data to test
  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;
  // Node
  std::shared_ptr<Controller> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("controller/input/reference_trajectory");
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("controller/input/current_odometry");
  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("controller/input/current_steering");
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_control_command](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
      });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);

  // Spin for transform to be published
  test_utils::spinWhile(node);

  // Straight trajectory: expect no steering
  received_control_command = false;
  Trajectory traj_msg;
  traj_msg.header.stamp = node->now();
  traj_msg.header.frame_id = "map";
  VehicleOdometry odom_msg;
  SteeringReport steer_msg;
  TrajectoryPoint p;
  traj_msg.header.stamp = node->now();
  p.pose.position.x = -1.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.pose.position.x = 1.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.pose.position.x = 2.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  traj_pub->publish(traj_msg);
  odom_msg.header.stamp = node->now();
  odom_msg.twist.twist.linear.x = 1.0;
  steer_msg.stamp = node->now();
  steer_msg.steering_tire_angle = 0.0;
  odom_pub->publish(odom_msg);
  steer_pub->publish(steer_msg);

  test_utils::waitForMessage(node, this, received_control_command);
  ASSERT_TRUE(received_control_command);
  EXPECT_EQ(cmd_msg->lateral.steering_tire_angle, 0.0f);
  EXPECT_EQ(cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

TEST_F(FakeNodeFixture, right_turn)
{
  // Data to test
  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;
  // Node
  std::shared_ptr<Controller> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("controller/input/reference_trajectory");
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("controller/input/current_odometry");
  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("controller/input/current_steering");
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_control_command](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
      });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);

  // Spin for transform to be published
  test_utils::spinWhile(node);

  // Right turning trajectory: expect right steering
  received_control_command = false;
  Trajectory traj_msg;
  traj_msg.header.stamp = node->now();
  traj_msg.header.frame_id = "map";
  VehicleOdometry odom_msg;
  SteeringReport steer_msg;
  TrajectoryPoint p;
  traj_msg.points.clear();
  p.pose.position.x = -1.0;
  p.pose.position.y = -1.0;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.pose.position.x = 1.0;
  p.pose.position.y = -1.0;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.pose.position.x = 2.0;
  p.pose.position.y = -2.0;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  traj_pub->publish(traj_msg);
  odom_msg.header.stamp = node->now();
  odom_msg.twist.twist.linear.x = 1.0;
  steer_msg.stamp = node->now();
  steer_msg.steering_tire_angle = 0.0;
  odom_pub->publish(odom_msg);
  steer_pub->publish(steer_msg);

  test_utils::waitForMessage(node, this, received_control_command);
  ASSERT_TRUE(received_control_command);
  EXPECT_LT(cmd_msg->lateral.steering_tire_angle, 0.0f);
  EXPECT_LT(cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

TEST_F(FakeNodeFixture, left_turn)
{
  // Data to test
  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;
  // Node
  std::shared_ptr<Controller> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("controller/input/reference_trajectory");
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("controller/input/current_odometry");
  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("controller/input/current_steering");
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_control_command](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
      });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);

  // Spin for transform to be published
  test_utils::spinWhile(node);

  // Left turning trajectory: expect left steering
  received_control_command = false;
  Trajectory traj_msg;
  traj_msg.header.stamp = node->now();
  traj_msg.header.frame_id = "map";
  VehicleOdometry odom_msg;
  SteeringReport steer_msg;
  TrajectoryPoint p;
  traj_msg.points.clear();
  p.pose.position.x = -1.0;
  p.pose.position.y = 1.0;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.pose.position.x = 1.0;
  p.pose.position.y = 1.0;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  p.pose.position.x = 2.0;
  p.pose.position.y = 2.0;
  p.longitudinal_velocity_mps = 1.0f;
  traj_msg.points.push_back(p);
  traj_pub->publish(traj_msg);
  odom_msg.header.stamp = node->now();
  odom_msg.twist.twist.linear.x = 1.0;
  steer_msg.stamp = node->now();
  steer_msg.steering_tire_angle = 0.0;
  odom_pub->publish(odom_msg);
  steer_pub->publish(steer_msg);

  test_utils::waitForMessage(node, this, received_control_command);
  ASSERT_TRUE(received_control_command);
  EXPECT_GT(cmd_msg->lateral.steering_tire_angle, 0.0f);
  EXPECT_GT(cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

TEST_F(FakeNodeFixture, stopped)
{
  // Data to test
  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;
  // Node
  std::shared_ptr<Controller> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("controller/input/reference_trajectory");
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("controller/input/current_odometry");
  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("controller/input/current_steering");
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_control_command](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
      });
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br =
    std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->get_fake_node());

  // Dummy transform: ego is at (0.0, 0.0) in map frame
  geometry_msgs::msg::TransformStamped transform = test_utils::getDummyTransform();
  transform.header.stamp = node->now();
  br->sendTransform(transform);

  // Spin for transform to be published
  test_utils::spinWhile(node);

  // Straight trajectory: expect no steering
  received_control_command = false;
  Trajectory traj_msg;
  traj_msg.header.stamp = node->now();
  traj_msg.header.frame_id = "map";
  VehicleOdometry odom_msg;
  SteeringReport steer_msg;
  TrajectoryPoint p;
  traj_msg.header.stamp = node->now();
  p.pose.position.x = -1.0;
  p.pose.position.y = 0.0;
  // Set a 0 current velocity and 0 target velocity -> stopped state
  p.longitudinal_velocity_mps = 0.0f;
  traj_msg.points.push_back(p);
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 0.0f;
  traj_msg.points.push_back(p);
  p.pose.position.x = 1.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 0.0f;
  traj_msg.points.push_back(p);
  p.pose.position.x = 2.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 0.0f;
  traj_msg.points.push_back(p);
  traj_pub->publish(traj_msg);
  odom_msg.header.stamp = node->now();
  odom_msg.twist.twist.linear.x = 0.0;
  steer_msg.stamp = node->now();
  steer_msg.steering_tire_angle = -0.5;
  odom_pub->publish(odom_msg);
  steer_pub->publish(steer_msg);

  test_utils::waitForMessage(node, this, received_control_command);
  ASSERT_TRUE(received_control_command);
  EXPECT_EQ(cmd_msg->lateral.steering_tire_angle, steer_msg.steering_tire_angle);
  EXPECT_EQ(cmd_msg->lateral.steering_tire_rotation_rate, 0.0f);
  EXPECT_GT(rclcpp::Time(cmd_msg->stamp), rclcpp::Time(traj_msg.header.stamp));
}

// longitudinal
TEST_F(FakeNodeFixture, longitudinal_keep_velocity)
{
  // Data to test
  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;
  // Node
  std::shared_ptr<Controller> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("controller/input/reference_trajectory");
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("controller/input/current_odometry");
  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("controller/input/current_steering");
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_control_command](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
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
  // Publish steering
  SteeringReport steer_msg;
  steer_msg.stamp = node->now();
  steer_msg.steering_tire_angle = 0.0;
  steer_pub->publish(steer_msg);
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
  test_utils::waitForMessage(node, this, received_control_command);

  ASSERT_TRUE(received_control_command);
  EXPECT_DOUBLE_EQ(cmd_msg->longitudinal.speed, 1.0);
  EXPECT_DOUBLE_EQ(cmd_msg->longitudinal.acceleration, 0.0);

  // Generate another control message
  received_control_command = false;
  traj_pub->publish(traj);
  test_utils::waitForMessage(node, this, received_control_command);
  ASSERT_TRUE(received_control_command);
  EXPECT_DOUBLE_EQ(cmd_msg->longitudinal.speed, 1.0);
  EXPECT_DOUBLE_EQ(cmd_msg->longitudinal.acceleration, 0.0);
}

TEST_F(FakeNodeFixture, longitudinal_slow_down)
{
  // Data to test
  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;
  // Node
  std::shared_ptr<Controller> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("controller/input/reference_trajectory");
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("controller/input/current_odometry");
  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("controller/input/current_steering");
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_control_command](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
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
  // Publish steering
  SteeringReport steer_msg;
  steer_msg.stamp = node->now();
  steer_msg.steering_tire_angle = 0.0;
  steer_pub->publish(steer_msg);
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
  test_utils::waitForMessage(node, this, received_control_command);

  ASSERT_TRUE(received_control_command);
  EXPECT_LT(cmd_msg->longitudinal.speed, static_cast<float>(odom_msg.twist.twist.linear.x));
  EXPECT_LT(cmd_msg->longitudinal.acceleration, 0.0f);

  // Generate another control message
  received_control_command = false;
  traj_pub->publish(traj);
  test_utils::waitForMessage(node, this, received_control_command);
  ASSERT_TRUE(received_control_command);
  EXPECT_LT(cmd_msg->longitudinal.speed, static_cast<float>(odom_msg.twist.twist.linear.x));
  EXPECT_LT(cmd_msg->longitudinal.acceleration, 0.0f);
}

TEST_F(FakeNodeFixture, longitudinal_accelerate)
{
  // Data to test
  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;
  // Node
  std::shared_ptr<Controller> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("controller/input/reference_trajectory");
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("controller/input/current_odometry");
  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("controller/input/current_steering");
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_control_command](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
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
  // Publish steering
  SteeringReport steer_msg;
  steer_msg.stamp = node->now();
  steer_msg.steering_tire_angle = 0.0;
  steer_pub->publish(steer_msg);
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
  test_utils::waitForMessage(node, this, received_control_command);

  ASSERT_TRUE(received_control_command);
  EXPECT_GT(cmd_msg->longitudinal.speed, static_cast<float>(odom_msg.twist.twist.linear.x));
  EXPECT_GT(cmd_msg->longitudinal.acceleration, 0.0f);

  // Generate another control message
  received_control_command = false;
  traj_pub->publish(traj);
  test_utils::waitForMessage(node, this, received_control_command);
  ASSERT_TRUE(received_control_command);
  EXPECT_GT(cmd_msg->longitudinal.speed, static_cast<float>(odom_msg.twist.twist.linear.x));
  EXPECT_GT(cmd_msg->longitudinal.acceleration, 0.0f);
}

TEST_F(FakeNodeFixture, longitudinal_stopped)
{
  // Data to test
  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;
  // Node
  std::shared_ptr<Controller> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("controller/input/reference_trajectory");
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("controller/input/current_odometry");
  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("controller/input/current_steering");
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_control_command](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
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
  // Publish steering
  SteeringReport steer_msg;
  steer_msg.stamp = node->now();
  steer_msg.steering_tire_angle = 0.0;
  steer_pub->publish(steer_msg);
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
  test_utils::waitForMessage(node, this, received_control_command);

  ASSERT_TRUE(received_control_command);
  EXPECT_DOUBLE_EQ(cmd_msg->longitudinal.speed, 0.0f);
  EXPECT_LT(
    cmd_msg->longitudinal.acceleration, 0.0f);  // when stopped negative acceleration to brake
}

TEST_F(FakeNodeFixture, longitudinal_reverse)
{
  // Data to test
  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;
  // Node
  std::shared_ptr<Controller> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("controller/input/reference_trajectory");
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("controller/input/current_odometry");
  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("controller/input/current_steering");
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_control_command](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
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
  // Publish steering
  SteeringReport steer_msg;
  steer_msg.stamp = node->now();
  steer_msg.steering_tire_angle = 0.0;
  steer_pub->publish(steer_msg);
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
  test_utils::waitForMessage(node, this, received_control_command);

  ASSERT_TRUE(received_control_command);
  EXPECT_LT(cmd_msg->longitudinal.speed, 0.0f);
  EXPECT_GT(cmd_msg->longitudinal.acceleration, 0.0f);
}

TEST_F(FakeNodeFixture, longitudinal_emergency)
{
  // Data to test
  AckermannControlCommand::SharedPtr cmd_msg;
  bool received_control_command = false;
  // Node
  std::shared_ptr<Controller> node = makeNode();
  // Publisher/Subscribers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub =
    this->create_publisher<Trajectory>("controller/input/reference_trajectory");
  rclcpp::Publisher<VehicleOdometry>::SharedPtr odom_pub =
    this->create_publisher<VehicleOdometry>("controller/input/current_odometry");
  rclcpp::Publisher<SteeringReport>::SharedPtr steer_pub =
    this->create_publisher<SteeringReport>("controller/input/current_steering");
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr cmd_sub =
    this->create_subscription<AckermannControlCommand>(
      "controller/output/control_cmd", *this->get_fake_node(),
      [&cmd_msg, &received_control_command](const AckermannControlCommand::SharedPtr msg) {
        cmd_msg = msg;
        received_control_command = true;
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
  // Publish steering
  SteeringReport steer_msg;
  steer_msg.stamp = node->now();
  steer_msg.steering_tire_angle = 0.0;
  steer_pub->publish(steer_msg);
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
  test_utils::waitForMessage(node, this, received_control_command);

  ASSERT_TRUE(received_control_command);
  // Emergencies (e.g., far from trajectory) produces braking command (0 vel, negative accel)
  EXPECT_DOUBLE_EQ(cmd_msg->longitudinal.speed, 0.0f);
  EXPECT_LT(cmd_msg->longitudinal.acceleration, 0.0f);
}
