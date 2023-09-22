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

#include "gtest/gtest.h"
#include "simple_planning_simulator/simple_planning_simulator_core.hpp"
#include "tf2/utils.h"

#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include <memory>

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using nav_msgs::msg::Odometry;

using simulation::simple_planning_simulator::SimplePlanningSimulator;

std::string toStrInfo(const Odometry & o)
{
  const auto & p = o.pose.pose;
  const auto & t = o.twist.twist;
  std::stringstream ss;
  ss << "state x: " << p.position.x << ", y: " << p.position.y
     << ", yaw: " << tf2::getYaw(p.orientation) << ", vx = " << t.linear.x << ", vy: " << t.linear.y
     << ", wz: " << t.angular.z;
  return ss.str();
}

class PubSubNode : public rclcpp::Node
{
public:
  PubSubNode() : Node{"test_simple_planning_simulator_pubsub"}
  {
    current_odom_sub_ = create_subscription<Odometry>(
      "output/odometry", rclcpp::QoS{1},
      [this](const Odometry::ConstSharedPtr msg) { current_odom_ = msg; });
    pub_ackermann_command_ =
      create_publisher<AckermannControlCommand>("input/ackermann_control_command", rclcpp::QoS{1});
    pub_initialpose_ =
      create_publisher<PoseWithCovarianceStamped>("input/initialpose", rclcpp::QoS{1});
    pub_gear_cmd_ = create_publisher<GearCommand>("input/gear_command", rclcpp::QoS{1});
  }

  rclcpp::Subscription<Odometry>::SharedPtr current_odom_sub_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_ackermann_command_;
  rclcpp::Publisher<GearCommand>::SharedPtr pub_gear_cmd_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;

  Odometry::ConstSharedPtr current_odom_;
};

/**
 * @brief Generate an AckermannControlCommand message
 * @param [in] t timestamp
 * @param [in] steer [rad] steering
 * @param [in] steer_rate [rad/s] steering rotation rate
 * @param [in] vel [m/s] velocity
 * @param [in] acc [m/s²] acceleration
 * @param [in] jerk [m/s3] jerk
 */
AckermannControlCommand cmdGen(
  const builtin_interfaces::msg::Time & t, double steer, double steer_rate, double vel, double acc,
  double jerk)
{
  AckermannControlCommand cmd;
  cmd.stamp = t;
  cmd.lateral.stamp = t;
  cmd.lateral.steering_tire_angle = steer;
  cmd.lateral.steering_tire_rotation_rate = steer_rate;
  cmd.longitudinal.stamp = t;
  cmd.longitudinal.speed = vel;
  cmd.longitudinal.acceleration = acc;
  cmd.longitudinal.jerk = jerk;
  return cmd;
}

void resetInitialpose(rclcpp::Node::SharedPtr sim_node, std::shared_ptr<PubSubNode> pub_sub_node)
{
  PoseWithCovarianceStamped p;
  p.header.frame_id = "odom";
  p.header.stamp = sim_node->now();
  p.pose.pose.orientation.w = 1.0;  // yaw = 0
  for (int i = 0; i < 10; ++i) {
    pub_sub_node->pub_initialpose_->publish(p);
    rclcpp::spin_some(sim_node);
    rclcpp::spin_some(pub_sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
  }
}

void sendGear(
  uint8_t gear, rclcpp::Node::SharedPtr sim_node, std::shared_ptr<PubSubNode> pub_sub_node)
{
  GearCommand cmd;
  cmd.stamp = sim_node->now();
  cmd.command = gear;
  for (int i = 0; i < 10; ++i) {
    pub_sub_node->pub_gear_cmd_->publish(cmd);
    rclcpp::spin_some(sim_node);
    rclcpp::spin_some(pub_sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
  }
}

/**
 * @brief publish the given command message
 * @param [in] cmd command to publish
 * @param [in] sim_node pointer to the simulation node
 * @param [in] pub_sub_node pointer to the node used for communication
 */
void sendCommand(
  const AckermannControlCommand & cmd, rclcpp::Node::SharedPtr sim_node,
  std::shared_ptr<PubSubNode> pub_sub_node)
{
  for (int i = 0; i < 150; ++i) {
    pub_sub_node->pub_ackermann_command_->publish(cmd);
    rclcpp::spin_some(sim_node);
    rclcpp::spin_some(pub_sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
  }
}

// Check which direction the vehicle is heading on the baselink coordinates.
//                      y
//                      |
//                      |         (Fwd-Left)
//                      |
//  ---------(Bwd)------------------(Fwd)----------> x
//                      |
//        (Bwd-Right)   |
//                      |
//
void isOnForward(const Odometry & state, const Odometry & init)
{
  double forward_thr = 1.0;
  double dx = state.pose.pose.position.x - init.pose.pose.position.x;
  EXPECT_GT(dx, forward_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
}

void isOnBackward(const Odometry & state, const Odometry & init)
{
  double backward_thr = -1.0;
  double dx = state.pose.pose.position.x - init.pose.pose.position.x;
  EXPECT_LT(dx, backward_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
}

void isOnForwardLeft(const Odometry & state, const Odometry & init)
{
  double forward_thr = 1.0;
  double left_thr = 0.1f;
  double dx = state.pose.pose.position.x - init.pose.pose.position.x;
  double dy = state.pose.pose.position.y - init.pose.pose.position.y;
  EXPECT_GT(dx, forward_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
  EXPECT_GT(dy, left_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
}

void isOnBackwardRight(const Odometry & state, const Odometry & init)
{
  double backward_thr = -1.0;
  double right_thr = -0.1;
  double dx = state.pose.pose.position.x - init.pose.pose.position.x;
  double dy = state.pose.pose.position.y - init.pose.pose.position.y;
  EXPECT_LT(dx, backward_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
  EXPECT_LT(dy, right_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
}

void declareVehicleInfoParams(rclcpp::NodeOptions & node_options)
{
  node_options.append_parameter_override("wheel_radius", 0.5);
  node_options.append_parameter_override("wheel_width", 0.2);
  node_options.append_parameter_override("wheel_base", 3.0);
  node_options.append_parameter_override("wheel_tread", 2.0);
  node_options.append_parameter_override("front_overhang", 1.0);
  node_options.append_parameter_override("rear_overhang", 1.0);
  node_options.append_parameter_override("left_overhang", 0.5);
  node_options.append_parameter_override("right_overhang", 0.5);
  node_options.append_parameter_override("vehicle_height", 1.5);
  node_options.append_parameter_override("max_steer_angle", 0.7);
}

// Send a control command and run the simulation.
// Then check if the vehicle is moving in the desired direction.
class TestSimplePlanningSimulator : public ::testing::TestWithParam<std::string>
{
};

TEST_P(TestSimplePlanningSimulator, TestIdealSteerVel)
{
  rclcpp::init(0, nullptr);

  const auto vehicle_model_type = GetParam();

  std::cout << "\n\n vehicle model = " << vehicle_model_type << std::endl << std::endl;
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("initialize_source", "INITIAL_POSE_TOPIC");
  node_options.append_parameter_override("vehicle_model_type", vehicle_model_type);
  node_options.append_parameter_override("initial_engage_state", true);
  node_options.append_parameter_override("add_measurement_noise", false);
  declareVehicleInfoParams(node_options);
  const auto sim_node = std::make_shared<SimplePlanningSimulator>(node_options);

  const auto pub_sub_node = std::make_shared<PubSubNode>();

  const double target_vel = 5.0f;
  const double target_acc = 5.0f;
  const double target_steer = 0.2f;

  auto _resetInitialpose = [&]() { resetInitialpose(sim_node, pub_sub_node); };
  auto _sendFwdGear = [&]() { sendGear(GearCommand::DRIVE, sim_node, pub_sub_node); };
  auto _sendBwdGear = [&]() { sendGear(GearCommand::REVERSE, sim_node, pub_sub_node); };
  auto _sendCommand = [&](const auto & _cmd) { sendCommand(_cmd, sim_node, pub_sub_node); };

  // check pub-sub connections
  {
    size_t expected = 1;
    EXPECT_EQ(pub_sub_node->pub_ackermann_command_->get_subscription_count(), expected);
    EXPECT_EQ(pub_sub_node->pub_gear_cmd_->get_subscription_count(), expected);
    EXPECT_EQ(pub_sub_node->pub_initialpose_->get_subscription_count(), expected);
    EXPECT_EQ(pub_sub_node->current_odom_sub_->get_publisher_count(), expected);
  }

  // check initial pose
  _resetInitialpose();
  const auto init_state = *(pub_sub_node->current_odom_);

  // go forward
  _resetInitialpose();
  _sendFwdGear();
  _sendCommand(cmdGen(sim_node->now(), 0.0f, 0.0f, target_vel, target_acc, 0.0f));
  isOnForward(*(pub_sub_node->current_odom_), init_state);

  // go backward
  // NOTE: positive acceleration with reverse gear drives the vehicle backward.
  _resetInitialpose();
  _sendBwdGear();
  _sendCommand(cmdGen(sim_node->now(), 0.0f, 0.0f, -target_vel, target_acc, 0.0f));
  isOnBackward(*(pub_sub_node->current_odom_), init_state);

  // go forward left
  _resetInitialpose();
  _sendFwdGear();
  _sendCommand(cmdGen(sim_node->now(), target_steer, 0.0f, target_vel, target_acc, 0.0f));
  isOnForwardLeft(*(pub_sub_node->current_odom_), init_state);

  // go backward right
  // NOTE: positive acceleration with reverse gear drives the vehicle backward.
  _resetInitialpose();
  _sendBwdGear();
  _sendCommand(cmdGen(sim_node->now(), -target_steer, 0.0f, -target_vel, target_acc, 0.0f));
  isOnBackwardRight(*(pub_sub_node->current_odom_), init_state);

  rclcpp::shutdown();
}

// clang-format off
const std::string VEHICLE_MODEL_LIST[] = {   // NOLINT
  "IDEAL_STEER_VEL", "IDEAL_STEER_ACC", "IDEAL_STEER_ACC_GEARED",
  "DELAY_STEER_VEL", "DELAY_STEER_ACC", "DELAY_STEER_ACC_GEARED",
};
// clang-format on

INSTANTIATE_TEST_SUITE_P(
  TestForEachVehicleModel, TestSimplePlanningSimulator, ::testing::ValuesIn(VEHICLE_MODEL_LIST));
