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
#include "gtest/gtest.h"
#include "simple_planning_simulator/simple_planning_simulator_core.hpp"
#include "tf2/utils.h"

#include "tier4_vehicle_msgs/msg/actuation_command_stamped.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include <memory>

using autoware_control_msgs::msg::Control;
using autoware_vehicle_msgs::msg::GearCommand;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using tier4_vehicle_msgs::msg::ActuationCommandStamped;

using simulation::simple_planning_simulator::InputCommand;
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

enum class CommandType { Ackermann, Actuation };

struct Ackermann
{
  double steer = 0.0;
  double steer_rate = 0.0;
  double vel = 0.0;
  double acc = 0.0;
  double jerk = 0.0;
};
struct Actuation
{
  double steer = 0.0;
  double accel = 0.0;
  double brake = 0.0;
};

class PubSubNode : public rclcpp::Node
{
public:
  PubSubNode() : Node{"test_simple_planning_simulator_pubsub"}
  {
    current_odom_sub_ = create_subscription<Odometry>(
      "output/odometry", rclcpp::QoS{1},
      [this](const Odometry::ConstSharedPtr msg) { current_odom_ = msg; });
    pub_ackermann_command_ =
      create_publisher<Control>("input/ackermann_control_command", rclcpp::QoS{1});
    pub_actuation_command_ =
      create_publisher<ActuationCommandStamped>("input/actuation_command", rclcpp::QoS{1});
    pub_initialpose_ =
      create_publisher<PoseWithCovarianceStamped>("input/initialpose", rclcpp::QoS{1});
    pub_gear_cmd_ = create_publisher<GearCommand>("input/gear_command", rclcpp::QoS{1});
  }

  rclcpp::Subscription<Odometry>::SharedPtr current_odom_sub_;
  rclcpp::Publisher<Control>::SharedPtr pub_ackermann_command_;
  rclcpp::Publisher<ActuationCommandStamped>::SharedPtr pub_actuation_command_;
  rclcpp::Publisher<GearCommand>::SharedPtr pub_gear_cmd_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_initialpose_;

  Odometry::ConstSharedPtr current_odom_;
};

/**
 * @brief Generate an Control message
 * @param [in] t timestamp
 * @param [in] steer [rad] steering
 * @param [in] steer_rate [rad/s] steering rotation rate
 * @param [in] vel [m/s] velocity
 * @param [in] acc [m/s²] acceleration
 * @param [in] jerk [m/s3] jerk
 */
Control ackermannCmdGen(const builtin_interfaces::msg::Time & t, const Ackermann & ackermann_cmd)
{
  Control cmd;
  cmd.stamp = t;
  cmd.lateral.stamp = t;
  cmd.lateral.steering_tire_angle = ackermann_cmd.steer;
  cmd.lateral.steering_tire_rotation_rate = ackermann_cmd.steer_rate;
  cmd.longitudinal.stamp = t;
  cmd.longitudinal.velocity = ackermann_cmd.vel;
  cmd.longitudinal.acceleration = ackermann_cmd.acc;
  cmd.longitudinal.jerk = ackermann_cmd.jerk;
  return cmd;
}

/**
 * @brief Generate an ActuationCommandStamped message
 * @param [in] t timestamp
 * @param [in] accel_cmd accel actuation command
 * @param [in] brake_cmd brake actuation command
 * @param [in] steer_cmd steer actuation command
 */
ActuationCommandStamped actuationCmdGen(
  const builtin_interfaces::msg::Time & t, const Actuation & actuation_cmd)
{
  ActuationCommandStamped cmd;
  cmd.header.stamp = t;
  cmd.actuation.accel_cmd = actuation_cmd.accel;
  cmd.actuation.brake_cmd = actuation_cmd.brake;
  cmd.actuation.steer_cmd = actuation_cmd.steer;
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
 * @param [in] cmd_orig command to publish
 * @param [in] sim_node pointer to the simulation node
 * @param [in] pub_sub_node pointer to the node used for communication
 */
void sendAckermannCommand(
  const Control & cmd_orig, rclcpp::Node::SharedPtr sim_node,
  std::shared_ptr<PubSubNode> pub_sub_node)
{
  auto cmd = cmd_orig;
  for (int i = 0; i < 150; ++i) {
    cmd.stamp = sim_node->now();
    pub_sub_node->pub_ackermann_command_->publish(cmd);
    rclcpp::spin_some(sim_node);
    rclcpp::spin_some(pub_sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
  }
}

void sendActuationCommand(
  const ActuationCommandStamped & cmd_orig, rclcpp::Node::SharedPtr sim_node,
  std::shared_ptr<PubSubNode> pub_sub_node)
{
  auto cmd = cmd_orig;
  for (int i = 0; i < 150; ++i) {
    cmd.header.stamp = sim_node->now();
    pub_sub_node->pub_actuation_command_->publish(cmd);
    rclcpp::spin_some(sim_node);
    rclcpp::spin_some(pub_sub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
  }
}

void sendCommand(
  const CommandType & cmd_type, rclcpp::Node::SharedPtr sim_node,
  std::shared_ptr<PubSubNode> pub_sub_node, const builtin_interfaces::msg::Time & t,
  const Ackermann & ackermann_cmd, const Actuation & actuation_cmd)
{
  if (cmd_type == CommandType::Ackermann) {
    sendAckermannCommand(ackermannCmdGen(t, ackermann_cmd), sim_node, pub_sub_node);
  } else if (cmd_type == CommandType::Actuation) {
    sendActuationCommand(actuationCmdGen(t, actuation_cmd), sim_node, pub_sub_node);
  } else {
    throw std::invalid_argument("command type is unexpected.");
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
  std::cout << "isOnForward: dx: " << dx << ", forward_thr: " << forward_thr << std::endl;
  EXPECT_GT(dx, forward_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
}

void isOnBackward(const Odometry & state, const Odometry & init)
{
  double backward_thr = -1.0;
  double dx = state.pose.pose.position.x - init.pose.pose.position.x;
  std::cout << "isOnBackward: dx: " << dx << ", backward_thr: " << backward_thr << std::endl;
  EXPECT_LT(dx, backward_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
}

void isOnForwardLeft(const Odometry & state, const Odometry & init)
{
  double forward_thr = 1.0;
  double left_thr = 0.1f;
  double dx = state.pose.pose.position.x - init.pose.pose.position.x;
  double dy = state.pose.pose.position.y - init.pose.pose.position.y;
  std::cout << "isOnForwardLeft: dx: " << dx << ", forward_thr: " << forward_thr << ", dy: " << dy
            << ", left_thr: " << left_thr << std::endl;
  EXPECT_GT(dx, forward_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
  EXPECT_GT(dy, left_thr) << "[curr] " << toStrInfo(state) << ", [init] " << toStrInfo(init);
}

void isOnBackwardRight(const Odometry & state, const Odometry & init)
{
  double backward_thr = -1.0;
  double right_thr = -0.1;
  double dx = state.pose.pose.position.x - init.pose.pose.position.x;
  double dy = state.pose.pose.position.y - init.pose.pose.position.y;
  std::cout << "isOnBackwardRight: dx: " << dx << ", backward_thr: " << backward_thr
            << ", dy: " << dy << ", right_thr: " << right_thr << std::endl;
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

using DefaultParamType = std::tuple<CommandType, std::string>;
using ActuationCmdParamType = std::tuple<CommandType, std::string, std::string>;
using ParamType = std::variant<DefaultParamType, ActuationCmdParamType>;
std::unordered_map<std::string, std::type_index> vehicle_model_type_map = {
  {"IDEAL_STEER_VEL", typeid(DefaultParamType)},
  {"IDEAL_STEER_ACC", typeid(DefaultParamType)},
  {"IDEAL_STEER_ACC_GEARED", typeid(DefaultParamType)},
  {"DELAY_STEER_VEL", typeid(DefaultParamType)},
  {"DELAY_STEER_ACC", typeid(DefaultParamType)},
  {"DELAY_STEER_ACC_GEARED", typeid(DefaultParamType)},
  {"DELAY_STEER_ACC_GEARED_WO_FALL_GUARD", typeid(DefaultParamType)},
  {"ACTUATION_CMD", typeid(ActuationCmdParamType)}};

std::pair<CommandType, std::string> get_common_params(const ParamType & params)
{
  return std::visit(
    [](const auto & param) -> std::pair<CommandType, std::string> {
      return std::make_pair(std::get<0>(param), std::get<1>(param));
    },
    params);
}

// Send a control command and run the simulation.
// Then check if the vehicle is moving in the desired direction.
class TestSimplePlanningSimulator : public ::testing::TestWithParam<ParamType>
{
};

TEST_P(TestSimplePlanningSimulator, TestIdealSteerVel)
{
  rclcpp::init(0, nullptr);

  const auto params = GetParam();
  // common parameters
  const auto common_params = get_common_params(params);
  const auto command_type = common_params.first;
  const auto vehicle_model_type = common_params.second;
  // optional parameters
  std::optional<std::string> conversion_type{};  // for ActuationCmdParamType

  // Determine the ParamType corresponding to vehicle_model_type and get the specific parameters.
  const auto iter = vehicle_model_type_map.find(vehicle_model_type);
  if (iter == vehicle_model_type_map.end()) {
    throw std::invalid_argument("Unexpected vehicle_model_type.");
  }
  if (iter->second == typeid(ActuationCmdParamType)) {
    conversion_type = std::get<2>(std::get<ActuationCmdParamType>(params));
  }

  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("initialize_source", "INITIAL_POSE_TOPIC");
  node_options.append_parameter_override("vehicle_model_type", vehicle_model_type);
  node_options.append_parameter_override("initial_engage_state", true);
  node_options.append_parameter_override("add_measurement_noise", false);
  node_options.append_parameter_override("accel_time_delay", 0.2);
  node_options.append_parameter_override("accel_time_constant", 0.2);
  node_options.append_parameter_override("brake_time_delay", 0.2);
  node_options.append_parameter_override("brake_time_constant", 0.2);
  node_options.append_parameter_override("convert_accel_cmd", true);
  node_options.append_parameter_override("convert_brake_cmd", true);
  node_options.append_parameter_override("convert_steer_cmd", true);
  const auto share_dir = ament_index_cpp::get_package_share_directory("simple_planning_simulator");
  const auto accel_map_path = share_dir + "/test/actuation_cmd_map/accel_map.csv";
  const auto brake_map_path = share_dir + "/test/actuation_cmd_map/brake_map.csv";
  const auto steer_map_path = share_dir + "/test/actuation_cmd_map/steer_map.csv";
  node_options.append_parameter_override("accel_map_path", accel_map_path);
  node_options.append_parameter_override("brake_map_path", brake_map_path);
  node_options.append_parameter_override("steer_map_path", steer_map_path);
  node_options.append_parameter_override("vgr_coef_a", 15.713);
  node_options.append_parameter_override("vgr_coef_b", 0.053);
  node_options.append_parameter_override("vgr_coef_c", 0.042);
  if (conversion_type.has_value()) {
    std::cout << "\n\n vehicle model = " << vehicle_model_type
              << ", conversion_type = " << conversion_type.value() << std::endl
              << std::endl;
    node_options.append_parameter_override("convert_steer_cmd_method", conversion_type.value());
  } else {
    std::cout << "\n\n vehicle model = " << vehicle_model_type << std::endl << std::endl;
  }

  declareVehicleInfoParams(node_options);
  const auto sim_node = std::make_shared<SimplePlanningSimulator>(node_options);

  const auto pub_sub_node = std::make_shared<PubSubNode>();

  const double target_vel = 5.0f;
  const double target_acc = 5.0f;
  const double target_steer = 0.2f;

  // NOTE: As the value of the actuation map is known, roughly determine whether it is
  // acceleration or braking, and whether it turns left or right, and generate an actuation
  // command. So do not change the map. If it is necessary, you need to change this parameters as
  // well.
  const double target_steer_actuation = 10.0f;
  const double target_accel_actuation = 0.5f;
  // const double target_brake_actuation = 0.5f;  // unused for now.

  auto _resetInitialpose = [&]() { resetInitialpose(sim_node, pub_sub_node); };
  auto _sendFwdGear = [&]() { sendGear(GearCommand::DRIVE, sim_node, pub_sub_node); };
  auto _sendBwdGear = [&]() { sendGear(GearCommand::REVERSE, sim_node, pub_sub_node); };
  auto _sendCommand = [&](auto ackermann_cmd, auto actuation_cmd) {
    const auto t = sim_node->now();
    sendCommand(command_type, sim_node, pub_sub_node, t, ackermann_cmd, actuation_cmd);
  };

  // check pub-sub connections
  {
    size_t expected = 1;
    // actuation or ackermann must be subscribed
    const auto sub_command_count =
      (command_type == CommandType::Actuation)
        ? pub_sub_node->pub_actuation_command_->get_subscription_count()
        : pub_sub_node->pub_ackermann_command_->get_subscription_count();
    EXPECT_EQ(sub_command_count, expected);
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
  _sendCommand(
    Ackermann{0.0f, 0.0f, target_vel, target_acc, 0.0f},
    Actuation{0.0f, target_accel_actuation, 0.0f});
  isOnForward(*(pub_sub_node->current_odom_), init_state);

  // go backward
  // NOTE: positive acceleration with reverse gear drives the vehicle backward.
  _resetInitialpose();
  _sendBwdGear();
  _sendCommand(
    Ackermann{0.0f, 0.0f, -target_vel, target_acc, 0.0f},
    Actuation{0.0f, target_accel_actuation, 0.0f});
  isOnBackward(*(pub_sub_node->current_odom_), init_state);

  // go forward left
  _resetInitialpose();
  _sendFwdGear();
  _sendCommand(
    Ackermann{target_steer, 0.0f, target_vel, target_acc, 0.0f},
    Actuation{target_steer_actuation, target_accel_actuation, 0.0f});
  isOnForwardLeft(*(pub_sub_node->current_odom_), init_state);

  // go backward right
  // NOTE: positive acceleration with reverse gear drives the vehicle backward.
  _resetInitialpose();
  _sendBwdGear();
  _sendCommand(
    Ackermann{-target_steer, 0.0f, -target_vel, target_acc, 0.0f},
    Actuation{-target_steer_actuation, target_accel_actuation, 0.0f});
  isOnBackwardRight(*(pub_sub_node->current_odom_), init_state);

  rclcpp::shutdown();
}

INSTANTIATE_TEST_SUITE_P(
  TestForEachVehicleModelTrue, TestSimplePlanningSimulator,
  ::testing::Values(
    /* Ackermann type */
    std::make_tuple(CommandType::Ackermann, "IDEAL_STEER_VEL"),
    std::make_tuple(CommandType::Ackermann, "IDEAL_STEER_ACC"),
    std::make_tuple(CommandType::Ackermann, "IDEAL_STEER_ACC_GEARED"),
    std::make_tuple(CommandType::Ackermann, "DELAY_STEER_VEL"),
    std::make_tuple(CommandType::Ackermann, "DELAY_STEER_ACC"),
    std::make_tuple(CommandType::Ackermann, "DELAY_STEER_ACC_GEARED"),
    std::make_tuple(CommandType::Ackermann, "DELAY_STEER_ACC_GEARED_WO_FALL_GUARD"),
    /* Actuation type */
    std::make_tuple(CommandType::Actuation, "ACTUATION_CMD", "steer_map"),
    std::make_tuple(CommandType::Actuation, "ACTUATION_CMD", "vgr")));
