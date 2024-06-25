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

#include "../../src/vehicle_cmd_gate.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#define ASSERT_LT_NEAR(x, y, alpha) ASSERT_LT(x, y * alpha)
#define ASSERT_GT_NEAR(x, y, alpha) ASSERT_GT(x, y * alpha)

#define PRINT_VALUES(...) print_values(0, #__VA_ARGS__, __VA_ARGS__)
template <typename T>
void print_values([[maybe_unused]] int i, [[maybe_unused]] T name)
{
  std::cerr << std::endl;
}
template <typename T1, typename T2, typename... T3>
void print_values(int i, const T1 & name, const T2 & a, const T3 &... b)
{
  for (; name[i] != ',' && name[i] != '\0'; i++) std::cerr << name[i];

  std::ostringstream oss;
  oss << std::setprecision(4) << std::setw(9) << a;
  std::cerr << ":" << oss.str() << " ";
  print_values(i + 1, name, b...);
}

// global params
const std::vector<double> reference_speed_points = {5., 10., 15., 20.};
const std::vector<double> steer_lim = {0.5, 0.3, 0.2, 0.1};
const std::vector<double> steer_rate_lim = {0.5, 0.3, 0.2, 0.1};
const std::vector<double> lon_acc_lim = {1.5, 1.0, 0.8, 0.6};
const std::vector<double> lon_jerk_lim = {1.4, 0.9, 0.7, 0.5};
const std::vector<double> lat_acc_lim = {2.0, 1.6, 1.2, 0.8};
const std::vector<double> lat_jerk_lim = {1.7, 1.3, 0.9, 0.6};
const std::vector<double> actual_steer_diff_lim = {0.5, 0.4, 0.2, 0.1};
const double wheelbase = 2.89;

using autoware::vehicle_cmd_gate::VehicleCmdGate;

using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_control_msgs::msg::Control;
using autoware_vehicle_msgs::msg::GearCommand;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using tier4_control_msgs::msg::GateMode;
using tier4_external_api_msgs::msg::Emergency;
using tier4_external_api_msgs::msg::Heartbeat;
using EngageMsg = autoware_vehicle_msgs::msg::Engage;

class PubSubNode : public rclcpp::Node
{
public:
  PubSubNode() : Node{"test_vehicle_cmd_gate_filter_pubsub"}
  {
    sub_cmd_ = create_subscription<Control>(
      "output/control_cmd", rclcpp::QoS{1}, [this](const Control::ConstSharedPtr msg) {
        cmd_history_.push_back(msg);
        cmd_received_times_.push_back(now());
        // check the effectiveness of the filter for last x elements in the queue
        checkFilter(4);
      });

    rclcpp::QoS qos{1};
    qos.transient_local();

    pub_external_emergency_stop_heartbeat_ =
      create_publisher<Heartbeat>("input/external_emergency_stop_heartbeat", qos);
    pub_engage_ = create_publisher<EngageMsg>("input/engage", qos);
    pub_gate_mode_ = create_publisher<GateMode>("input/gate_mode", qos);
    pub_odom_ = create_publisher<Odometry>("/localization/kinematic_state", qos);
    pub_acc_ = create_publisher<AccelWithCovarianceStamped>("input/acceleration", qos);
    pub_steer_ = create_publisher<SteeringReport>("input/steering", qos);
    pub_operation_mode_ = create_publisher<OperationModeState>("input/operation_mode", qos);
    pub_mrm_state_ = create_publisher<MrmState>("input/mrm_state", qos);

    pub_auto_control_cmd_ = create_publisher<Control>("input/auto/control_cmd", qos);
    pub_auto_turn_indicator_cmd_ =
      create_publisher<TurnIndicatorsCommand>("input/auto/turn_indicators_cmd", qos);
    pub_auto_hazard_light_cmd_ =
      create_publisher<HazardLightsCommand>("input/auto/hazard_lights_cmd", qos);
    pub_auto_gear_cmd_ = create_publisher<GearCommand>("input/auto/gear_cmd", qos);
  }

  rclcpp::Subscription<Control>::SharedPtr sub_cmd_;

  rclcpp::Publisher<Heartbeat>::SharedPtr pub_external_emergency_stop_heartbeat_;
  rclcpp::Publisher<EngageMsg>::SharedPtr pub_engage_;
  rclcpp::Publisher<GateMode>::SharedPtr pub_gate_mode_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr pub_acc_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steer_;
  rclcpp::Publisher<OperationModeState>::SharedPtr pub_operation_mode_;
  rclcpp::Publisher<MrmState>::SharedPtr pub_mrm_state_;
  rclcpp::Publisher<Control>::SharedPtr pub_auto_control_cmd_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr pub_auto_turn_indicator_cmd_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr pub_auto_hazard_light_cmd_;
  rclcpp::Publisher<GearCommand>::SharedPtr pub_auto_gear_cmd_;

  std::vector<Control::ConstSharedPtr> cmd_history_;
  std::vector<Control::ConstSharedPtr> raw_cmd_history_;
  std::vector<rclcpp::Time> cmd_received_times_;

  // publish except for the control_cmd
  void publishDefaultTopicsNoSpin()
  {
    {
      Heartbeat msg;
      msg.stamp = now();
      pub_external_emergency_stop_heartbeat_->publish(msg);
    }
    {
      EngageMsg msg;
      msg.stamp = now();
      msg.engage = true;
      pub_engage_->publish(msg);
    }
    {
      GateMode msg;
      msg.data = GateMode::AUTO;
      pub_gate_mode_->publish(msg);
    }
    {
      Odometry msg;  // initialized for zero pose and twist
      msg.header.frame_id = "baselink";
      msg.header.stamp = now();
      msg.pose.pose.orientation.w = 1.0;
      msg.twist.twist.linear.x = 0.0;
      if (!cmd_history_.empty()) {  // ego moves as commanded.
        msg.twist.twist.linear.x =
          cmd_history_.back()->longitudinal.velocity;  // ego moves as commanded.
      }
      pub_odom_->publish(msg);
    }
    {
      AccelWithCovarianceStamped msg;
      msg.header.frame_id = "baselink";
      msg.header.stamp = now();
      msg.accel.accel.linear.x = 0.0;
      if (!cmd_history_.empty()) {  // ego moves as commanded.
        msg.accel.accel.linear.x = cmd_history_.back()->longitudinal.acceleration;
      }
      pub_acc_->publish(msg);
    }
    {
      SteeringReport msg;
      msg.stamp = now();
      msg.steering_tire_angle = 0.0;
      if (!cmd_history_.empty()) {  // ego moves as commanded.
        msg.steering_tire_angle = cmd_history_.back()->lateral.steering_tire_angle;
      }
      pub_steer_->publish(msg);
    }
    {
      OperationModeState msg;
      msg.stamp = now();
      msg.mode = OperationModeState::AUTONOMOUS;
      msg.is_autoware_control_enabled = true;
      pub_operation_mode_->publish(msg);
    }
    {
      MrmState msg;
      msg.stamp = now();
      msg.state = MrmState::NORMAL;
      msg.behavior = MrmState::NONE;
      pub_mrm_state_->publish(msg);
    }
    {
      TurnIndicatorsCommand msg;
      msg.stamp = now();
      msg.command = TurnIndicatorsCommand::DISABLE;
      pub_auto_turn_indicator_cmd_->publish(msg);
    }
    {
      HazardLightsCommand msg;
      msg.stamp = now();
      msg.command = HazardLightsCommand::DISABLE;
      pub_auto_hazard_light_cmd_->publish(msg);
    }
    {
      GearCommand msg;
      msg.stamp = now();
      msg.command = GearCommand::DRIVE;
      pub_auto_gear_cmd_->publish(msg);
    }
  }

  void publishControlCommand(Control msg)
  {
    msg.stamp = now();
    pub_auto_control_cmd_->publish(msg);
    raw_cmd_history_.push_back(std::make_shared<Control>(msg));
  }

  void checkFilter(size_t last_x)
  {
    ASSERT_TRUE(wheelbase > 0.0) << "Wheelbase must be positive. Check vehicle configuration.";
    ASSERT_GT(last_x, 1u) << "last_x must be greater than 1 to calculate the jerk values";
    ASSERT_EQ(cmd_history_.size(), cmd_received_times_.size())
      << "cmd_history_ and cmd_received_times_ must have the same size. Check the implementation.";
    size_t history_size = cmd_history_.size();
    if (history_size < last_x) return;  // not enough data for checkFilter or last_x is too small.

    // Initialize accumulators
    double avg_lon_acc = 0.0;
    double avg_lon_jerk = 0.0;
    double avg_lat_acc = 0.0;
    double avg_lat_jerk = 0.0;

    double lon_vel = 0.0;
    double prev_lon_acc = 0.0;
    double prev_lat_acc = 0.0;
    // TODO(Horibe): Remove this variable when the filtering logic is fixed.
    double prev_tire_angle = 0.0;

    auto calculate_lateral_acceleration =
      [](const double lon_vel, const double steer, const double wheelbase) {
        return lon_vel * lon_vel * std::tan(steer) / wheelbase;
      };

    size_t ind_start = history_size - last_x + 1;
    {
      const auto & cmd_start = cmd_history_.at(ind_start - 1);
      prev_lon_acc = cmd_start->longitudinal.acceleration;
      // TODO(Horibe): prev_lat_acc should use the previous velocity, but use the current velocity
      // since the current filtering logic uses the current velocity.
      // when it's fixed, should be like this:
      // prev_lat_acc = calculate_lateral_acceleration(cmd_start->longitudinal.velocity,
      // cmd_start->lateral.steering_tire_angle, wheelbase);
      prev_tire_angle = cmd_start->lateral.steering_tire_angle;
    }

    for (size_t i = ind_start; i < history_size; ++i) {
      const auto & cmd = cmd_history_.at(i);

      const auto dt = (cmd_received_times_.at(i) - cmd_received_times_.at(i - 1)).seconds();

      ASSERT_GT(dt, 0.0) << "Invalid dt. Time must be strictly positive.";

      lon_vel = cmd->longitudinal.velocity;
      const auto lon_acc = cmd->longitudinal.acceleration;
      const auto lon_jerk = (lon_acc - prev_lon_acc) / dt;

      const auto lat_acc =
        calculate_lateral_acceleration(lon_vel, cmd->lateral.steering_tire_angle, wheelbase);
      // TODO(Horibe): prev_lat_acc should use the previous velocity, but use the current velocity
      // since the current filtering logic uses the current velocity.
      // when it's fixed, it should be moved to the bottom of this loop.
      prev_lat_acc = calculate_lateral_acceleration(lon_vel, prev_tire_angle, wheelbase);
      const auto lat_jerk = (lat_acc - prev_lat_acc) / dt;

      avg_lon_acc += lon_acc;
      avg_lon_jerk += lon_jerk;
      avg_lat_acc += lat_acc;
      avg_lat_jerk += lat_jerk;

      prev_lon_acc = lon_acc;
      // TODO(Horibe): when the filtering logic is fixed, this line should be removed.
      prev_tire_angle = cmd->lateral.steering_tire_angle;
    }

    // Compute averages
    // Because we look at differences, we have one less interval than points
    double denominator = static_cast<double>(last_x) - 1;
    avg_lon_acc /= denominator;
    avg_lon_jerk /= denominator;
    avg_lat_acc /= denominator;
    avg_lat_jerk /= denominator;

    const auto max_lon_acc_lim = *std::max_element(lon_acc_lim.begin(), lon_acc_lim.end());
    const auto max_lon_jerk_lim = *std::max_element(lon_jerk_lim.begin(), lon_jerk_lim.end());
    const auto max_lat_acc_lim = *std::max_element(lat_acc_lim.begin(), lat_acc_lim.end());
    const auto max_lat_jerk_lim = *std::max_element(lat_jerk_lim.begin(), lat_jerk_lim.end());

    // This test is designed to verify that the filter is applied correctly. However, if topic
    // communication is delayed, the allowable range of change in the command values between the
    // sender and receiver of the topic will vary, making the results dependent on processing time.
    // We define the allowable error margin to account for this.
    constexpr auto threshold_scale = 1.1;

    // Output command must be smaller than maximum limit.
    // TODO(Horibe): check for each velocity range.
    if (std::abs(lon_vel) > 0.01) {
      // Assert over averaged values against limits
      ASSERT_LT_NEAR(std::abs(avg_lon_acc), max_lon_acc_lim, threshold_scale)
        << "last_x was = " << last_x;
      ASSERT_LT_NEAR(std::abs(avg_lon_jerk), max_lon_jerk_lim, threshold_scale)
        << "last_x was = " << last_x;
      ASSERT_LT_NEAR(std::abs(avg_lat_acc), max_lat_acc_lim, threshold_scale)
        << "last_x was = " << last_x;
      ASSERT_LT_NEAR(std::abs(avg_lat_jerk), max_lat_jerk_lim, threshold_scale)
        << "last_x was = " << last_x;
    }
  }
};

struct CmdParam
{
  double max;
  double freq;
  double bias;
  CmdParam() {}
  CmdParam(double m, double f, double b) : max(m), freq(f), bias(b) {}
};

struct CmdParams
{
  CmdParam steering;
  CmdParam velocity;
  CmdParam acceleration;
  CmdParams() {}
  CmdParams(CmdParam s, CmdParam v, CmdParam a) : steering(s), velocity(v), acceleration(a) {}
};

class ControlCmdGenerator
{
public:
  CmdParams p_;  // used for sin wave command generation

  using Clock = std::chrono::high_resolution_clock;
  std::chrono::time_point<Clock> start_time_{Clock::now()};

  // generate ControlCommand with sin wave format.
  // TODO(Horibe): implement steering_rate and jerk command.
  Control calcSinWaveCommand(bool reset_clock = false)
  {
    if (reset_clock) {
      start_time_ = Clock::now();
    }

    const auto dt_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start_time_);
    const auto dt_s = dt_ns.count() / 1e9;

    const auto sinWave = [&](auto amp, auto freq, auto bias) {
      return amp * std::sin(2.0 * M_PI * freq * dt_s + bias);
    };

    Control cmd;
    cmd.lateral.steering_tire_angle = sinWave(p_.steering.max, p_.steering.freq, p_.steering.bias);
    cmd.longitudinal.velocity =
      sinWave(p_.velocity.max, p_.velocity.freq, p_.velocity.bias) + p_.velocity.max;
    cmd.longitudinal.acceleration =
      sinWave(p_.acceleration.max, p_.acceleration.freq, p_.acceleration.bias);

    return cmd;
  }
};

std::shared_ptr<VehicleCmdGate> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};

  const auto vehicle_cmd_gate_dir =
    ament_index_cpp::get_package_share_directory("autoware_vehicle_cmd_gate");
  const auto vehicle_info_util_dir =
    ament_index_cpp::get_package_share_directory("autoware_vehicle_info_utils");

  node_options.arguments(
    {"--ros-args", "--params-file", vehicle_cmd_gate_dir + "/config/vehicle_cmd_gate.param.yaml",
     "--ros-args", "--params-file", vehicle_info_util_dir + "/config/vehicle_info.param.yaml"});

  node_options.append_parameter_override(
    "check_external_emergency_heartbeat", true);  // This parameter has to be set when launching.

  const auto override = [&](const auto s, const std::vector<double> v) {
    node_options.append_parameter_override<std::vector<double>>(s, v);
  };

  node_options.append_parameter_override("wheel_base", wheelbase);
  override("nominal.reference_speed_points", reference_speed_points);
  override("nominal.reference_speed_points", reference_speed_points);
  override("nominal.steer_lim", steer_lim);
  override("nominal.steer_rate_lim", steer_rate_lim);
  override("nominal.lon_acc_lim", lon_acc_lim);
  override("nominal.lon_jerk_lim", lon_jerk_lim);
  override("nominal.lat_acc_lim", lat_acc_lim);
  override("nominal.lat_jerk_lim", lat_jerk_lim);
  override("nominal.actual_steer_diff_lim", actual_steer_diff_lim);

  return std::make_shared<VehicleCmdGate>(node_options);
}

class TestFixture : public ::testing::TestWithParam<CmdParams>
{
protected:
  void SetUp() override
  {
    vehicle_cmd_gate_node_ = generateNode();
    cmd_generator_.p_ = GetParam();
  }

  void TearDown() override
  {
    // rclcpp::shutdown();
  }

  PubSubNode pub_sub_node_;
  std::shared_ptr<VehicleCmdGate> vehicle_cmd_gate_node_;
  ControlCmdGenerator cmd_generator_;
};

TEST_P(TestFixture, CheckFilterForSinCmd)
{
  [[maybe_unused]] auto a = std::system("ros2 node list");
  [[maybe_unused]] auto b = std::system("ros2 node info /test_vehicle_cmd_gate_filter_pubsub");
  [[maybe_unused]] auto c = std::system("ros2 node info /vehicle_cmd_gate");

  // std::cerr << "speed signal: " << cmd_generator_.p_.velocity.max << " * sin(2pi * "
  //           << cmd_generator_.p_.velocity.freq << " * dt + " << cmd_generator_.p_.velocity.bias
  //           << ")" << std::endl;
  // std::cerr << "accel signal: " << cmd_generator_.p_.acceleration.max << " * sin(2pi * "
  //           << cmd_generator_.p_.acceleration.freq << " * dt + "
  //           << cmd_generator_.p_.acceleration.bias << ")" << std::endl;
  // std::cerr << "steer signal: " << cmd_generator_.p_.steering.max << " * sin(2pi * "
  //           << cmd_generator_.p_.steering.freq << " * dt + " << cmd_generator_.p_.steering.bias
  //           << ")" << std::endl;

  for (size_t i = 0; i < 30; ++i) {
    auto start_time = std::chrono::steady_clock::now();

    const bool reset_clock = (i == 0);
    const auto cmd = cmd_generator_.calcSinWaveCommand(reset_clock);
    pub_sub_node_.publishControlCommand(cmd);
    pub_sub_node_.publishDefaultTopicsNoSpin();
    for (int j = 0; j < 20; ++j) {
      rclcpp::spin_some(pub_sub_node_.get_node_base_interface());
      rclcpp::spin_some(vehicle_cmd_gate_node_->get_node_base_interface());
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::milliseconds elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    // The value determines the period of the topic. The filter logic of vehicle_cmd_gate depends on
    // real-time, and if the time from publishing to subscribing becomes too long, this test will
    // fail (the test specification itself should be improved). To prevent processing bottlenecks,
    // please set this value appropriately. It is set to 30ms because it occasionally fails at 10ms.
    constexpr int running_ms = 30;

    std::chrono::milliseconds sleep_duration = std::chrono::milliseconds{running_ms} - elapsed;
    if (sleep_duration.count() > 0) {
      std::this_thread::sleep_for(sleep_duration);
    }
  }

  std::cerr << "received cmd num = " << pub_sub_node_.cmd_received_times_.size() << std::endl;
};

// High frequency, large value
CmdParams p1 = {/*steer*/ {0.5, 1, 0}, /*velocity*/ {10, 0.0, 0}, /*acc*/ {5, 1.5, 2}};
INSTANTIATE_TEST_SUITE_P(TestParam1, TestFixture, ::testing::Values(p1));

// High frequency, normal value
CmdParams p2 = {/*steer*/ {0.5, 2, 1}, /*velocity*/ {5, 1.0, 0}, /*acc*/ {2.0, 3.0, 2}};
INSTANTIATE_TEST_SUITE_P(TestParam2, TestFixture, ::testing::Values(p2));

// High frequency, small value
CmdParams p3 = {/*steer*/ {0.5, 3, 2}, /*velocity*/ {2, 3, 0}, /*acc*/ {0.5, 3, 2}};
INSTANTIATE_TEST_SUITE_P(TestParam3, TestFixture, ::testing::Values(p3));

// Low frequency
CmdParams p4 = {/*steer*/ {0.5, 0.1, 0.5}, /*velocity*/ {10, 0.2, 0}, /*acc*/ {5, 0.1, 2}};
INSTANTIATE_TEST_SUITE_P(TestParam4, TestFixture, ::testing::Values(p4));

// Large steer, large velocity -> this test fails.
// Lateral acceleration and lateral jerk affect both steer and velocity, and if both steer and
// velocity changes significantly, the current logic cannot adequately handle the situation.
// CmdParams p5 = {/*steer*/ {10.0, 1.0, 0.5}, /*velocity*/ {10, 0.2, 0}, /*acc*/ {5, 0.1, 2}};
// INSTANTIATE_TEST_SUITE_P(TestParam5, TestFixture, ::testing::Values(p5));
