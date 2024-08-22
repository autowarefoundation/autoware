// Copyright 2023 TIER IV, Inc.
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

#include "gyro_odometer_core.hpp"
#include "test_gyro_odometer_helper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

/*
 * This test checks if twist is published from gyro_odometer
 */
using geometry_msgs::msg::TwistWithCovarianceStamped;
using sensor_msgs::msg::Imu;

class ImuGenerator : public rclcpp::Node
{
public:
  ImuGenerator() : Node("imu_generator"), imu_pub(create_publisher<Imu>("/imu", 1)) {}
  rclcpp::Publisher<Imu>::SharedPtr imu_pub;
};

class VelocityGenerator : public rclcpp::Node
{
public:
  VelocityGenerator()
  : Node("velocity_generator"),
    vehicle_velocity_pub(
      create_publisher<TwistWithCovarianceStamped>("/vehicle/twist_with_covariance", 1))
  {
  }
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr vehicle_velocity_pub;
};

class GyroOdometerValidator : public rclcpp::Node
{
public:
  GyroOdometerValidator()
  : Node("gyro_odometer_validator"),
    twist_sub(create_subscription<TwistWithCovarianceStamped>(
      "/twist_with_covariance", 1,
      [this](const TwistWithCovarianceStamped::ConstSharedPtr msg) {
        received_latest_twist_ptr = msg;
      })),
    received_latest_twist_ptr(nullptr)
  {
  }

  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr twist_sub;
  TwistWithCovarianceStamped::ConstSharedPtr received_latest_twist_ptr;
};

void wait_spin_some(rclcpp::Node::SharedPtr node_ptr)
{
  for (int i = 0; i < 50; ++i) {
    rclcpp::spin_some(node_ptr);
    rclcpp::WallRate(100).sleep();
  }
}

bool is_twist_valid(
  const TwistWithCovarianceStamped & twist, const TwistWithCovarianceStamped & twist_ground_truth)
{
  if (twist.twist.twist.linear.x != twist_ground_truth.twist.twist.linear.x) {
    return false;
  }
  if (twist.twist.twist.linear.y != twist_ground_truth.twist.twist.linear.y) {
    return false;
  }
  if (twist.twist.twist.linear.z != twist_ground_truth.twist.twist.linear.z) {
    return false;
  }
  if (twist.twist.twist.angular.x != twist_ground_truth.twist.twist.angular.x) {
    return false;
  }
  if (twist.twist.twist.angular.y != twist_ground_truth.twist.twist.angular.y) {
    return false;
  }
  if (twist.twist.twist.angular.z != twist_ground_truth.twist.twist.angular.z) {
    return false;
  }
  return true;
}

// IMU & Velocity test
// Verify that the gyro_odometer successfully publishes the fused twist message when both IMU and
// velocity data are provided
TEST(GyroOdometer, TestGyroOdometerWithImuAndVelocity)
{
  Imu input_imu = generate_sample_imu();
  TwistWithCovarianceStamped input_velocity = generate_sample_velocity();

  TwistWithCovarianceStamped expected_output_twist;
  expected_output_twist.twist.twist.linear.x = input_velocity.twist.twist.linear.x;
  expected_output_twist.twist.twist.angular.x = input_imu.angular_velocity.x;
  expected_output_twist.twist.twist.angular.y = input_imu.angular_velocity.y;
  expected_output_twist.twist.twist.angular.z = input_imu.angular_velocity.z;

  auto gyro_odometer_node = std::make_shared<autoware::gyro_odometer::GyroOdometerNode>(
    get_node_options_with_default_params());
  auto imu_generator = std::make_shared<ImuGenerator>();
  auto velocity_generator = std::make_shared<VelocityGenerator>();
  auto gyro_odometer_validator_node = std::make_shared<GyroOdometerValidator>();

  velocity_generator->vehicle_velocity_pub->publish(
    input_velocity);  // need this for now, which should eventually be removed
  imu_generator->imu_pub->publish(input_imu);
  velocity_generator->vehicle_velocity_pub->publish(input_velocity);

  // gyro_odometer receives IMU and velocity, and publishes the fused twist data.
  wait_spin_some(gyro_odometer_node);

  // validator node receives the fused twist data and store in "received_latest_twist_ptr".
  wait_spin_some(gyro_odometer_validator_node);

  EXPECT_FALSE(gyro_odometer_validator_node->received_latest_twist_ptr == nullptr);
  EXPECT_TRUE(is_twist_valid(
    *(gyro_odometer_validator_node->received_latest_twist_ptr), expected_output_twist));
}

// IMU-only test
// Verify that the gyro_odometer does NOT publish any outputs when only IMU is provided
TEST(GyroOdometer, TestGyroOdometerImuOnly)
{
  Imu input_imu = generate_sample_imu();

  auto gyro_odometer_node = std::make_shared<autoware::gyro_odometer::GyroOdometerNode>(
    get_node_options_with_default_params());
  auto imu_generator = std::make_shared<ImuGenerator>();
  auto gyro_odometer_validator_node = std::make_shared<GyroOdometerValidator>();

  imu_generator->imu_pub->publish(input_imu);

  // gyro_odometer receives IMU
  wait_spin_some(gyro_odometer_node);

  // validator node waits for the output fused twist from gyro_odometer
  wait_spin_some(gyro_odometer_validator_node);

  EXPECT_TRUE(gyro_odometer_validator_node->received_latest_twist_ptr == nullptr);
}
