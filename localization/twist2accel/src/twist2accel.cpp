// Copyright 2022 TIER IV
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

#include "twist2accel/twist2accel.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
#define DEBUG_PRINT_MAT(X) {if (show_debug_info_) {std::cout << #X << ": " << X << std::endl;}}

// clang-format on
using std::placeholders::_1;

Twist2Accel::Twist2Accel(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1, std::bind(&Twist2Accel::callbackOdometry, this, _1));
  sub_twist_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "input/twist", 1, std::bind(&Twist2Accel::callbackTwistWithCovariance, this, _1));

  pub_accel_ = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>("output/accel", 1);

  prev_twist_ptr_ = nullptr;
  accel_lowpass_gain_ = declare_parameter("accel_lowpass_gain", 0.5);
  use_odom_ = declare_parameter("use_odom", true);

  lpf_alx_ptr_ = std::make_shared<LowpassFilter1d>(accel_lowpass_gain_);
  lpf_aly_ptr_ = std::make_shared<LowpassFilter1d>(accel_lowpass_gain_);
  lpf_alz_ptr_ = std::make_shared<LowpassFilter1d>(accel_lowpass_gain_);
  lpf_aax_ptr_ = std::make_shared<LowpassFilter1d>(accel_lowpass_gain_);
  lpf_aay_ptr_ = std::make_shared<LowpassFilter1d>(accel_lowpass_gain_);
  lpf_aaz_ptr_ = std::make_shared<LowpassFilter1d>(accel_lowpass_gain_);
}

void Twist2Accel::callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!use_odom_) return;

  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;
  estimateAccel(std::make_shared<geometry_msgs::msg::TwistStamped>(twist));
}

void Twist2Accel::callbackTwistWithCovariance(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  if (use_odom_) return;

  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;
  estimateAccel(std::make_shared<geometry_msgs::msg::TwistStamped>(twist));
}

void Twist2Accel::estimateAccel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  geometry_msgs::msg::AccelWithCovarianceStamped accel_msg;
  accel_msg.header = msg->header;

  if (prev_twist_ptr_ != nullptr) {
    const double dt = std::max(
      (rclcpp::Time(msg->header.stamp) - rclcpp::Time(prev_twist_ptr_->header.stamp)).seconds(),
      1.0e-3);

    double alx = (msg->twist.linear.x - prev_twist_ptr_->twist.linear.x) / dt;
    double aly = (msg->twist.linear.y - prev_twist_ptr_->twist.linear.y) / dt;
    double alz = (msg->twist.linear.z - prev_twist_ptr_->twist.linear.z) / dt;
    double aax = (msg->twist.angular.x - prev_twist_ptr_->twist.angular.x) / dt;
    double aay = (msg->twist.angular.y - prev_twist_ptr_->twist.angular.y) / dt;
    double aaz = (msg->twist.angular.z - prev_twist_ptr_->twist.angular.z) / dt;

    accel_msg.accel.accel.linear.x = lpf_alx_ptr_->filter(alx);
    accel_msg.accel.accel.linear.y = lpf_aly_ptr_->filter(aly);
    accel_msg.accel.accel.linear.z = lpf_alz_ptr_->filter(alz);
    accel_msg.accel.accel.angular.x = lpf_aax_ptr_->filter(aax);
    accel_msg.accel.accel.angular.y = lpf_aay_ptr_->filter(aay);
    accel_msg.accel.accel.angular.z = lpf_aaz_ptr_->filter(aaz);

    // Ideally speaking, these covariance should be properly estimated.
    accel_msg.accel.covariance[0 * 6 + 0] = 1.0;
    accel_msg.accel.covariance[1 * 6 + 1] = 1.0;
    accel_msg.accel.covariance[2 * 6 + 2] = 1.0;
    accel_msg.accel.covariance[3 * 6 + 3] = 0.05;
    accel_msg.accel.covariance[4 * 6 + 4] = 0.05;
    accel_msg.accel.covariance[5 * 6 + 5] = 0.05;
  }

  pub_accel_->publish(accel_msg);
  prev_twist_ptr_ = msg;
}
