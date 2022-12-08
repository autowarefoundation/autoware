// Copyright 2021 TierIV
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

#include "stop_filter/stop_filter.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>

using std::placeholders::_1;

StopFilter::StopFilter(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options)
{
  vx_threshold_ = declare_parameter("vx_threshold", 0.01);
  wz_threshold_ = declare_parameter("wz_threshold", 0.01);

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1, std::bind(&StopFilter::callbackOdometry, this, _1));

  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("output/odom", 1);
  pub_stop_flag_ = create_publisher<tier4_debug_msgs::msg::BoolStamped>("debug/stop_flag", 1);
}

void StopFilter::callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tier4_debug_msgs::msg::BoolStamped stop_flag_msg;
  stop_flag_msg.stamp = msg->header.stamp;
  stop_flag_msg.data = false;

  nav_msgs::msg::Odometry odom_msg;
  odom_msg = *msg;

  if (
    std::fabs(msg->twist.twist.linear.x) < vx_threshold_ &&
    std::fabs(msg->twist.twist.angular.z) < wz_threshold_) {
    odom_msg.twist.twist.linear.x = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
    stop_flag_msg.data = true;
  }

  pub_stop_flag_->publish(stop_flag_msg);
  pub_odom_->publish(odom_msg);
}
