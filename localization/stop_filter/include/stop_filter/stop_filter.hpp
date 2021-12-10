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

#ifndef STOP_FILTER__STOP_FILTER_HPP_
#define STOP_FILTER__STOP_FILTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_debug_msgs/msg/bool_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

class StopFilter : public rclcpp::Node
{
public:
  StopFilter(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;  //!< @brief odom publisher
  rclcpp::Publisher<tier4_debug_msgs::msg::BoolStamped>::SharedPtr
    pub_stop_flag_;  //!< @brief stop flag publisher
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    sub_odom_;  //!< @brief measurement odometry subscriber

  double vx_threshold_;  //!< @brief vx threshold
  double wz_threshold_;  //!< @brief wz threshold

  /**
   * @brief set odometry measurement
   */
  void callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
};
#endif  // STOP_FILTER__STOP_FILTER_HPP_
