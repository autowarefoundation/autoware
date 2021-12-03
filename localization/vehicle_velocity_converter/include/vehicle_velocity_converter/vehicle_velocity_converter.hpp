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

#ifndef _VEHICLE_VELOCITY_CONVERTER_HPP_
#define _VEHICLE_VELOCITY_CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <array>
#include <string>
#include <vector>

class VehicleVelocityConverter : public rclcpp::Node
{
public:
  VehicleVelocityConverter();
  ~VehicleVelocityConverter() = default;

private:
  void callbackVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg);

  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr
    vehicle_report_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_with_covariance_pub_;

  std::string frame_id_;
  std::array<double, 36> twist_covariance_;
};

#endif
