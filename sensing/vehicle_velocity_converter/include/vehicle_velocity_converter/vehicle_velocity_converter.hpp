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

#ifndef VEHICLE_VELOCITY_CONVERTER__VEHICLE_VELOCITY_CONVERTER_HPP_
#define VEHICLE_VELOCITY_CONVERTER__VEHICLE_VELOCITY_CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <array>
#include <string>
#include <vector>

class VehicleVelocityConverter : public rclcpp::Node
{
public:
  explicit VehicleVelocityConverter(const rclcpp::NodeOptions & options);

private:
  void callback_velocity_report(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg);

  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr vehicle_report_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_with_covariance_pub_;

  std::string frame_id_;
  double stddev_vx_;
  double stddev_wz_;
  double speed_scale_factor_;
};

#endif  // VEHICLE_VELOCITY_CONVERTER__VEHICLE_VELOCITY_CONVERTER_HPP_
