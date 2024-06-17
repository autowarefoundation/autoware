// Copyright 2022 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__STEERING_FACTOR_INTERFACE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__STEERING_FACTOR_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/steering_factor_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <mutex>
#include <string>

namespace steering_factor_interface
{
using autoware_adapi_v1_msgs::msg::SteeringFactor;
using autoware_adapi_v1_msgs::msg::SteeringFactorArray;
using geometry_msgs::msg::Pose;

class SteeringFactorInterface
{
public:
  SteeringFactorInterface(rclcpp::Node * node, const std::string & name);
  void publishSteeringFactor(const rclcpp::Time & stamp);
  void updateSteeringFactor(
    const std::array<Pose, 2> & poses, const std::array<double, 2> distances,
    const std::string & behavior, const uint16_t direction, const uint16_t status,
    const std::string & detail);
  void clearSteeringFactors();

private:
  rclcpp::Logger getLogger() const;

  rclcpp::Publisher<SteeringFactorArray>::SharedPtr pub_steering_factors_;

  std::mutex mutex_;
  rclcpp::Logger logger_;
  SteeringFactorArray registered_steering_factors_;
};

}  // namespace steering_factor_interface

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__STEERING_FACTOR_INTERFACE_HPP_
