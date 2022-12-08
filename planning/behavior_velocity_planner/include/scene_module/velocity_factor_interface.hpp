
// Copyright 2022 TIER IV, Inc.
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

#ifndef SCENE_MODULE__VELOCITY_FACTOR_INTERFACE_HPP_
#define SCENE_MODULE__VELOCITY_FACTOR_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/velocity_factor.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_factor_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{

using autoware_adapi_v1_msgs::msg::VelocityFactor;
using autoware_adapi_v1_msgs::msg::VelocityFactorArray;
using geometry_msgs::msg::Pose;
using VelocityFactorType = VelocityFactor::_type_type;
using VelocityFactorStatus = VelocityFactor::_status_type;

class VelocityFactorInterface
{
public:
  VelocityFactorInterface() { type_ = VelocityFactor::UNKNOWN; }

  VelocityFactor get() const { return velocity_factor_; }
  void init(const VelocityFactorType type) { type_ = type; }
  void reset() { velocity_factor_.type = VelocityFactor::UNKNOWN; }

  template <class T>
  void set(
    const T & points, const Pose & curr_pose, const Pose & stop_pose,
    const VelocityFactorStatus status, const std::string detail = "")
  {
    const auto & curr_point = curr_pose.position;
    const auto & stop_point = stop_pose.position;
    velocity_factor_.type = type_;
    velocity_factor_.pose = stop_pose;
    velocity_factor_.distance = motion_utils::calcSignedArcLength(points, curr_point, stop_point);
    velocity_factor_.status = status;
    velocity_factor_.detail = detail;
  }

private:
  VelocityFactorType type_;
  VelocityFactor velocity_factor_;
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__VELOCITY_FACTOR_INTERFACE_HPP_
