
// Copyright 2022-2024 TIER IV, Inc.
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

#ifndef AUTOWARE__MOTION_UTILS__FACTOR__VELOCITY_FACTOR_INTERFACE_HPP_
#define AUTOWARE__MOTION_UTILS__FACTOR__VELOCITY_FACTOR_INTERFACE_HPP_

#include <autoware_adapi_v1_msgs/msg/planning_behavior.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_factor.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_factor_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <string>
#include <vector>

namespace autoware::motion_utils
{
using autoware_adapi_v1_msgs::msg::PlanningBehavior;
using autoware_adapi_v1_msgs::msg::VelocityFactor;
using VelocityFactorBehavior = VelocityFactor::_behavior_type;
using VelocityFactorStatus = VelocityFactor::_status_type;
using geometry_msgs::msg::Pose;

class VelocityFactorInterface
{
public:
  [[nodiscard]] VelocityFactor get() const { return velocity_factor_; }
  void init(const VelocityFactorBehavior & behavior) { behavior_ = behavior; }
  void reset() { velocity_factor_.behavior = PlanningBehavior::UNKNOWN; }

  template <class PointType>
  void set(
    const std::vector<PointType> & points, const Pose & curr_pose, const Pose & stop_pose,
    const VelocityFactorStatus status, const std::string & detail = "");

private:
  VelocityFactorBehavior behavior_{VelocityFactor::UNKNOWN};
  VelocityFactor velocity_factor_{};
};

}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__FACTOR__VELOCITY_FACTOR_INTERFACE_HPP_
