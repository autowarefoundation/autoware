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

#include "behavior_path_planner_common/interface/steering_factor_interface.hpp"

namespace steering_factor_interface
{
SteeringFactorInterface::SteeringFactorInterface(rclcpp::Node * node, const std::string & name)
: logger_{node->get_logger().get_child("PlanningAPI[" + name + "]")}
{
  // Publisher
  pub_steering_factors_ =
    node->create_publisher<SteeringFactorArray>("/planning/steering_factor/" + name, 1);
}

void SteeringFactorInterface::publishSteeringFactor(const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  registered_steering_factors_.header.stamp = stamp;
  pub_steering_factors_->publish(registered_steering_factors_);
}

void SteeringFactorInterface::updateSteeringFactor(
  const std::array<Pose, 2> & pose, const std::array<double, 2> distance,
  const std::string & behavior, const uint16_t direction, const uint16_t status,
  const std::string & detail)
{
  std::lock_guard<std::mutex> lock(mutex_);
  SteeringFactor factor;
  factor.pose = pose;
  std::array<float, 2> converted_distance{0.0, 0.0};
  for (int i = 0; i < 2; ++i) converted_distance[i] = static_cast<float>(distance[i]);
  factor.distance = converted_distance;
  factor.behavior = behavior;
  factor.direction = direction;
  factor.status = status;
  factor.detail = detail;
  registered_steering_factors_.factors = {factor};
}

void SteeringFactorInterface::clearSteeringFactors()
{
  std::lock_guard<std::mutex> lock(mutex_);
  registered_steering_factors_.factors.clear();
}

rclcpp::Logger SteeringFactorInterface::getLogger() const
{
  return logger_;
}

}  // namespace steering_factor_interface
