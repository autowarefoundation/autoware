// Copyright 2018-2023 The Autoware Foundation
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

#include "autoware/mpc_lateral_controller/steering_offset/steering_offset.hpp"

#include <algorithm>
#include <iostream>
#include <numeric>

SteeringOffsetEstimator::SteeringOffsetEstimator(
  double wheelbase, double average_num, double vel_thres, double steer_thres, double offset_limit)
: wheelbase_(wheelbase),
  average_num_(average_num),
  update_vel_threshold_(vel_thres),
  update_steer_threshold_(steer_thres),
  offset_limit_(offset_limit),
  steering_offset_storage_(average_num, 0.0)
{
}

void SteeringOffsetEstimator::updateOffset(
  const geometry_msgs::msg::Twist & twist, const double steering)
{
  const bool update_offset =
    (std::abs(twist.linear.x) > update_vel_threshold_ &&
     std::abs(steering) < update_steer_threshold_);

  if (!update_offset) return;

  const auto steer_angvel = std::atan2(twist.angular.z * wheelbase_, twist.linear.x);
  const auto steer_offset = steering - steer_angvel;
  steering_offset_storage_.push_back(steer_offset);
  if (steering_offset_storage_.size() > average_num_) {
    steering_offset_storage_.pop_front();
  }

  steering_offset_ =
    std::accumulate(std::begin(steering_offset_storage_), std::end(steering_offset_storage_), 0.0) /
    std::size(steering_offset_storage_);
}

double SteeringOffsetEstimator::getOffset() const
{
  return std::clamp(steering_offset_, -offset_limit_, offset_limit_);
}
