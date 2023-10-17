// Copyright 2023 TIER IV, Inc.
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

#ifndef GYRO_BIAS_ESTIMATION_MODULE_HPP_
#define GYRO_BIAS_ESTIMATION_MODULE_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <deque>
#include <utility>
#include <vector>

namespace imu_corrector
{
class GyroBiasEstimationModule
{
public:
  GyroBiasEstimationModule() = default;
  void update_bias(
    const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
    const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list);
  geometry_msgs::msg::Vector3 get_bias_base_link() const;

private:
  std::pair<geometry_msgs::msg::Vector3, geometry_msgs::msg::Vector3> gyro_bias_pair_;
};
}  // namespace imu_corrector

#endif  // GYRO_BIAS_ESTIMATION_MODULE_HPP_
