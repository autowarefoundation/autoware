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

#include <geometry_msgs/msg/vector3.hpp>

#include <deque>

namespace imu_corrector
{
class GyroBiasEstimationModule
{
public:
  GyroBiasEstimationModule(
    const double velocity_threshold, const double timestamp_threshold,
    const size_t data_num_threshold);
  geometry_msgs::msg::Vector3 get_bias() const;
  void update_gyro(const double time, const geometry_msgs::msg::Vector3 & gyro);
  void update_velocity(const double time, const double velocity);

private:
  const double velocity_threshold_;
  const double timestamp_threshold_;
  const size_t data_num_threshold_;
  bool is_stopped_;
  std::deque<geometry_msgs::msg::Vector3> gyro_buffer_;

  double last_velocity_time_;
};
}  // namespace imu_corrector

#endif  // GYRO_BIAS_ESTIMATION_MODULE_HPP_
