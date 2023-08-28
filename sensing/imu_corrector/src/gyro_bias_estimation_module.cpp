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

#include "gyro_bias_estimation_module.hpp"

namespace imu_corrector
{
GyroBiasEstimationModule::GyroBiasEstimationModule(
  const double velocity_threshold, const double timestamp_threshold,
  const size_t data_num_threshold)
: velocity_threshold_(velocity_threshold),
  timestamp_threshold_(timestamp_threshold),
  data_num_threshold_(data_num_threshold),
  is_stopped_(false)
{
}

void GyroBiasEstimationModule::update_gyro(
  const double time, const geometry_msgs::msg::Vector3 & gyro)
{
  if (time - last_velocity_time_ > timestamp_threshold_) {
    return;
  }
  if (!is_stopped_) {
    return;
  }
  gyro_buffer_.push_back(gyro);
  if (gyro_buffer_.size() > data_num_threshold_) {
    gyro_buffer_.pop_front();
  }
}

void GyroBiasEstimationModule::update_velocity(const double time, const double velocity)
{
  is_stopped_ = velocity <= velocity_threshold_;
  last_velocity_time_ = time;
}

geometry_msgs::msg::Vector3 GyroBiasEstimationModule::get_bias() const
{
  if (gyro_buffer_.size() < data_num_threshold_) {
    throw std::runtime_error("Bias estimation is not yet ready because of insufficient data.");
  }

  geometry_msgs::msg::Vector3 bias;
  bias.x = 0.0;
  bias.y = 0.0;
  bias.z = 0.0;
  for (const auto & gyro : gyro_buffer_) {
    bias.x += gyro.x;
    bias.y += gyro.y;
    bias.z += gyro.z;
  }
  bias.x /= gyro_buffer_.size();
  bias.y /= gyro_buffer_.size();
  bias.z /= gyro_buffer_.size();
  return bias;
}

}  // namespace imu_corrector
