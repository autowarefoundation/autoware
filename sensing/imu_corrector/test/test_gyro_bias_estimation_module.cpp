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

#include "../src/gyro_bias_estimation_module.hpp"

#include <gtest/gtest.h>

namespace imu_corrector
{
class GyroBiasEstimationModuleTest : public ::testing::Test
{
protected:
  double velocity_threshold = 1.0;
  double timestamp_threshold = 5.0;
  size_t data_num_threshold = 10;
  GyroBiasEstimationModule module =
    GyroBiasEstimationModule(velocity_threshold, timestamp_threshold, data_num_threshold);
};

TEST_F(GyroBiasEstimationModuleTest, GetBiasEstimationWhenVehicleStopped)
{
  geometry_msgs::msg::Vector3 gyro;
  gyro.x = 0.1;
  gyro.y = 0.2;
  gyro.z = 0.3;
  for (size_t i = 0; i < data_num_threshold + 1; ++i) {
    module.update_velocity(
      i * 0.1 * timestamp_threshold, 0.0);  // velocity = 0.0 < 1.0 = velocity_threshold
    module.update_gyro(i * 0.1 * timestamp_threshold, gyro);
  }
  ASSERT_NEAR(module.get_bias().x, gyro.x, 0.0001);
  ASSERT_NEAR(module.get_bias().y, gyro.y, 0.0001);
  ASSERT_NEAR(module.get_bias().z, gyro.z, 0.0001);
}

TEST_F(GyroBiasEstimationModuleTest, GetInsufficientDataException)
{
  ASSERT_THROW(module.get_bias(), std::runtime_error);
}

TEST_F(GyroBiasEstimationModuleTest, GetInsufficientDataExceptionWhenVehicleMoving)
{
  geometry_msgs::msg::Vector3 gyro;
  gyro.x = 0.1;
  gyro.y = 0.2;
  gyro.z = 0.3;
  for (size_t i = 0; i < data_num_threshold + 1; ++i) {
    module.update_velocity(
      i * 0.1 * timestamp_threshold, 5.0);  // velocity = 5.0 > 1.0 = velocity_threshold
    module.update_gyro(i * 0.1 * timestamp_threshold, gyro);
  }
  ASSERT_THROW(module.get_bias(), std::runtime_error);
}
}  // namespace imu_corrector
