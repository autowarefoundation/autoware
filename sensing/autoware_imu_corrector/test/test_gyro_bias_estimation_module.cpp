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

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

namespace autoware::imu_corrector
{
class GyroBiasEstimationModuleTest : public ::testing::Test
{
protected:
  GyroBiasEstimationModule module;
};

TEST_F(GyroBiasEstimationModuleTest, GetBiasEstimationWhenVehicleStopped)
{
  std::vector<geometry_msgs::msg::PoseStamped> pose_list;
  std::vector<geometry_msgs::msg::Vector3Stamped> gyro_list;
  geometry_msgs::msg::PoseStamped pose1;
  pose1.header.stamp = rclcpp::Time(0);
  pose1.pose.orientation.w = 1.0;
  pose_list.push_back(pose1);
  geometry_msgs::msg::PoseStamped pose2;
  pose2.header.stamp = rclcpp::Time(1e9);
  pose2.pose.orientation.w = 1.0;
  pose_list.push_back(pose2);

  geometry_msgs::msg::Vector3Stamped gyro1;
  gyro1.header.stamp = rclcpp::Time(0.25 * 1e9);
  gyro1.vector.x = 0.1;
  gyro1.vector.y = 0.2;
  gyro1.vector.z = 0.3;
  gyro_list.push_back(gyro1);
  geometry_msgs::msg::Vector3Stamped gyro2;
  gyro2.header.stamp = rclcpp::Time(0.5 * 1e9);
  gyro2.vector.x = 0.1;
  gyro2.vector.y = 0.2;
  gyro2.vector.z = 0.3;
  gyro_list.push_back(gyro2);

  for (size_t i = 0; i < 10; ++i) {
    module.update_bias(pose_list, gyro_list);
  }
  const geometry_msgs::msg::Vector3 result = module.get_bias_base_link();
  ASSERT_DOUBLE_EQ(result.x, 0.1);
  ASSERT_DOUBLE_EQ(result.y, 0.2);
  ASSERT_DOUBLE_EQ(result.z, 0.3);
}

TEST_F(GyroBiasEstimationModuleTest, GetInsufficientDataException)
{
  // for the case of method with [[nodiscard]] attribute
  geometry_msgs::msg::Vector3 bias_base_link;
  ASSERT_THROW(bias_base_link = module.get_bias_base_link(), std::runtime_error);
}

TEST_F(GyroBiasEstimationModuleTest, GetInsufficientDataExceptionWhenVehicleMoving)
{
  std::vector<geometry_msgs::msg::PoseStamped> pose_list;
  std::vector<geometry_msgs::msg::Vector3Stamped> gyro_list;
  geometry_msgs::msg::PoseStamped pose1;
  pose1.header.stamp = rclcpp::Time(0);
  pose1.pose.orientation.w = 1.0;
  pose_list.push_back(pose1);

  geometry_msgs::msg::Vector3Stamped gyro1;
  gyro1.header.stamp = rclcpp::Time(0);
  gyro1.vector.x = 0.1;
  gyro1.vector.y = 0.2;
  gyro1.vector.z = 0.3;
  gyro_list.push_back(gyro1);

  for (size_t i = 0; i < 10; ++i) {
    ASSERT_THROW(module.update_bias(pose_list, gyro_list), std::runtime_error);
  }
}
}  // namespace autoware::imu_corrector
