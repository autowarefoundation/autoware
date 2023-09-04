// Copyright 2020 Tier IV, Inc.
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
#ifndef IMU_CORRECTOR_CORE_HPP_
#define IMU_CORRECTOR_CORE_HPP_

#include "tier4_autoware_utils/ros/msg_covariance.hpp"
#include "tier4_autoware_utils/ros/transform_listener.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace imu_corrector
{
class ImuCorrector : public rclcpp::Node
{
  using COV_IDX = tier4_autoware_utils::xyz_covariance_index::XYZ_COV_IDX;

public:
  ImuCorrector();

private:
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  double angular_velocity_offset_x_imu_link_;
  double angular_velocity_offset_y_imu_link_;
  double angular_velocity_offset_z_imu_link_;

  double angular_velocity_stddev_xx_imu_link_;
  double angular_velocity_stddev_yy_imu_link_;
  double angular_velocity_stddev_zz_imu_link_;

  double accel_stddev_imu_link_;

  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;

  std::string output_frame_;
};
}  // namespace imu_corrector

#endif  // IMU_CORRECTOR_CORE_HPP_
