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

#include "imu_corrector/imu_corrector_core.hpp"

namespace imu_corrector
{
ImuCorrector::ImuCorrector(const rclcpp::NodeOptions & node_options)
: Node("imu_corrector", node_options)
{
  angular_velocity_offset_z_ = declare_parameter<double>("angular_velocity_offset_z", 0.0);

  angular_velocity_stddev_zz_ = declare_parameter<double>("angular_velocity_stddev_zz", 0.03);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "input", rclcpp::QoS{1}, std::bind(&ImuCorrector::callbackImu, this, std::placeholders::_1));

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("output", rclcpp::QoS{10});
}

void ImuCorrector::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg = *imu_msg_ptr;

  imu_msg.angular_velocity.z += angular_velocity_offset_z_;

  imu_msg.angular_velocity_covariance[8] =
    angular_velocity_stddev_zz_ * angular_velocity_stddev_zz_;

  imu_pub_->publish(imu_msg);
}

}  // namespace imu_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(imu_corrector::ImuCorrector)
