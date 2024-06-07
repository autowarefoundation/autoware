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

#include "test_gyro_odometer_helper.hpp"

using geometry_msgs::msg::TwistWithCovarianceStamped;
using sensor_msgs::msg::Imu;

Imu generate_sample_imu()
{
  Imu imu;
  imu.header.frame_id = "base_link";
  imu.angular_velocity.x = 0.1;
  imu.angular_velocity.y = 0.2;
  imu.angular_velocity.z = 0.3;
  return imu;
}

TwistWithCovarianceStamped generate_sample_velocity()
{
  TwistWithCovarianceStamped twist;
  twist.header.frame_id = "base_link";
  twist.twist.twist.linear.x = 1.0;
  return twist;
}

rclcpp::NodeOptions get_node_options_with_default_params()
{
  rclcpp::NodeOptions node_options;

  // for gyro_odometer
  node_options.append_parameter_override("output_frame", "base_link");
  node_options.append_parameter_override("message_timeout_sec", 1e12);
  return node_options;
}
