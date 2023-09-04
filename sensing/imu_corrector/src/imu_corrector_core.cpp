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

#include "imu_corrector_core.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <algorithm>

std::array<double, 9> transformCovariance(const std::array<double, 9> & cov)
{
  using COV_IDX = tier4_autoware_utils::xyz_covariance_index::XYZ_COV_IDX;

  double max_cov = 0.0;
  max_cov = std::max(max_cov, cov[COV_IDX::X_X]);
  max_cov = std::max(max_cov, cov[COV_IDX::Y_Y]);
  max_cov = std::max(max_cov, cov[COV_IDX::Z_Z]);

  std::array<double, 9> cov_transformed;
  cov_transformed.fill(0.);
  cov_transformed[COV_IDX::X_X] = max_cov;
  cov_transformed[COV_IDX::Y_Y] = max_cov;
  cov_transformed[COV_IDX::Z_Z] = max_cov;
  return cov_transformed;
}

geometry_msgs::msg::Vector3 transformVector3(
  const geometry_msgs::msg::Vector3 & vec, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::Vector3Stamped vec_stamped;
  vec_stamped.vector = vec;

  geometry_msgs::msg::Vector3Stamped vec_stamped_transformed;
  tf2::doTransform(vec_stamped, vec_stamped_transformed, transform);
  return vec_stamped_transformed.vector;
}

namespace imu_corrector
{
ImuCorrector::ImuCorrector()
: Node("imu_corrector"), output_frame_(declare_parameter<std::string>("base_link", "base_link"))
{
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  angular_velocity_offset_x_imu_link_ = declare_parameter<double>("angular_velocity_offset_x", 0.0);
  angular_velocity_offset_y_imu_link_ = declare_parameter<double>("angular_velocity_offset_y", 0.0);
  angular_velocity_offset_z_imu_link_ = declare_parameter<double>("angular_velocity_offset_z", 0.0);

  angular_velocity_stddev_xx_imu_link_ =
    declare_parameter<double>("angular_velocity_stddev_xx", 0.03);
  angular_velocity_stddev_yy_imu_link_ =
    declare_parameter<double>("angular_velocity_stddev_yy", 0.03);
  angular_velocity_stddev_zz_imu_link_ =
    declare_parameter<double>("angular_velocity_stddev_zz", 0.03);

  accel_stddev_imu_link_ = declare_parameter<double>("acceleration_stddev", 10000.0);

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "input", rclcpp::QoS{1}, std::bind(&ImuCorrector::callbackImu, this, std::placeholders::_1));

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("output", rclcpp::QoS{10});
}

void ImuCorrector::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg = *imu_msg_ptr;

  imu_msg.angular_velocity.x -= angular_velocity_offset_x_imu_link_;
  imu_msg.angular_velocity.y -= angular_velocity_offset_y_imu_link_;
  imu_msg.angular_velocity.z -= angular_velocity_offset_z_imu_link_;

  imu_msg.angular_velocity_covariance[COV_IDX::X_X] =
    angular_velocity_stddev_xx_imu_link_ * angular_velocity_stddev_xx_imu_link_;
  imu_msg.angular_velocity_covariance[COV_IDX::Y_Y] =
    angular_velocity_stddev_yy_imu_link_ * angular_velocity_stddev_yy_imu_link_;
  imu_msg.angular_velocity_covariance[COV_IDX::Z_Z] =
    angular_velocity_stddev_zz_imu_link_ * angular_velocity_stddev_zz_imu_link_;
  imu_msg.linear_acceleration_covariance[COV_IDX::X_X] =
    accel_stddev_imu_link_ * accel_stddev_imu_link_;
  imu_msg.linear_acceleration_covariance[COV_IDX::Y_Y] =
    accel_stddev_imu_link_ * accel_stddev_imu_link_;
  imu_msg.linear_acceleration_covariance[COV_IDX::Z_Z] =
    accel_stddev_imu_link_ * accel_stddev_imu_link_;

  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener_->getLatestTransform(imu_msg.header.frame_id, output_frame_);
  if (!tf_imu2base_ptr) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", output_frame_.c_str(),
      (imu_msg.header.frame_id).c_str());
    return;
  }

  sensor_msgs::msg::Imu imu_msg_base_link;
  imu_msg_base_link.header.stamp = imu_msg_ptr->header.stamp;
  imu_msg_base_link.header.frame_id = output_frame_;
  imu_msg_base_link.linear_acceleration =
    transformVector3(imu_msg.linear_acceleration, *tf_imu2base_ptr);
  imu_msg_base_link.linear_acceleration_covariance =
    transformCovariance(imu_msg.linear_acceleration_covariance);
  imu_msg_base_link.angular_velocity = transformVector3(imu_msg.angular_velocity, *tf_imu2base_ptr);
  imu_msg_base_link.angular_velocity_covariance =
    transformCovariance(imu_msg.angular_velocity_covariance);

  imu_pub_->publish(imu_msg_base_link);
}

}  // namespace imu_corrector
