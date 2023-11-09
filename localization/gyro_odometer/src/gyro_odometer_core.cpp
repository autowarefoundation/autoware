// Copyright 2015-2019 Autoware Foundation
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

#include "gyro_odometer/gyro_odometer_core.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <fmt/core.h>

#include <cmath>
#include <memory>
#include <string>

std::array<double, 9> transformCovariance(const std::array<double, 9> & cov)
{
  using COV_IDX = tier4_autoware_utils::xyz_covariance_index::XYZ_COV_IDX;

  double max_cov = std::max({cov[COV_IDX::X_X], cov[COV_IDX::Y_Y], cov[COV_IDX::Z_Z]});

  std::array<double, 9> cov_transformed;
  cov_transformed.fill(0.);
  cov_transformed[COV_IDX::X_X] = max_cov;
  cov_transformed[COV_IDX::Y_Y] = max_cov;
  cov_transformed[COV_IDX::Z_Z] = max_cov;
  return cov_transformed;
}

geometry_msgs::msg::TwistWithCovarianceStamped concatGyroAndOdometer(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & vehicle_twist_queue,
  const std::deque<sensor_msgs::msg::Imu> & gyro_queue)
{
  using COV_IDX_XYZ = tier4_autoware_utils::xyz_covariance_index::XYZ_COV_IDX;
  using COV_IDX_XYZRPY = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

  double vx_mean = 0;
  geometry_msgs::msg::Vector3 gyro_mean{};
  double vx_covariance_original = 0;
  geometry_msgs::msg::Vector3 gyro_covariance_original{};
  for (const auto & vehicle_twist : vehicle_twist_queue) {
    vx_mean += vehicle_twist.twist.twist.linear.x;
    vx_covariance_original += vehicle_twist.twist.covariance[0 * 6 + 0];
  }
  vx_mean /= vehicle_twist_queue.size();
  vx_covariance_original /= vehicle_twist_queue.size();

  for (const auto & gyro : gyro_queue) {
    gyro_mean.x += gyro.angular_velocity.x;
    gyro_mean.y += gyro.angular_velocity.y;
    gyro_mean.z += gyro.angular_velocity.z;
    gyro_covariance_original.x += gyro.angular_velocity_covariance[COV_IDX_XYZ::X_X];
    gyro_covariance_original.y += gyro.angular_velocity_covariance[COV_IDX_XYZ::Y_Y];
    gyro_covariance_original.z += gyro.angular_velocity_covariance[COV_IDX_XYZ::Z_Z];
  }
  gyro_mean.x /= gyro_queue.size();
  gyro_mean.y /= gyro_queue.size();
  gyro_mean.z /= gyro_queue.size();
  gyro_covariance_original.x /= gyro_queue.size();
  gyro_covariance_original.y /= gyro_queue.size();
  gyro_covariance_original.z /= gyro_queue.size();

  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov;
  const auto latest_vehicle_twist_stamp = rclcpp::Time(vehicle_twist_queue.back().header.stamp);
  const auto latest_imu_stamp = rclcpp::Time(gyro_queue.back().header.stamp);
  if (latest_vehicle_twist_stamp < latest_imu_stamp) {
    twist_with_cov.header.stamp = latest_imu_stamp;
  } else {
    twist_with_cov.header.stamp = latest_vehicle_twist_stamp;
  }
  twist_with_cov.header.frame_id = gyro_queue.front().header.frame_id;
  twist_with_cov.twist.twist.linear.x = vx_mean;
  twist_with_cov.twist.twist.angular = gyro_mean;

  // From a statistical point of view, here we reduce the covariances according to the number of
  // observed data
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::X_X] =
    vx_covariance_original / vehicle_twist_queue.size();
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Y_Y] = 100000.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Z_Z] = 100000.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::ROLL_ROLL] =
    gyro_covariance_original.x / gyro_queue.size();
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::PITCH_PITCH] =
    gyro_covariance_original.y / gyro_queue.size();
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::YAW_YAW] =
    gyro_covariance_original.z / gyro_queue.size();

  return twist_with_cov;
}

GyroOdometer::GyroOdometer(const rclcpp::NodeOptions & options)
: Node("gyro_odometer", options),
  output_frame_(declare_parameter<std::string>("output_frame")),
  message_timeout_sec_(declare_parameter<double>("message_timeout_sec")),
  vehicle_twist_arrived_(false),
  imu_arrived_(false)
{
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);
  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);

  vehicle_twist_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "vehicle/twist_with_covariance", rclcpp::QoS{100},
    std::bind(&GyroOdometer::callbackVehicleTwist, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::QoS{100}, std::bind(&GyroOdometer::callbackImu, this, std::placeholders::_1));

  twist_raw_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist_raw", rclcpp::QoS{10});
  twist_with_covariance_raw_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance_raw", rclcpp::QoS{10});

  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist", rclcpp::QoS{10});
  twist_with_covariance_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", rclcpp::QoS{10});

  // TODO(YamatoAndo) createTimer
}

GyroOdometer::~GyroOdometer()
{
}

void GyroOdometer::callbackVehicleTwist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr vehicle_twist_ptr)
{
  vehicle_twist_arrived_ = true;
  if (!imu_arrived_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Imu msg is not subscribed");
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  const double twist_dt = std::abs((this->now() - vehicle_twist_ptr->header.stamp).seconds());
  if (twist_dt > message_timeout_sec_) {
    const std::string error_msg = fmt::format(
      "Twist msg is timeout. twist_dt: {}[sec], tolerance {}[sec]", twist_dt, message_timeout_sec_);
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, error_msg.c_str());
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  vehicle_twist_queue_.push_back(*vehicle_twist_ptr);

  if (gyro_queue_.empty()) return;
  const double imu_dt = std::abs((this->now() - gyro_queue_.back().header.stamp).seconds());
  if (imu_dt > message_timeout_sec_) {
    const std::string error_msg = fmt::format(
      "Imu msg is timeout. twist_dt: {}[sec], tolerance {}[sec]", imu_dt, message_timeout_sec_);
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, error_msg.c_str());
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  const geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov_raw =
    concatGyroAndOdometer(vehicle_twist_queue_, gyro_queue_);
  publishData(twist_with_cov_raw);
  vehicle_twist_queue_.clear();
  gyro_queue_.clear();
}

void GyroOdometer::callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  imu_arrived_ = true;
  if (!vehicle_twist_arrived_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Twist msg is not subscribed");
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  const double imu_dt = std::abs((this->now() - imu_msg_ptr->header.stamp).seconds());
  if (imu_dt > message_timeout_sec_) {
    const std::string error_msg = fmt::format(
      "Imu msg is timeout. imu_dt: {}[sec], tolerance {}[sec]", imu_dt, message_timeout_sec_);
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, error_msg.c_str());
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener_->getLatestTransform(imu_msg_ptr->header.frame_id, output_frame_);
  if (!tf_imu2base_ptr) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", output_frame_.c_str(),
      (imu_msg_ptr->header.frame_id).c_str());
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  geometry_msgs::msg::Vector3Stamped angular_velocity;
  angular_velocity.header = imu_msg_ptr->header;
  angular_velocity.vector = imu_msg_ptr->angular_velocity;

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  transformed_angular_velocity.header = tf_imu2base_ptr->header;
  tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_imu2base_ptr);

  sensor_msgs::msg::Imu gyro_base_link;
  gyro_base_link.header = imu_msg_ptr->header;
  gyro_base_link.header.frame_id = output_frame_;
  gyro_base_link.angular_velocity = transformed_angular_velocity.vector;
  gyro_base_link.angular_velocity_covariance =
    transformCovariance(imu_msg_ptr->angular_velocity_covariance);

  gyro_queue_.push_back(gyro_base_link);

  if (vehicle_twist_queue_.empty()) return;
  const double twist_dt =
    std::abs((this->now() - vehicle_twist_queue_.back().header.stamp).seconds());
  if (twist_dt > message_timeout_sec_) {
    const std::string error_msg = fmt::format(
      "Twist msg is timeout. twist_dt: {}[sec], tolerance {}[sec]", twist_dt, message_timeout_sec_);
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, error_msg.c_str());
    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  const geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov_raw =
    concatGyroAndOdometer(vehicle_twist_queue_, gyro_queue_);
  publishData(twist_with_cov_raw);
  vehicle_twist_queue_.clear();
  gyro_queue_.clear();
}

void GyroOdometer::publishData(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_raw)
{
  geometry_msgs::msg::TwistStamped twist_raw;
  twist_raw.header = twist_with_cov_raw.header;
  twist_raw.twist = twist_with_cov_raw.twist.twist;

  twist_raw_pub_->publish(twist_raw);
  twist_with_covariance_raw_pub_->publish(twist_with_cov_raw);

  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance = twist_with_cov_raw;
  geometry_msgs::msg::TwistStamped twist = twist_raw;

  // clear imu yaw bias if vehicle is stopped
  if (
    std::fabs(twist_with_cov_raw.twist.twist.angular.z) < 0.01 &&
    std::fabs(twist_with_cov_raw.twist.twist.linear.x) < 0.01) {
    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;
    twist_with_covariance.twist.twist.angular.x = 0.0;
    twist_with_covariance.twist.twist.angular.y = 0.0;
    twist_with_covariance.twist.twist.angular.z = 0.0;
  }

  twist_pub_->publish(twist);
  twist_with_covariance_pub_->publish(twist_with_covariance);
}
