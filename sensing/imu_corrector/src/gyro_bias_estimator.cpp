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

#include "gyro_bias_estimator.hpp"

namespace imu_corrector
{
GyroBiasEstimator::GyroBiasEstimator()
: Node("gyro_bias_validator"),
  gyro_bias_threshold_(declare_parameter<double>("gyro_bias_threshold")),
  angular_velocity_offset_x_(declare_parameter<double>("angular_velocity_offset_x")),
  angular_velocity_offset_y_(declare_parameter<double>("angular_velocity_offset_y")),
  angular_velocity_offset_z_(declare_parameter<double>("angular_velocity_offset_z")),
  updater_(this),
  gyro_bias_(std::nullopt)
{
  updater_.setHardwareID(get_name());
  updater_.add("gyro_bias_validator", this, &GyroBiasEstimator::update_diagnostics);

  const double velocity_threshold = declare_parameter<double>("velocity_threshold");
  const double timestamp_threshold = declare_parameter<double>("timestamp_threshold");
  const size_t data_num_threshold =
    static_cast<size_t>(declare_parameter<int>("data_num_threshold"));
  gyro_bias_estimation_module_ = std::make_unique<GyroBiasEstimationModule>(
    velocity_threshold, timestamp_threshold, data_num_threshold);

  imu_sub_ = create_subscription<Imu>(
    "~/input/imu_raw", rclcpp::SensorDataQoS(),
    [this](const Imu::ConstSharedPtr msg) { callback_imu(msg); });
  twist_sub_ = create_subscription<TwistWithCovarianceStamped>(
    "~/input/twist", rclcpp::SensorDataQoS(),
    [this](const TwistWithCovarianceStamped::ConstSharedPtr msg) { callback_twist(msg); });

  gyro_bias_pub_ = create_publisher<Vector3Stamped>("~/output/gyro_bias", rclcpp::SensorDataQoS());
}

void GyroBiasEstimator::callback_imu(const Imu::ConstSharedPtr imu_msg_ptr)
{
  // Update gyro data
  gyro_bias_estimation_module_->update_gyro(
    rclcpp::Time(imu_msg_ptr->header.stamp).seconds(), imu_msg_ptr->angular_velocity);

  // Estimate gyro bias
  try {
    gyro_bias_ = gyro_bias_estimation_module_->get_bias();
  } catch (const std::runtime_error & e) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 1000, e.what());
  }

  // Publish results for debugging
  if (gyro_bias_ != std::nullopt) {
    Vector3Stamped gyro_bias_msg;
    gyro_bias_msg.header.stamp = this->now();
    gyro_bias_msg.vector = gyro_bias_.value();
    gyro_bias_pub_->publish(gyro_bias_msg);
  }

  // Update diagnostics
  updater_.force_update();
}

void GyroBiasEstimator::callback_twist(
  const TwistWithCovarianceStamped::ConstSharedPtr twist_msg_ptr)
{
  gyro_bias_estimation_module_->update_velocity(
    rclcpp::Time(twist_msg_ptr->header.stamp).seconds(), twist_msg_ptr->twist.twist.linear.x);
}

void GyroBiasEstimator::update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (gyro_bias_ == std::nullopt) {
    stat.add("gyro_bias", "Bias estimation is not yet ready because of insufficient data.");
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Not initialized");
  } else {
    stat.add("gyro_bias_x_for_imu_corrector", gyro_bias_.value().x);
    stat.add("gyro_bias_y_for_imu_corrector", gyro_bias_.value().y);
    stat.add("gyro_bias_z_for_imu_corrector", gyro_bias_.value().z);

    stat.add("estimated_gyro_bias_x", gyro_bias_.value().x - angular_velocity_offset_x_);
    stat.add("estimated_gyro_bias_y", gyro_bias_.value().y - angular_velocity_offset_y_);
    stat.add("estimated_gyro_bias_z", gyro_bias_.value().z - angular_velocity_offset_z_);

    // Validation
    const bool is_bias_small_enough =
      std::abs(gyro_bias_.value().x - angular_velocity_offset_x_) < gyro_bias_threshold_ &&
      std::abs(gyro_bias_.value().y - angular_velocity_offset_y_) < gyro_bias_threshold_ &&
      std::abs(gyro_bias_.value().z - angular_velocity_offset_z_) < gyro_bias_threshold_;

    // Update diagnostics
    if (is_bias_small_enough) {
      stat.add("gyro_bias", "OK");
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
    } else {
      stat.add(
        "gyro_bias",
        "Gyro bias may be incorrect. Please calibrate IMU and reflect the result in "
        "imu_corrector. You may also use the output of gyro_bias_estimator.");
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "WARN");
    }
  }
}

}  // namespace imu_corrector
