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

#include <rclcpp/rclcpp.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <fmt/core.h>

#include <cmath>
#include <memory>
#include <sstream>
#include <string>

namespace autoware::gyro_odometer
{

std::array<double, 9> transform_covariance(const std::array<double, 9> & cov)
{
  using COV_IDX = autoware::universe_utils::xyz_covariance_index::XYZ_COV_IDX;

  double max_cov = std::max({cov[COV_IDX::X_X], cov[COV_IDX::Y_Y], cov[COV_IDX::Z_Z]});

  std::array<double, 9> cov_transformed = {};
  cov_transformed.fill(0.);
  cov_transformed[COV_IDX::X_X] = max_cov;
  cov_transformed[COV_IDX::Y_Y] = max_cov;
  cov_transformed[COV_IDX::Z_Z] = max_cov;
  return cov_transformed;
}

GyroOdometerNode::GyroOdometerNode(const rclcpp::NodeOptions & node_options)
: Node("gyro_odometer", node_options),
  output_frame_(declare_parameter<std::string>("output_frame")),
  message_timeout_sec_(declare_parameter<double>("message_timeout_sec")),
  vehicle_twist_arrived_(false),
  imu_arrived_(false)
{
  transform_listener_ = std::make_shared<autoware::universe_utils::TransformListener>(this);
  logger_configure_ = std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this);

  vehicle_twist_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "vehicle/twist_with_covariance", rclcpp::QoS{100},
    std::bind(&GyroOdometerNode::callback_vehicle_twist, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::QoS{100},
    std::bind(&GyroOdometerNode::callback_imu, this, std::placeholders::_1));

  twist_raw_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist_raw", rclcpp::QoS{10});
  twist_with_covariance_raw_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance_raw", rclcpp::QoS{10});

  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist", rclcpp::QoS{10});
  twist_with_covariance_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", rclcpp::QoS{10});

  diagnostics_ = std::make_unique<DiagnosticsModule>(this, "gyro_odometer_status");

  // TODO(YamatoAndo) createTimer
}

void GyroOdometerNode::callback_vehicle_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr vehicle_twist_msg_ptr)
{
  diagnostics_->clear();
  diagnostics_->add_key_value(
    "topic_time_stamp",
    static_cast<rclcpp::Time>(vehicle_twist_msg_ptr->header.stamp).nanoseconds());

  vehicle_twist_arrived_ = true;
  latest_vehicle_twist_ros_time_ = vehicle_twist_msg_ptr->header.stamp;
  vehicle_twist_queue_.push_back(*vehicle_twist_msg_ptr);
  concat_gyro_and_odometer();

  diagnostics_->publish(vehicle_twist_msg_ptr->header.stamp);
}

void GyroOdometerNode::callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  diagnostics_->clear();
  diagnostics_->add_key_value(
    "topic_time_stamp", static_cast<rclcpp::Time>(imu_msg_ptr->header.stamp).nanoseconds());

  imu_arrived_ = true;
  latest_imu_ros_time_ = imu_msg_ptr->header.stamp;
  gyro_queue_.push_back(*imu_msg_ptr);
  concat_gyro_and_odometer();

  diagnostics_->publish(imu_msg_ptr->header.stamp);
}

void GyroOdometerNode::concat_gyro_and_odometer()
{
  // check arrive first topic
  diagnostics_->add_key_value("is_arrived_first_vehicle_twist", vehicle_twist_arrived_);
  diagnostics_->add_key_value("is_arrived_first_imu", imu_arrived_);
  if (!vehicle_twist_arrived_) {
    std::stringstream message;
    message << "Twist msg is not subscribed";
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());

    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }
  if (!imu_arrived_) {
    std::stringstream message;
    message << "Imu msg is not subscribed";
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());

    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  // check timeout
  const double vehicle_twist_dt =
    std::abs((this->now() - latest_vehicle_twist_ros_time_).seconds());
  const double imu_dt = std::abs((this->now() - latest_imu_ros_time_).seconds());
  diagnostics_->add_key_value("vehicle_twist_time_stamp_dt", vehicle_twist_dt);
  diagnostics_->add_key_value("imu_time_stamp_dt", imu_dt);
  if (vehicle_twist_dt > message_timeout_sec_) {
    const std::string message = fmt::format(
      "Vehicle twist msg is timeout. vehicle_twist_dt: {}[sec], tolerance {}[sec]",
      vehicle_twist_dt, message_timeout_sec_);
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message);
    diagnostics_->update_level_and_message(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);

    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }
  if (imu_dt > message_timeout_sec_) {
    const std::string message = fmt::format(
      "Imu msg is timeout. imu_dt: {}[sec], tolerance {}[sec]", imu_dt, message_timeout_sec_);
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message);
    diagnostics_->update_level_and_message(diagnostic_msgs::msg::DiagnosticStatus::ERROR, message);

    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  // check queue size
  diagnostics_->add_key_value("vehicle_twist_queue_size", vehicle_twist_queue_.size());
  diagnostics_->add_key_value("imu_queue_size", gyro_queue_.size());
  if (vehicle_twist_queue_.empty()) {
    // not output error and clear queue
    return;
  }
  if (gyro_queue_.empty()) {
    // not output error and clear queue
    return;
  }

  // get transformation
  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener_->getLatestTransform(gyro_queue_.front().header.frame_id, output_frame_);

  const bool is_succeed_transform_imu = (tf_imu2base_ptr != nullptr);
  diagnostics_->add_key_value("is_succeed_transform_imu", is_succeed_transform_imu);
  if (!is_succeed_transform_imu) {
    std::stringstream message;
    message << "Please publish TF " << output_frame_ << " to "
            << gyro_queue_.front().header.frame_id;
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());

    vehicle_twist_queue_.clear();
    gyro_queue_.clear();
    return;
  }

  // transform gyro frame
  for (auto & gyro : gyro_queue_) {
    geometry_msgs::msg::Vector3Stamped angular_velocity;
    angular_velocity.header = gyro.header;
    angular_velocity.vector = gyro.angular_velocity;

    geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
    transformed_angular_velocity.header = tf_imu2base_ptr->header;
    tf2::doTransform(angular_velocity, transformed_angular_velocity, *tf_imu2base_ptr);

    gyro.header.frame_id = output_frame_;
    gyro.angular_velocity = transformed_angular_velocity.vector;
    gyro.angular_velocity_covariance = transform_covariance(gyro.angular_velocity_covariance);
  }

  using COV_IDX_XYZ = autoware::universe_utils::xyz_covariance_index::XYZ_COV_IDX;
  using COV_IDX_XYZRPY = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

  // calc mean, covariance
  double vx_mean = 0;
  geometry_msgs::msg::Vector3 gyro_mean{};
  double vx_covariance_original = 0;
  geometry_msgs::msg::Vector3 gyro_covariance_original{};
  for (const auto & vehicle_twist : vehicle_twist_queue_) {
    vx_mean += vehicle_twist.twist.twist.linear.x;
    vx_covariance_original += vehicle_twist.twist.covariance[0 * 6 + 0];
  }
  vx_mean /= static_cast<double>(vehicle_twist_queue_.size());
  vx_covariance_original /= static_cast<double>(vehicle_twist_queue_.size());

  for (const auto & gyro : gyro_queue_) {
    gyro_mean.x += gyro.angular_velocity.x;
    gyro_mean.y += gyro.angular_velocity.y;
    gyro_mean.z += gyro.angular_velocity.z;
    gyro_covariance_original.x += gyro.angular_velocity_covariance[COV_IDX_XYZ::X_X];
    gyro_covariance_original.y += gyro.angular_velocity_covariance[COV_IDX_XYZ::Y_Y];
    gyro_covariance_original.z += gyro.angular_velocity_covariance[COV_IDX_XYZ::Z_Z];
  }
  gyro_mean.x /= static_cast<double>(gyro_queue_.size());
  gyro_mean.y /= static_cast<double>(gyro_queue_.size());
  gyro_mean.z /= static_cast<double>(gyro_queue_.size());
  gyro_covariance_original.x /= static_cast<double>(gyro_queue_.size());
  gyro_covariance_original.y /= static_cast<double>(gyro_queue_.size());
  gyro_covariance_original.z /= static_cast<double>(gyro_queue_.size());

  // concat
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_cov;
  const auto latest_vehicle_twist_stamp = rclcpp::Time(vehicle_twist_queue_.back().header.stamp);
  const auto latest_imu_stamp = rclcpp::Time(gyro_queue_.back().header.stamp);
  if (latest_vehicle_twist_stamp < latest_imu_stamp) {
    twist_with_cov.header.stamp = latest_imu_stamp;
  } else {
    twist_with_cov.header.stamp = latest_vehicle_twist_stamp;
  }
  twist_with_cov.header.frame_id = gyro_queue_.front().header.frame_id;
  twist_with_cov.twist.twist.linear.x = vx_mean;
  twist_with_cov.twist.twist.angular = gyro_mean;

  // From a statistical point of view, here we reduce the covariances according to the number of
  // observed data
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::X_X] =
    vx_covariance_original / static_cast<double>(vehicle_twist_queue_.size());
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Y_Y] = 100000.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::Z_Z] = 100000.0;
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::ROLL_ROLL] =
    gyro_covariance_original.x / static_cast<double>(gyro_queue_.size());
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::PITCH_PITCH] =
    gyro_covariance_original.y / static_cast<double>(gyro_queue_.size());
  twist_with_cov.twist.covariance[COV_IDX_XYZRPY::YAW_YAW] =
    gyro_covariance_original.z / static_cast<double>(gyro_queue_.size());

  publish_data(twist_with_cov);

  vehicle_twist_queue_.clear();
  gyro_queue_.clear();
}

void GyroOdometerNode::publish_data(
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

}  // namespace autoware::gyro_odometer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gyro_odometer::GyroOdometerNode)
