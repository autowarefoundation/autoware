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

#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <utility>

namespace imu_corrector
{
GyroBiasEstimator::GyroBiasEstimator()
: Node("gyro_bias_validator"),
  gyro_bias_threshold_(declare_parameter<double>("gyro_bias_threshold")),
  angular_velocity_offset_x_(declare_parameter<double>("angular_velocity_offset_x")),
  angular_velocity_offset_y_(declare_parameter<double>("angular_velocity_offset_y")),
  angular_velocity_offset_z_(declare_parameter<double>("angular_velocity_offset_z")),
  timer_callback_interval_sec_(declare_parameter<double>("timer_callback_interval_sec")),
  straight_motion_ang_vel_upper_limit_(
    declare_parameter<double>("straight_motion_ang_vel_upper_limit")),
  updater_(this),
  gyro_bias_(std::nullopt)
{
  updater_.setHardwareID(get_name());
  updater_.add("gyro_bias_validator", this, &GyroBiasEstimator::update_diagnostics);

  gyro_bias_estimation_module_ = std::make_unique<GyroBiasEstimationModule>();

  imu_sub_ = create_subscription<Imu>(
    "~/input/imu_raw", rclcpp::SensorDataQoS(),
    [this](const Imu::ConstSharedPtr msg) { callback_imu(msg); });
  odom_sub_ = create_subscription<Odometry>(
    "~/input/odom", rclcpp::SensorDataQoS(),
    [this](const Odometry::ConstSharedPtr msg) { callback_odom(msg); });
  gyro_bias_pub_ = create_publisher<Vector3Stamped>("~/output/gyro_bias", rclcpp::SensorDataQoS());

  auto timer_callback = std::bind(&GyroBiasEstimator::timer_callback, this);
  auto period_control = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timer_callback_interval_sec_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_control, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);

  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);
}

void GyroBiasEstimator::callback_imu(const Imu::ConstSharedPtr imu_msg_ptr)
{
  imu_frame_ = imu_msg_ptr->header.frame_id;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener_->getLatestTransform(imu_frame_, output_frame_);
  if (!tf_imu2base_ptr) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", output_frame_.c_str(),
      (imu_frame_).c_str());
    return;
  }

  geometry_msgs::msg::Vector3Stamped gyro;
  gyro.header.stamp = imu_msg_ptr->header.stamp;
  gyro.vector = transform_vector3(imu_msg_ptr->angular_velocity, *tf_imu2base_ptr);

  gyro_all_.push_back(gyro);

  // Publish results for debugging
  if (gyro_bias_ != std::nullopt) {
    Vector3Stamped gyro_bias_msg;

    gyro_bias_msg.header.stamp = this->now();
    gyro_bias_msg.vector = gyro_bias_.value();

    gyro_bias_pub_->publish(gyro_bias_msg);
  }
}

void GyroBiasEstimator::callback_odom(const Odometry::ConstSharedPtr odom_msg_ptr)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom_msg_ptr->header;
  pose.pose = odom_msg_ptr->pose.pose;
  pose_buf_.push_back(pose);
}

void GyroBiasEstimator::timer_callback()
{
  if (pose_buf_.empty()) {
    return;
  }

  // Copy data
  const std::vector<geometry_msgs::msg::PoseStamped> pose_buf = pose_buf_;
  const std::vector<geometry_msgs::msg::Vector3Stamped> gyro_all = gyro_all_;
  pose_buf_.clear();
  gyro_all_.clear();

  // Check time
  const rclcpp::Time t0_rclcpp_time = rclcpp::Time(pose_buf.front().header.stamp);
  const rclcpp::Time t1_rclcpp_time = rclcpp::Time(pose_buf.back().header.stamp);
  if (t1_rclcpp_time <= t0_rclcpp_time) {
    return;
  }

  // Filter gyro data
  std::vector<geometry_msgs::msg::Vector3Stamped> gyro_filtered;
  for (const auto & gyro : gyro_all) {
    const rclcpp::Time t = rclcpp::Time(gyro.header.stamp);
    if (t0_rclcpp_time <= t && t < t1_rclcpp_time) {
      gyro_filtered.push_back(gyro);
    }
  }

  // Check gyro data size
  // Data size must be greater than or equal to 2 since the time difference will be taken later
  if (gyro_filtered.size() <= 1) {
    return;
  }

  // Check if the vehicle is moving straight
  const geometry_msgs::msg::Vector3 rpy_0 =
    tier4_autoware_utils::getRPY(pose_buf.front().pose.orientation);
  const geometry_msgs::msg::Vector3 rpy_1 =
    tier4_autoware_utils::getRPY(pose_buf.back().pose.orientation);
  const double yaw_diff = std::abs(tier4_autoware_utils::normalizeRadian(rpy_1.z - rpy_0.z));
  const double time_diff = (t1_rclcpp_time - t0_rclcpp_time).seconds();
  const double yaw_vel = yaw_diff / time_diff;
  const bool is_straight = (yaw_vel < straight_motion_ang_vel_upper_limit_);
  if (!is_straight) {
    return;
  }

  // Calculate gyro bias
  gyro_bias_estimation_module_->update_bias(pose_buf, gyro_filtered);

  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_base2imu_ptr =
    transform_listener_->getLatestTransform(output_frame_, imu_frame_);
  if (!tf_base2imu_ptr) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", imu_frame_.c_str(), output_frame_.c_str());
    return;
  }
  gyro_bias_ =
    transform_vector3(gyro_bias_estimation_module_->get_bias_base_link(), *tf_base2imu_ptr);

  updater_.force_update();
}

geometry_msgs::msg::Vector3 GyroBiasEstimator::transform_vector3(
  const geometry_msgs::msg::Vector3 & vec, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::Vector3Stamped vec_stamped;
  vec_stamped.vector = vec;

  geometry_msgs::msg::Vector3Stamped vec_stamped_transformed;
  tf2::doTransform(vec_stamped, vec_stamped_transformed, transform);
  return vec_stamped_transformed.vector;
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
