// Copyright 2023- Autoware Foundation
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

#include "pose_instability_detector.hpp"

#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <string>

PoseInstabilityDetector::PoseInstabilityDetector(const rclcpp::NodeOptions & options)
: Node("pose_instability_detector", options),
  threshold_diff_position_x_(this->declare_parameter<double>("threshold_diff_position_x")),
  threshold_diff_position_y_(this->declare_parameter<double>("threshold_diff_position_y")),
  threshold_diff_position_z_(this->declare_parameter<double>("threshold_diff_position_z")),
  threshold_diff_angle_x_(this->declare_parameter<double>("threshold_diff_angle_x")),
  threshold_diff_angle_y_(this->declare_parameter<double>("threshold_diff_angle_y")),
  threshold_diff_angle_z_(this->declare_parameter<double>("threshold_diff_angle_z"))
{
  odometry_sub_ = this->create_subscription<Odometry>(
    "~/input/odometry", 10,
    std::bind(&PoseInstabilityDetector::callback_odometry, this, std::placeholders::_1));

  twist_sub_ = this->create_subscription<TwistWithCovarianceStamped>(
    "~/input/twist", 10,
    std::bind(&PoseInstabilityDetector::callback_twist, this, std::placeholders::_1));

  const double interval_sec = this->declare_parameter<double>("interval_sec");
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), std::chrono::duration<double>(interval_sec),
    std::bind(&PoseInstabilityDetector::callback_timer, this));

  diff_pose_pub_ = this->create_publisher<PoseStamped>("~/debug/diff_pose", 10);
  diagnostics_pub_ = this->create_publisher<DiagnosticArray>("/diagnostics", 10);
}

void PoseInstabilityDetector::callback_odometry(Odometry::ConstSharedPtr odometry_msg_ptr)
{
  latest_odometry_ = *odometry_msg_ptr;
}

void PoseInstabilityDetector::callback_twist(
  TwistWithCovarianceStamped::ConstSharedPtr twist_msg_ptr)
{
  twist_buffer_.push_back(*twist_msg_ptr);
}

void PoseInstabilityDetector::callback_timer()
{
  if (latest_odometry_ == std::nullopt) {
    return;
  }
  if (prev_odometry_ == std::nullopt) {
    prev_odometry_ = latest_odometry_;
    return;
  }

  auto quat_to_rpy = [](const Quaternion & quat) {
    tf2::Quaternion tf2_quat(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 mat(tf2_quat);
    double roll{};
    double pitch{};
    double yaw{};
    mat.getRPY(roll, pitch, yaw);
    return std::make_tuple(roll, pitch, yaw);
  };

  Pose pose = prev_odometry_->pose.pose;
  rclcpp::Time prev_time = rclcpp::Time(prev_odometry_->header.stamp);
  for (const TwistWithCovarianceStamped & twist_with_cov : twist_buffer_) {
    const Twist twist = twist_with_cov.twist.twist;

    const rclcpp::Time curr_time = rclcpp::Time(twist_with_cov.header.stamp);
    if (curr_time > latest_odometry_->header.stamp) {
      break;
    }

    const rclcpp::Duration time_diff = curr_time - prev_time;
    const double time_diff_sec = time_diff.seconds();
    if (time_diff_sec < 0.0) {
      continue;
    }

    // quat to rpy
    auto [ang_x, ang_y, ang_z] = quat_to_rpy(pose.orientation);

    // rpy update
    ang_x += twist.angular.x * time_diff_sec;
    ang_y += twist.angular.y * time_diff_sec;
    ang_z += twist.angular.z * time_diff_sec;
    tf2::Quaternion quat;
    quat.setRPY(ang_x, ang_y, ang_z);

    // Convert twist to world frame
    tf2::Vector3 linear_velocity(twist.linear.x, twist.linear.y, twist.linear.z);
    linear_velocity = tf2::quatRotate(quat, linear_velocity);

    // update
    pose.position.x += linear_velocity.x() * time_diff_sec;
    pose.position.y += linear_velocity.y() * time_diff_sec;
    pose.position.z += linear_velocity.z() * time_diff_sec;
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    prev_time = curr_time;
  }

  // compare pose and latest_odometry_
  const Pose latest_ekf_pose = latest_odometry_->pose.pose;
  const Pose ekf_to_odom = tier4_autoware_utils::inverseTransformPose(pose, latest_ekf_pose);
  const geometry_msgs::msg::Point pos = ekf_to_odom.position;
  const auto [ang_x, ang_y, ang_z] = quat_to_rpy(ekf_to_odom.orientation);
  const std::vector<double> values = {pos.x, pos.y, pos.z, ang_x, ang_y, ang_z};

  const rclcpp::Time stamp = latest_odometry_->header.stamp;

  // publish diff_pose for debug
  PoseStamped diff_pose;
  diff_pose.header = latest_odometry_->header;
  diff_pose.pose = ekf_to_odom;
  diff_pose_pub_->publish(diff_pose);

  const std::vector<double> thresholds = {threshold_diff_position_x_, threshold_diff_position_y_,
                                          threshold_diff_position_z_, threshold_diff_angle_x_,
                                          threshold_diff_angle_y_,    threshold_diff_angle_z_};

  const std::vector<std::string> labels = {"diff_position_x", "diff_position_y", "diff_position_z",
                                           "diff_angle_x",    "diff_angle_y",    "diff_angle_z"};

  // publish diagnostics
  DiagnosticStatus status;
  status.name = "localization: pose_instability_detector";
  status.hardware_id = this->get_name();
  bool all_ok = true;

  for (size_t i = 0; i < values.size(); ++i) {
    const bool ok = (std::abs(values[i]) < thresholds[i]);
    all_ok &= ok;
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = labels[i] + ":threshold";
    kv.value = std::to_string(thresholds[i]);
    status.values.push_back(kv);
    kv.key = labels[i] + ":value";
    kv.value = std::to_string(values[i]);
    status.values.push_back(kv);
    kv.key = labels[i] + ":status";
    kv.value = (ok ? "OK" : "WARN");
    status.values.push_back(kv);
  }
  status.level = (all_ok ? DiagnosticStatus::OK : DiagnosticStatus::WARN);
  status.message = (all_ok ? "OK" : "WARN");

  DiagnosticArray diagnostics;
  diagnostics.header.stamp = stamp;
  diagnostics.status.emplace_back(status);
  diagnostics_pub_->publish(diagnostics);

  // prepare for next loop
  prev_odometry_ = latest_odometry_;
  twist_buffer_.clear();
}
