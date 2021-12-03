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

#include "localization_error_monitor/node.hpp"

#include <Eigen/Dense>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>

LocalizationErrorMonitor::LocalizationErrorMonitor()
: Node("localization_error_monitor"), updater_(this)
{
  scale_ = this->declare_parameter("scale", 3.0);
  error_ellipse_size_ = this->declare_parameter("error_ellipse_size", 1.0);
  warn_ellipse_size_ = this->declare_parameter("warn_ellipse_size", 0.8);

  error_ellipse_size_lateral_direction_ =
    this->declare_parameter("error_ellipse_size_lateral_direction", 0.3);
  warn_ellipse_size_lateral_direction_ =
    this->declare_parameter("warn_ellipse_size_lateral_direction", 0.2);

  pose_with_cov_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "input/pose_with_cov", 1,
    std::bind(&LocalizationErrorMonitor::onPoseWithCovariance, this, std::placeholders::_1));

  // QoS setup
  rclcpp::QoS durable_qos(1);
  durable_qos.transient_local();  // option for latching
  ellipse_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("debug/ellipse_marker", durable_qos);

  updater_.setHardwareID("localization_error_monitor");
  updater_.add("localization_accuracy", this, &LocalizationErrorMonitor::checkLocalizationAccuracy);
  updater_.add(
    "localization_accuracy_lateral_direction", this,
    &LocalizationErrorMonitor::checkLocalizationAccuracyLateralDirection);

  // Set timer
  auto timer_callback = std::bind(&LocalizationErrorMonitor::onTimer, this);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.1));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void LocalizationErrorMonitor::onTimer() { updater_.force_update(); }

void LocalizationErrorMonitor::checkLocalizationAccuracy(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("localization_accuracy", ellipse_.long_radius);
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "ellipse size is within the expected range";
  if (warn_ellipse_size_ <= ellipse_.long_radius) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag_message = "ellipse size is too large";
  }
  if (error_ellipse_size_ <= ellipse_.long_radius) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "ellipse size is over the expected range";
  }
  stat.summary(diag_level, diag_message);
}

void LocalizationErrorMonitor::checkLocalizationAccuracyLateralDirection(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  stat.add("localization_accuracy_lateral_direction", ellipse_.size_lateral_direction);
  int8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string diag_message = "ellipse size along lateral direction is within the expected range";
  if (warn_ellipse_size_lateral_direction_ <= ellipse_.size_lateral_direction) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag_message = "ellipse size along lateral direction is too large";
  }
  if (error_ellipse_size_lateral_direction_ <= ellipse_.size_lateral_direction) {
    diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diag_message = "ellipse size along lateral direction is over the expected range";
  }
  stat.summary(diag_level, diag_message);
}

visualization_msgs::msg::Marker LocalizationErrorMonitor::createEllipseMarker(
  const Ellipse & ellipse,
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_with_cov)
{
  tf2::Quaternion quat;
  quat.setEuler(0, 0, ellipse.yaw);

  const double ellipse_long_radius = std::min(ellipse.long_radius, 30.0);
  const double ellipse_short_radius = std::min(ellipse.short_radius, 30.0);
  visualization_msgs::msg::Marker marker;
  marker.header = pose_with_cov->header;
  marker.header.stamp = this->now();
  marker.ns = "error_ellipse";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = pose_with_cov->pose.pose;
  marker.pose.orientation = tf2::toMsg(quat);
  marker.scale.x = ellipse_long_radius * 2;
  marker.scale.y = ellipse_short_radius * 2;
  marker.scale.z = 0.01;
  marker.color.a = 0.1;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  return marker;
}

void LocalizationErrorMonitor::onPoseWithCovariance(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr input_msg)
{
  // create xy covariance (2x2 matrix)
  // input geometry_msgs::PoseWithCovariance contain 6x6 matrix
  Eigen::Matrix2d xy_covariance;
  const auto cov = input_msg->pose.covariance;
  xy_covariance(0, 0) = cov[0 * 6 + 0];
  xy_covariance(0, 1) = cov[0 * 6 + 1];
  xy_covariance(1, 0) = cov[1 * 6 + 0];
  xy_covariance(1, 1) = cov[1 * 6 + 1];

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(xy_covariance);

  // eigen values and vectors are sorted in ascending order
  ellipse_.long_radius = scale_ * std::sqrt(eigensolver.eigenvalues()(1));
  ellipse_.short_radius = scale_ * std::sqrt(eigensolver.eigenvalues()(0));

  // principal component vector
  const Eigen::Vector2d pc_vector = eigensolver.eigenvectors().col(1);
  ellipse_.yaw = std::atan2(pc_vector.y(), pc_vector.x());

  // ellipse size along lateral direction (body-frame)
  ellipse_.P = xy_covariance;
  const double yaw_vehicle = tf2::getYaw(input_msg->pose.pose.orientation);
  ellipse_.size_lateral_direction =
    scale_ * measureSizeEllipseAlongBodyFrame(ellipse_.P.inverse(), yaw_vehicle);

  const auto ellipse_marker = createEllipseMarker(ellipse_, input_msg);
  ellipse_marker_pub_->publish(ellipse_marker);
}

double LocalizationErrorMonitor::measureSizeEllipseAlongBodyFrame(
  const Eigen::Matrix2d & Pinv, const double theta)
{
  Eigen::MatrixXd e(2, 1);
  e(0, 0) = std::cos(theta);
  e(1, 0) = std::sin(theta);

  double d = std::sqrt((e.transpose() * Pinv * e)(0, 0) / Pinv.determinant());
  return d;
}
