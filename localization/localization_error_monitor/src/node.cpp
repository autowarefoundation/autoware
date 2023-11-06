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

#include "localization_error_monitor/diagnostics.hpp"

#include <Eigen/Dense>

#include <tf2/utils.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>

LocalizationErrorMonitor::LocalizationErrorMonitor() : Node("localization_error_monitor")
{
  scale_ = this->declare_parameter<double>("scale");
  error_ellipse_size_ = this->declare_parameter<double>("error_ellipse_size");
  warn_ellipse_size_ = this->declare_parameter<double>("warn_ellipse_size");

  error_ellipse_size_lateral_direction_ =
    this->declare_parameter<double>("error_ellipse_size_lateral_direction");
  warn_ellipse_size_lateral_direction_ =
    this->declare_parameter<double>("warn_ellipse_size_lateral_direction");

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "input/odom", 1, std::bind(&LocalizationErrorMonitor::onOdom, this, std::placeholders::_1));

  // QoS setup
  rclcpp::QoS durable_qos(1);
  durable_qos.transient_local();  // option for latching
  ellipse_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("debug/ellipse_marker", durable_qos);

  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);
}

visualization_msgs::msg::Marker LocalizationErrorMonitor::createEllipseMarker(
  const Ellipse & ellipse, nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  tf2::Quaternion quat;
  quat.setEuler(0, 0, ellipse.yaw);

  const double ellipse_long_radius = std::min(ellipse.long_radius, 30.0);
  const double ellipse_short_radius = std::min(ellipse.short_radius, 30.0);
  visualization_msgs::msg::Marker marker;
  marker.header = odom->header;
  marker.header.stamp = this->now();
  marker.ns = "error_ellipse";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = odom->pose.pose;
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

void LocalizationErrorMonitor::onOdom(nav_msgs::msg::Odometry::ConstSharedPtr input_msg)
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

  // diagnostics
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> diag_status_array;
  diag_status_array.push_back(
    checkLocalizationAccuracy(ellipse_.long_radius, warn_ellipse_size_, error_ellipse_size_));
  diag_status_array.push_back(checkLocalizationAccuracyLateralDirection(
    ellipse_.size_lateral_direction, warn_ellipse_size_lateral_direction_,
    error_ellipse_size_lateral_direction_));

  diagnostic_msgs::msg::DiagnosticStatus diag_merged_status;
  diag_merged_status = mergeDiagnosticStatus(diag_status_array);
  diag_merged_status.name = "localization: " + std::string(this->get_name());
  diag_merged_status.hardware_id = this->get_name();

  diagnostic_msgs::msg::DiagnosticArray diag_msg;
  diag_msg.header.stamp = this->now();
  diag_msg.status.push_back(diag_merged_status);
  diag_pub_->publish(diag_msg);
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
