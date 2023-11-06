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

#ifndef LOCALIZATION_ERROR_MONITOR__NODE_HPP_
#define LOCALIZATION_ERROR_MONITOR__NODE_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/logger_level_configure.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

struct Ellipse
{
  double long_radius;
  double short_radius;
  double yaw;
  Eigen::Matrix2d P;
  double size_lateral_direction;
};

class LocalizationErrorMonitor : public rclcpp::Node
{
private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ellipse_marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;

  double scale_;
  double error_ellipse_size_;
  double warn_ellipse_size_;
  double error_ellipse_size_lateral_direction_;
  double warn_ellipse_size_lateral_direction_;
  Ellipse ellipse_;

  void onOdom(nav_msgs::msg::Odometry::ConstSharedPtr input_msg);
  visualization_msgs::msg::Marker createEllipseMarker(
    const Ellipse & ellipse, nav_msgs::msg::Odometry::ConstSharedPtr odom);
  double measureSizeEllipseAlongBodyFrame(const Eigen::Matrix2d & Pinv, double theta);

public:
  LocalizationErrorMonitor();
  ~LocalizationErrorMonitor() = default;
};
#endif  // LOCALIZATION_ERROR_MONITOR__NODE_HPP_
