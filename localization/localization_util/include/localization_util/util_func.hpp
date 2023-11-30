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

#ifndef LOCALIZATION_UTIL__UTIL_FUNC_HPP_
#define LOCALIZATION_UTIL__UTIL_FUNC_HPP_

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <deque>
#include <random>
#include <string>
#include <vector>

// ref by http://takacity.blog.fc2.com/blog-entry-69.html
std_msgs::msg::ColorRGBA exchange_color_crc(double x);

double calc_diff_for_radian(const double lhs_rad, const double rhs_rad);

// x: roll, y: pitch, z: yaw
geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::PoseStamped & pose);
geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

geometry_msgs::msg::Quaternion rpy_rad_to_quaternion(
  const double r_rad, const double p_rad, const double y_rad);
geometry_msgs::msg::Quaternion rpy_deg_to_quaternion(
  const double r_deg, const double p_deg, const double y_deg);

geometry_msgs::msg::Twist calc_twist(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b);

geometry_msgs::msg::PoseStamped interpolate_pose(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b,
  const rclcpp::Time & time_stamp);

geometry_msgs::msg::PoseStamped interpolate_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_a,
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_b, const rclcpp::Time & time_stamp);

Eigen::Affine3d pose_to_affine3d(const geometry_msgs::msg::Pose & ros_pose);
Eigen::Matrix4f pose_to_matrix4f(const geometry_msgs::msg::Pose & ros_pose);
geometry_msgs::msg::Pose matrix4f_to_pose(const Eigen::Matrix4f & eigen_pose_matrix);
Eigen::Vector3d point_to_vector3d(const geometry_msgs::msg::Point & ros_pos);

template <class T>
T transform(const T & input, const geometry_msgs::msg::TransformStamped & transform)
{
  T output;
  tf2::doTransform<T>(input, output, transform);
  return output;
}

double norm(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

void output_pose_with_cov_to_log(
  const rclcpp::Logger & logger, const std::string & prefix,
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_with_cov);

#endif  // LOCALIZATION_UTIL__UTIL_FUNC_HPP_
