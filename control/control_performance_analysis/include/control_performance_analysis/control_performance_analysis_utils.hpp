// Copyright 2021 - 2022 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#ifndef CONTROL_PERFORMANCE_ANALYSIS__CONTROL_PERFORMANCE_ANALYSIS_UTILS_HPP_
#define CONTROL_PERFORMANCE_ANALYSIS__CONTROL_PERFORMANCE_ANALYSIS_UTILS_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2/utils.h>

#include <cmath>
#include <utility>
#include <vector>

namespace control_performance_analysis
{
namespace utils
{
// Right hand sided tangent and normal vectors
inline std::vector<double> getTangentVector(double yaw_angle)
{
  return std::vector<double>{cos(yaw_angle), sin(yaw_angle)};
}

inline std::vector<double> getNormalVector(double yaw_angle)
{
  return std::vector<double>{-sin(yaw_angle), cos(yaw_angle)};
}

inline std::pair<double, double> computeLateralLongitudinalError(
  const geometry_msgs::msg::Point & closest_point_position,
  const geometry_msgs::msg::Point & vehicle_position, const double & desired_yaw_angle)
{
  // Vector to path point originating from the vehicle r - rd
  std::vector<double> vector_to_path_point{
    vehicle_position.x - closest_point_position.x, vehicle_position.y - closest_point_position.y};

  double lateral_error = -sin(desired_yaw_angle) * vector_to_path_point[0] +
                         cos(desired_yaw_angle) * vector_to_path_point[1];
  double longitudinal_error = cos(desired_yaw_angle) * vector_to_path_point[0] +
                              sin(desired_yaw_angle) * vector_to_path_point[1];

  return {lateral_error, longitudinal_error};
}

inline geometry_msgs::msg::Quaternion createOrientationMsgFromYaw(double yaw_angle)
{
  geometry_msgs::msg::Quaternion orientation_msg;
  double roll_angle = 0.0;
  double pitch_angle = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(roll_angle, pitch_angle, yaw_angle);

  orientation_msg.w = quaternion.getW();
  orientation_msg.x = quaternion.getX();
  orientation_msg.y = quaternion.getY();
  orientation_msg.z = quaternion.getZ();

  return orientation_msg;
}

// p-points a, b contains [x, y] coordinates.
double determinant(std::array<double, 2> const & a, std::array<double, 2> const & b);

double curvatureFromThreePoints(
  std::array<double, 2> const & a, std::array<double, 2> const & b,
  std::array<double, 2> const & c);

}  // namespace utils
}  // namespace control_performance_analysis

#endif  // CONTROL_PERFORMANCE_ANALYSIS__CONTROL_PERFORMANCE_ANALYSIS_UTILS_HPP_
