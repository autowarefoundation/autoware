// Copyright 2024 Autoware Foundation
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

#ifndef LOCALIZATION_UTIL__COVARIANCE_ELLIPSE_HPP_
#define LOCALIZATION_UTIL__COVARIANCE_ELLIPSE_HPP_

#include <Eigen/Dense>

#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace autoware::localization_util
{

struct Ellipse
{
  double long_radius;
  double short_radius;
  double yaw;
  Eigen::Matrix2d P;
  double size_lateral_direction;
};

Ellipse calculate_xy_ellipse(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance, const double scale);

visualization_msgs::msg::Marker create_ellipse_marker(
  const Ellipse & ellipse, const std_msgs::msg::Header & header,
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance);

}  // namespace autoware::localization_util

#endif  // LOCALIZATION_UTIL__COVARIANCE_ELLIPSE_HPP_
