// Copyright 2022 TIER IV, Inc.
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

#ifndef IMAGE_PROJECTION_BASED_FUSION__UTILS__UTILS_HPP_
#define IMAGE_PROJECTION_BASED_FUSION__UTILS__UTILS_HPP_

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <optional>
#include <string>

namespace image_projection_based_fusion
{

std::optional<geometry_msgs::msg::TransformStamped> getTransformStamped(
  const tf2_ros::Buffer & tf_buffer, const std::string & target_frame_id,
  const std::string & source_frame_id, const rclcpp::Time & time);

Eigen::Affine3d transformToEigen(const geometry_msgs::msg::Transform & t);

}  // namespace image_projection_based_fusion

#endif  // IMAGE_PROJECTION_BASED_FUSION__UTILS__UTILS_HPP_
