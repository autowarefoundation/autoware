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

#ifndef TIER4_AUTOWARE_UTILS__TRANSFORM__TRANSFORMS_HPP_
#define TIER4_AUTOWARE_UTILS__TRANSFORM__TRANSFORMS_HPP_

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

namespace tier4_autoware_utils
{
template <typename PointT>
void transformPointCloud(
  const pcl::PointCloud<PointT> & cloud_in, pcl::PointCloud<PointT> & cloud_out,
  const Eigen::Matrix<float, 4, 4> & transform)
{
  if (cloud_in.empty() || cloud_in.width == 0) {
    RCLCPP_WARN(rclcpp::get_logger("transformPointCloud"), "input point cloud is empty!");
  } else {
    pcl::transformPointCloud(cloud_in, cloud_out, transform);
  }
}

template <typename PointT>
void transformPointCloud(
  const pcl::PointCloud<PointT> & cloud_in, pcl::PointCloud<PointT> & cloud_out,
  const Eigen::Affine3f & transform)
{
  if (cloud_in.empty() || cloud_in.width == 0) {
    RCLCPP_WARN(rclcpp::get_logger("transformPointCloud"), "input point cloud is empty!");
  } else {
    pcl::transformPointCloud(cloud_in, cloud_out, transform);
  }
}
}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__TRANSFORM__TRANSFORMS_HPP_
