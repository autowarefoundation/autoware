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

#ifndef OBSTACLE_VELOCITY_LIMITER__POINTCLOUD_UTILS_HPP_
#define OBSTACLE_VELOCITY_LIMITER__POINTCLOUD_UTILS_HPP_

#include "obstacle_velocity_limiter/obstacles.hpp"
#include "obstacle_velocity_limiter/types.hpp"
#include "tier4_autoware_utils/ros/transform_listener.hpp"

#include <geometry_msgs/msg/transform.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace obstacle_velocity_limiter
{

/// @brief return the pointcloud msg transformed and converted to PCL format
/// @param[in] pointcloud_msg pointcloud to transform
/// @param[in] transform_listener used to retrieve the latest transform
/// @param[in] target_frame frame of the returned pointcloud
/// @return PCL pointcloud
pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
  const PointCloud & pointcloud_msg, tier4_autoware_utils::TransformListener & transform_listener,
  const std::string & target_frame);

/// @brief filter the pointcloud to keep only relevent points
/// @param[in,out] pointcloud to filter
/// @param[in] masks obstacle masks used to filter the pointcloud
void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const ObstacleMasks & masks);

/// @brief extract obstacles from the given pointcloud
/// @param[in] pointcloud input pointcloud
/// @return extracted obstacles
multipoint_t extractObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);

}  // namespace obstacle_velocity_limiter

#endif  // OBSTACLE_VELOCITY_LIMITER__POINTCLOUD_UTILS_HPP_
