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

#ifndef POINTCLOUD_UTILS_HPP_
#define POINTCLOUD_UTILS_HPP_

#include "autoware/universe_utils/ros/transform_listener.hpp"
#include "obstacles.hpp"
#include "types.hpp"

#include <geometry_msgs/msg/transform.hpp>

#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
{

/// @brief filter the pointcloud to keep only relevant points
/// @param[in,out] pointcloud to filter
/// @param[in] masks obstacle masks used to filter the pointcloud
void filterPointCloud(PointCloud::Ptr pointcloud, const ObstacleMasks & masks);

/// @brief extract obstacles from the given pointcloud
/// @param[in] pointcloud input pointcloud
/// @return extracted obstacles
multipoint_t extractObstacles(const PointCloud & pointcloud);

}  // namespace autoware::motion_velocity_planner::obstacle_velocity_limiter

#endif  // POINTCLOUD_UTILS_HPP_
