// Copyright 2022 The Autoware Contributors
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

#ifndef POINTCLOUD_MAP_LOADER__UTILS_HPP_
#define POINTCLOUD_MAP_LOADER__UTILS_HPP_

#include <autoware_map_msgs/msg/area_info.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <pcl/common/common.h>

struct PCDFileMetadata
{
  pcl::PointXYZ min;
  pcl::PointXYZ max;
};

bool sphereAndBoxOverlapExists(
  const geometry_msgs::msg::Point position, const double radius, const pcl::PointXYZ position_min,
  const pcl::PointXYZ position_max);
bool isGridWithinQueriedArea(
  const autoware_map_msgs::msg::AreaInfo area, const PCDFileMetadata metadata);

#endif  // POINTCLOUD_MAP_LOADER__UTILS_HPP_
