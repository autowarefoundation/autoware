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
#include <yaml-cpp/yaml.h>

#include <map>
#include <string>
#include <vector>

struct PCDFileMetadata
{
  pcl::PointXYZ min;
  pcl::PointXYZ max;
  bool operator==(const PCDFileMetadata & other) const
  {
    return min.x == other.min.x && min.y == other.min.y && min.z == other.min.z &&
           max.x == other.max.x && max.y == other.max.y && max.z == other.max.z;
  }
};

std::map<std::string, PCDFileMetadata> loadPCDMetadata(const std::string & pcd_metadata_path);
std::map<std::string, PCDFileMetadata> replaceWithAbsolutePath(
  const std::map<std::string, PCDFileMetadata> & pcd_metadata_path,
  const std::vector<std::string> & pcd_paths);

bool cylinderAndBoxOverlapExists(
  const double center_x, const double center_y, const double radius,
  const pcl::PointXYZ position_min, const pcl::PointXYZ position_max);
bool isGridWithinQueriedArea(
  const autoware_map_msgs::msg::AreaInfo area, const PCDFileMetadata metadata);

#endif  // POINTCLOUD_MAP_LOADER__UTILS_HPP_
