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

#include "utils.hpp"

#include <fmt/format.h>

#include <map>
#include <string>
#include <vector>

std::map<std::string, PCDFileMetadata> loadPCDMetadata(const std::string & pcd_metadata_path)
{
  YAML::Node config = YAML::LoadFile(pcd_metadata_path);

  std::map<std::string, PCDFileMetadata> metadata;

  for (const auto & node : config) {
    if (
      node.first.as<std::string>() == "x_resolution" ||
      node.first.as<std::string>() == "y_resolution") {
      continue;
    }

    std::string key = node.first.as<std::string>();
    std::vector<int> values = node.second.as<std::vector<int>>();

    PCDFileMetadata fileMetadata;
    fileMetadata.min.x = values[0];
    fileMetadata.min.y = values[1];
    fileMetadata.max.x = values[0] + config["x_resolution"].as<float>();
    fileMetadata.max.y = values[1] + config["y_resolution"].as<float>();

    metadata[key] = fileMetadata;
  }

  return metadata;
}

std::map<std::string, PCDFileMetadata> replaceWithAbsolutePath(
  const std::map<std::string, PCDFileMetadata> & pcd_metadata_path,
  const std::vector<std::string> & pcd_paths)
{
  std::map<std::string, PCDFileMetadata> absolute_path_map;
  for (const auto & path : pcd_paths) {
    std::string filename = path.substr(path.find_last_of("/\\") + 1);
    auto it = pcd_metadata_path.find(filename);
    if (it != pcd_metadata_path.end()) {
      absolute_path_map[path] = it->second;
    }
  }
  return absolute_path_map;
}

bool cylinderAndBoxOverlapExists(
  const double center_x, const double center_y, const double radius,
  const pcl::PointXYZ box_min_point, const pcl::PointXYZ box_max_point)
{
  // Collision detection with x-y plane (circular base of the cylinder)
  if (
    box_min_point.x - radius <= center_x && center_x <= box_max_point.x + radius &&
    box_min_point.y - radius <= center_y && center_y <= box_max_point.y + radius) {
    return true;
  }

  // Collision detection with box edges
  const double dx0 = center_x - box_min_point.x;
  const double dx1 = center_x - box_max_point.x;
  const double dy0 = center_y - box_min_point.y;
  const double dy1 = center_y - box_max_point.y;

  if (
    std::hypot(dx0, dy0) <= radius || std::hypot(dx1, dy0) <= radius ||
    std::hypot(dx0, dy1) <= radius || std::hypot(dx1, dy1) <= radius) {
    return true;
  }

  return false;
}

bool isGridWithinQueriedArea(
  const autoware_map_msgs::msg::AreaInfo area, const PCDFileMetadata metadata)
{
  // Currently, the area load only supports cylindrical area
  double center_x = area.center_x;
  double center_y = area.center_y;
  double radius = area.radius;
  bool res = cylinderAndBoxOverlapExists(center_x, center_y, radius, metadata.min, metadata.max);
  return res;
}
