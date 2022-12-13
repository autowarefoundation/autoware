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

#ifndef POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_
#define POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_

#include "differential_map_loader_module.hpp"
#include "partial_map_loader_module.hpp"
#include "pointcloud_map_loader_module.hpp"

#include <rclcpp/rclcpp.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

class PointCloudMapLoaderNode : public rclcpp::Node
{
public:
  explicit PointCloudMapLoaderNode(const rclcpp::NodeOptions & options);

private:
  // ros param
  std::map<std::string, PCDFileMetadata> pcd_metadata_dict_;

  std::unique_ptr<PointcloudMapLoaderModule> pcd_map_loader_;
  std::unique_ptr<PointcloudMapLoaderModule> downsampled_pcd_map_loader_;
  std::unique_ptr<PartialMapLoaderModule> partial_map_loader_;
  std::unique_ptr<DifferentialMapLoaderModule> differential_map_loader_;

  std::vector<std::string> getPcdPaths(
    const std::vector<std::string> & pcd_paths_or_directory) const;
  std::map<std::string, PCDFileMetadata> generatePCDMetadata(
    const std::vector<std::string> & pcd_paths) const;
};

#endif  // POINTCLOUD_MAP_LOADER__POINTCLOUD_MAP_LOADER_NODE_HPP_
