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

#ifndef POINTCLOUD_MAP_LOADER__DIFFERENTIAL_MAP_LOADER_MODULE_HPP_
#define POINTCLOUD_MAP_LOADER__DIFFERENTIAL_MAP_LOADER_MODULE_HPP_

#include "utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_map_msgs/srv/get_differential_point_cloud_map.hpp"

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <string>
#include <vector>

class DifferentialMapLoaderModule
{
  using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

public:
  explicit DifferentialMapLoaderModule(
    rclcpp::Node * node, const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict);

private:
  rclcpp::Logger logger_;

  std::map<std::string, PCDFileMetadata> all_pcd_file_metadata_dict_;
  rclcpp::Service<GetDifferentialPointCloudMap>::SharedPtr get_differential_pcd_maps_service_;

  bool onServiceGetDifferentialPointCloudMap(
    GetDifferentialPointCloudMap::Request::SharedPtr req,
    GetDifferentialPointCloudMap::Response::SharedPtr res);
  void differentialAreaLoad(
    const autoware_map_msgs::msg::AreaInfo & area_info, const std::vector<std::string> & cached_ids,
    GetDifferentialPointCloudMap::Response::SharedPtr & response) const;
  autoware_map_msgs::msg::PointCloudMapCellWithID loadPointCloudMapCellWithID(
    const std::string & path, const std::string & map_id) const;
};

#endif  // POINTCLOUD_MAP_LOADER__DIFFERENTIAL_MAP_LOADER_MODULE_HPP_
