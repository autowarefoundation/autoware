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

#include "partial_map_loader_module.hpp"

PartialMapLoaderModule::PartialMapLoaderModule(
  rclcpp::Node * node, const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict)
: logger_(node->get_logger()), all_pcd_file_metadata_dict_(pcd_file_metadata_dict)
{
  get_partial_pcd_maps_service_ = node->create_service<GetPartialPointCloudMap>(
    "service/get_partial_pcd_map", std::bind(
                                     &PartialMapLoaderModule::onServiceGetPartialPointCloudMap,
                                     this, std::placeholders::_1, std::placeholders::_2));
}

void PartialMapLoaderModule::partialAreaLoad(
  const autoware_map_msgs::msg::AreaInfo area,
  GetPartialPointCloudMap::Response::SharedPtr & response) const
{
  // iterate over all the available pcd map grids

  for (const auto & ele : all_pcd_file_metadata_dict_) {
    std::string path = ele.first;
    PCDFileMetadata metadata = ele.second;

    // assume that the map ID = map path (for now)
    std::string map_id = path;

    // skip if the pcd file is not within the queried area
    if (!isGridWithinQueriedArea(area, metadata)) continue;

    autoware_map_msgs::msg::PointCloudMapCellWithID pointcloud_map_cell_with_id =
      loadPointCloudMapCellWithID(path, map_id);
    response->new_pointcloud_with_ids.push_back(pointcloud_map_cell_with_id);
  }
}

bool PartialMapLoaderModule::onServiceGetPartialPointCloudMap(
  GetPartialPointCloudMap::Request::SharedPtr req, GetPartialPointCloudMap::Response::SharedPtr res)
{
  auto area = req->area;
  partialAreaLoad(area, res);
  res->header.frame_id = "map";
  return true;
}

autoware_map_msgs::msg::PointCloudMapCellWithID PartialMapLoaderModule::loadPointCloudMapCellWithID(
  const std::string path, const std::string map_id) const
{
  sensor_msgs::msg::PointCloud2 pcd;
  if (pcl::io::loadPCDFile(path, pcd) == -1) {
    RCLCPP_ERROR_STREAM(logger_, "PCD load failed: " << path);
  }
  autoware_map_msgs::msg::PointCloudMapCellWithID pointcloud_map_cell_with_id;
  pointcloud_map_cell_with_id.pointcloud = pcd;
  pointcloud_map_cell_with_id.cell_id = map_id;
  return pointcloud_map_cell_with_id;
}
