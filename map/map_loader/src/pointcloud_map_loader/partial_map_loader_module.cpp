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

#include <utility>

PartialMapLoaderModule::PartialMapLoaderModule(
  rclcpp::Node * node, std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict)
: logger_(node->get_logger()), all_pcd_file_metadata_dict_(std::move(pcd_file_metadata_dict))
{
  get_partial_pcd_maps_service_ = node->create_service<GetPartialPointCloudMap>(
    "service/get_partial_pcd_map",
    std::bind(
      &PartialMapLoaderModule::on_service_get_partial_point_cloud_map, this, std::placeholders::_1,
      std::placeholders::_2));
}

void PartialMapLoaderModule::partial_area_load(
  const autoware_map_msgs::msg::AreaInfo & area,
  const GetPartialPointCloudMap::Response::SharedPtr & response) const
{
  // iterate over all the available pcd map grids
  for (const auto & ele : all_pcd_file_metadata_dict_) {
    std::string path = ele.first;
    PCDFileMetadata metadata = ele.second;

    // assume that the map ID = map path (for now)
    const std::string & map_id = path;

    // skip if the pcd file is not within the queried area
    if (!is_grid_within_queried_area(area, metadata)) continue;

    autoware_map_msgs::msg::PointCloudMapCellWithMetaData pointcloud_map_cell =
      load_point_cloud_map_cell_with_metadata(path, map_id);
    pointcloud_map_cell.metadata.min_x = metadata.min.x;
    pointcloud_map_cell.metadata.min_y = metadata.min.y;
    pointcloud_map_cell.metadata.max_x = metadata.max.x;
    pointcloud_map_cell.metadata.max_y = metadata.max.y;

    response->new_pointcloud_cells.push_back(pointcloud_map_cell);
  }
}

bool PartialMapLoaderModule::on_service_get_partial_point_cloud_map(
  GetPartialPointCloudMap::Request::SharedPtr req,
  GetPartialPointCloudMap::Response::SharedPtr res) const
{
  auto area = req->area;
  partial_area_load(area, res);
  res->header.frame_id = "map";
  return true;
}

autoware_map_msgs::msg::PointCloudMapCellWithMetaData
PartialMapLoaderModule::load_point_cloud_map_cell_with_metadata(
  const std::string & path, const std::string & map_id) const
{
  sensor_msgs::msg::PointCloud2 pcd;
  if (pcl::io::loadPCDFile(path, pcd) == -1) {
    RCLCPP_ERROR_STREAM(logger_, "PCD load failed: " << path);
  }
  autoware_map_msgs::msg::PointCloudMapCellWithMetaData pointcloud_map_cell;
  pointcloud_map_cell.pointcloud = pcd;
  pointcloud_map_cell.metadata.cell_id = map_id;
  return pointcloud_map_cell;
}
