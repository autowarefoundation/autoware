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

#include "selected_map_loader_module.hpp"

#include <utility>
namespace
{
autoware_map_msgs::msg::PointCloudMapMetaData create_metadata(
  const std::map<std::string, PCDFileMetadata> & pcd_file_metadata_dict)
{
  autoware_map_msgs::msg::PointCloudMapMetaData metadata_msg;
  metadata_msg.header.frame_id = "map";
  metadata_msg.header.stamp = rclcpp::Clock().now();

  for (const auto & ele : pcd_file_metadata_dict) {
    PCDFileMetadata metadata = ele.second;

    // assume that the map ID = map path (for now)
    const std::string & map_id = ele.first;

    autoware_map_msgs::msg::PointCloudMapCellMetaData cell_metadata;
    cell_metadata.cell_id = map_id;
    cell_metadata.min_x = metadata.min.x;
    cell_metadata.min_y = metadata.min.y;
    cell_metadata.max_x = metadata.max.x;
    cell_metadata.max_y = metadata.max.y;

    metadata_msg.metadata_list.push_back(cell_metadata);
  }

  return metadata_msg;
}
}  // namespace

SelectedMapLoaderModule::SelectedMapLoaderModule(
  rclcpp::Node * node, std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict)
: logger_(node->get_logger()), all_pcd_file_metadata_dict_(std::move(pcd_file_metadata_dict))
{
  get_selected_pcd_maps_service_ = node->create_service<GetSelectedPointCloudMap>(
    "service/get_selected_pcd_map",
    std::bind(
      &SelectedMapLoaderModule::on_service_get_selected_point_cloud_map, this,
      std::placeholders::_1, std::placeholders::_2));

  // publish the map metadata
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_metadata_ = node->create_publisher<autoware_map_msgs::msg::PointCloudMapMetaData>(
    "output/pointcloud_map_metadata", durable_qos);
  pub_metadata_->publish(create_metadata(all_pcd_file_metadata_dict_));
}

bool SelectedMapLoaderModule::on_service_get_selected_point_cloud_map(
  GetSelectedPointCloudMap::Request::SharedPtr req,
  GetSelectedPointCloudMap::Response::SharedPtr res) const
{
  const auto request_ids = req->cell_ids;
  for (const auto & request_id : request_ids) {
    const auto requested_selected_map_iterator = all_pcd_file_metadata_dict_.find(request_id);

    // skip if the requested ID is not found
    if (requested_selected_map_iterator == all_pcd_file_metadata_dict_.end()) {
      RCLCPP_WARN(logger_, "ID %s not found", request_id.c_str());
      continue;
    }

    const std::string path = requested_selected_map_iterator->first;
    // assume that the map ID = map path (for now)
    const std::string & map_id = path;
    PCDFileMetadata metadata = requested_selected_map_iterator->second;

    autoware_map_msgs::msg::PointCloudMapCellWithMetaData pointcloud_map_cell =
      load_point_cloud_map_cell_with_metadata(path, map_id);
    pointcloud_map_cell.metadata.min_x = metadata.min.x;
    pointcloud_map_cell.metadata.min_y = metadata.min.y;
    pointcloud_map_cell.metadata.max_x = metadata.max.x;
    pointcloud_map_cell.metadata.max_y = metadata.max.y;

    res->new_pointcloud_cells.push_back(pointcloud_map_cell);
  }
  res->header.frame_id = "map";
  return true;
}

autoware_map_msgs::msg::PointCloudMapCellWithMetaData
SelectedMapLoaderModule::load_point_cloud_map_cell_with_metadata(
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
