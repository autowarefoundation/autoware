// Copyright 2024 Autoware Foundation
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

#ifndef STUB_PCD_LOADER_HPP_
#define STUB_PCD_LOADER_HPP_

#include "test_util.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_map_msgs/srv/get_differential_point_cloud_map.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <limits>

class StubPcdLoader : public rclcpp::Node
{
  using GetDifferentialPointCloudMap = autoware_map_msgs::srv::GetDifferentialPointCloudMap;

public:
  StubPcdLoader() : Node("stub_pcd_loader")
  {
    get_differential_pcd_maps_service_ = create_service<GetDifferentialPointCloudMap>(
      "pcd_loader_service", std::bind(
                              &StubPcdLoader::on_service_get_differential_point_cloud_map, this,
                              std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::Service<GetDifferentialPointCloudMap>::SharedPtr get_differential_pcd_maps_service_;

  // NOLINTNEXTLINE
  bool on_service_get_differential_point_cloud_map(
    GetDifferentialPointCloudMap::Request::SharedPtr req,
    GetDifferentialPointCloudMap::Response::SharedPtr res)
  {
    const float offset_x = 100.0f;
    const float offset_y = 100.0f;

    // If the requested area is outside of the offset, return an empty response.
    if (
      req->area.center_x - req->area.radius > offset_x ||
      req->area.center_x + req->area.radius < offset_x ||
      req->area.center_y - req->area.radius > offset_y ||
      req->area.center_y + req->area.radius < offset_y) {
      res->header.frame_id = "map";
      return true;
    }

    autoware_map_msgs::msg::PointCloudMapCellWithID pcd_map_cell_with_id;
    pcd_map_cell_with_id.cell_id = "0";
    pcl::PointCloud<pcl::PointXYZ> cloud = make_sample_half_cubic_pcd();
    for (auto & point : cloud.points) {
      point.x += offset_x;
      point.y += offset_y;
    }
    pcd_map_cell_with_id.metadata.min_x = std::numeric_limits<float>::max();
    pcd_map_cell_with_id.metadata.min_y = std::numeric_limits<float>::max();
    pcd_map_cell_with_id.metadata.max_x = std::numeric_limits<float>::lowest();
    pcd_map_cell_with_id.metadata.max_y = std::numeric_limits<float>::lowest();
    for (const auto & point : cloud.points) {
      pcd_map_cell_with_id.metadata.min_x = std::min(pcd_map_cell_with_id.metadata.min_x, point.x);
      pcd_map_cell_with_id.metadata.min_y = std::min(pcd_map_cell_with_id.metadata.min_y, point.y);
      pcd_map_cell_with_id.metadata.max_x = std::max(pcd_map_cell_with_id.metadata.max_x, point.x);
      pcd_map_cell_with_id.metadata.max_y = std::max(pcd_map_cell_with_id.metadata.max_y, point.y);
    }
    RCLCPP_INFO_STREAM(get_logger(), "cloud size: " << cloud.size());
    pcl::toROSMsg(cloud, pcd_map_cell_with_id.pointcloud);
    res->new_pointcloud_with_ids.push_back(pcd_map_cell_with_id);
    res->header.frame_id = "map";
    return true;
  }
};

#endif  // STUB_PCD_LOADER_HPP_
