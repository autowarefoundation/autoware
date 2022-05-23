// Copyright 2020 Tier IV, Inc.
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

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "map_loader/pointcloud_map_loader_node.hpp"

#include <glob.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
bool isPcdFile(const std::string & p)
{
  if (fs::is_directory(p)) {
    return false;
  }

  const std::string ext = fs::path(p).extension();

  if (ext != ".pcd" && ext != ".PCD") {
    return false;
  }

  return true;
}
}  // namespace

PointCloudMapLoaderNode::PointCloudMapLoaderNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_map_loader", options)
{
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_pointcloud_map_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("output/pointcloud_map", durable_qos);

  const auto pcd_paths_or_directory =
    declare_parameter("pcd_paths_or_directory", std::vector<std::string>({}));

  std::vector<std::string> pcd_paths{};

  for (const auto & p : pcd_paths_or_directory) {
    if (!fs::exists(p)) {
      RCLCPP_ERROR_STREAM(get_logger(), "invalid path: " << p);
    }

    if (isPcdFile(p)) {
      pcd_paths.push_back(p);
    }

    if (fs::is_directory(p)) {
      for (const auto & file : fs::directory_iterator(p)) {
        const auto filename = file.path().string();
        if (isPcdFile(filename)) {
          pcd_paths.push_back(filename);
        }
      }
    }
  }

  const auto pcd = loadPCDFiles(pcd_paths);

  if (pcd.width == 0) {
    RCLCPP_ERROR(get_logger(), "No PCD was loaded: pcd_paths.size() = %zu", pcd_paths.size());
    return;
  }

  pub_pointcloud_map_->publish(pcd);
}

sensor_msgs::msg::PointCloud2 PointCloudMapLoaderNode::loadPCDFiles(
  const std::vector<std::string> & pcd_paths)
{
  sensor_msgs::msg::PointCloud2 whole_pcd{};

  sensor_msgs::msg::PointCloud2 partial_pcd;
  for (const auto & path : pcd_paths) {
    if (pcl::io::loadPCDFile(path, partial_pcd) == -1) {
      RCLCPP_ERROR_STREAM(get_logger(), "PCD load failed: " << path);
    }

    if (whole_pcd.width == 0) {
      whole_pcd = partial_pcd;
    } else {
      whole_pcd.width += partial_pcd.width;
      whole_pcd.row_step += partial_pcd.row_step;
      whole_pcd.data.reserve(whole_pcd.data.size() + partial_pcd.data.size());
      whole_pcd.data.insert(whole_pcd.data.end(), partial_pcd.data.begin(), partial_pcd.data.end());
    }
  }

  whole_pcd.header.frame_id = "map";

  return whole_pcd;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudMapLoaderNode)
