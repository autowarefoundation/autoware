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

#include "pointcloud_map_loader_node.hpp"

#include <glob.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
bool is_pcd_file(const std::string & p)
{
  if (fs::is_directory(p)) {
    return false;
  }

  const std::string ext = fs::path(p).extension();

  return !(ext != ".pcd" && ext != ".PCD");
}
}  // namespace

PointCloudMapLoaderNode::PointCloudMapLoaderNode(const rclcpp::NodeOptions & options)
: Node("pointcloud_map_loader", options)
{
  const auto pcd_paths =
    get_pcd_paths(declare_parameter<std::vector<std::string>>("pcd_paths_or_directory"));
  std::string pcd_metadata_path = declare_parameter<std::string>("pcd_metadata_path");
  bool enable_whole_load = declare_parameter<bool>("enable_whole_load");
  bool enable_downsample_whole_load = declare_parameter<bool>("enable_downsampled_whole_load");
  bool enable_partial_load = declare_parameter<bool>("enable_partial_load");
  bool enable_selected_load = declare_parameter<bool>("enable_selected_load");

  if (enable_whole_load) {
    std::string publisher_name = "output/pointcloud_map";
    pcd_map_loader_ =
      std::make_unique<PointcloudMapLoaderModule>(this, pcd_paths, publisher_name, false);
  }

  if (enable_downsample_whole_load) {
    std::string publisher_name = "output/debug/downsampled_pointcloud_map";
    downsampled_pcd_map_loader_ =
      std::make_unique<PointcloudMapLoaderModule>(this, pcd_paths, publisher_name, true);
  }

  // Parse the metadata file and get the map of (absolute pcd path, pcd file metadata)
  auto pcd_metadata_dict = get_pcd_metadata(pcd_metadata_path, pcd_paths);

  if (enable_partial_load) {
    partial_map_loader_ = std::make_unique<PartialMapLoaderModule>(this, pcd_metadata_dict);
  }

  differential_map_loader_ = std::make_unique<DifferentialMapLoaderModule>(this, pcd_metadata_dict);

  if (enable_selected_load) {
    selected_map_loader_ = std::make_unique<SelectedMapLoaderModule>(this, pcd_metadata_dict);
  }
}

std::map<std::string, PCDFileMetadata> PointCloudMapLoaderNode::get_pcd_metadata(
  const std::string & pcd_metadata_path, const std::vector<std::string> & pcd_paths) const
{
  if (fs::exists(pcd_metadata_path)) {
    std::set<std::string> missing_pcd_names;
    auto pcd_metadata_dict = load_pcd_metadata(pcd_metadata_path);

    pcd_metadata_dict = replace_with_absolute_path(pcd_metadata_dict, pcd_paths, missing_pcd_names);

    // Warning if some segments are missing
    if (!missing_pcd_names.empty()) {
      std::ostringstream oss;

      oss << "The following segment(s) are missing from the input PCDs: ";

      for (const auto & fname : missing_pcd_names) {
        oss << std::endl << fname;
      }

      RCLCPP_ERROR_STREAM(get_logger(), oss.str());
      throw std::runtime_error("Missing PCD segments. Exiting map loader...");
    }

    return pcd_metadata_dict;
  }

  if (pcd_paths.size() == 1) {
    // An exception when using a single file PCD map so that the users do not have to provide
    // a metadata file.
    // Note that this should ideally be avoided and thus eventually be removed by someone, until
    // Autoware users get used to handling the PCD file(s) with metadata.
    RCLCPP_DEBUG_STREAM(get_logger(), "Create PCD metadata, as the pointcloud is a single file.");
    pcl::PointCloud<pcl::PointXYZ> single_pcd;
    const auto & pcd_path = pcd_paths.front();
    if (pcl::io::loadPCDFile(pcd_path, single_pcd) == -1) {
      throw std::runtime_error("PCD load failed: " + pcd_path);
    }
    PCDFileMetadata metadata = {};
    pcl::getMinMax3D(single_pcd, metadata.min, metadata.max);
    return std::map<std::string, PCDFileMetadata>{{pcd_path, metadata}};
  }
  throw std::runtime_error("PCD metadata file not found: " + pcd_metadata_path);
}

std::vector<std::string> PointCloudMapLoaderNode::get_pcd_paths(
  const std::vector<std::string> & pcd_paths_or_directory) const
{
  std::vector<std::string> pcd_paths;
  for (const auto & p : pcd_paths_or_directory) {
    if (!fs::exists(p)) {
      RCLCPP_ERROR_STREAM(get_logger(), "invalid path: " << p);
    }

    if (is_pcd_file(p)) {
      pcd_paths.push_back(p);
    }

    if (fs::is_directory(p)) {
      for (const auto & file : fs::directory_iterator(p)) {
        const auto filename = file.path().string();
        if (is_pcd_file(filename)) {
          pcd_paths.push_back(filename);
        }
      }
    }
  }
  return pcd_paths;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudMapLoaderNode)
