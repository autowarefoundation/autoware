// Copyright 2024 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/outlier_filter/voxel_grid_outlier_filter_node.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <vector>

namespace autoware::pointcloud_preprocessor
{
VoxelGridOutlierFilterComponent::VoxelGridOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("VoxelGridOutlierFilter", options)
{
  // set initial parameters
  {
    voxel_size_x_ = declare_parameter<double>("voxel_size_x");
    voxel_size_y_ = declare_parameter<double>("voxel_size_y");
    voxel_size_z_ = declare_parameter<double>("voxel_size_z");
    voxel_points_threshold_ = declare_parameter<int>("voxel_points_threshold");
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&VoxelGridOutlierFilterComponent::paramCallback, this, _1));
}
void VoxelGridOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_voxelized_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_voxelized_input->points.reserve(pcl_input->points.size());
  voxel_filter.setInputCloud(pcl_input);
  voxel_filter.setSaveLeafLayout(true);
  voxel_filter.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
  voxel_filter.setMinimumPointsNumberPerVoxel(voxel_points_threshold_);
  voxel_filter.filter(*pcl_voxelized_input);

  pcl_output->points.reserve(pcl_input->points.size());
  for (size_t i = 0; i < pcl_input->points.size(); ++i) {
    const int index = voxel_filter.getCentroidIndexAt(voxel_filter.getGridCoordinates(
      pcl_input->points.at(i).x, pcl_input->points.at(i).y, pcl_input->points.at(i).z));
    if (index != -1) {  // not empty voxel
      pcl_output->points.push_back(pcl_input->points.at(i));
    }
  }

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

rcl_interfaces::msg::SetParametersResult VoxelGridOutlierFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "voxel_size_x", voxel_size_x_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %f.", voxel_size_x_);
  }
  if (get_param(p, "voxel_size_y", voxel_size_y_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %f.", voxel_size_y_);
  }
  if (get_param(p, "voxel_size_z", voxel_size_z_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %f.", voxel_size_z_);
  }
  if (get_param(p, "voxel_points_threshold", voxel_points_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %d.", voxel_points_threshold_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::VoxelGridOutlierFilterComponent)
