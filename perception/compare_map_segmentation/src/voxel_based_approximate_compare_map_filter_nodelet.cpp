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

#include "compare_map_segmentation/voxel_based_approximate_compare_map_filter_nodelet.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <vector>

namespace compare_map_segmentation
{

bool VoxelBasedApproximateStaticMapLoader::is_close_to_map(
  const pcl::PointXYZ & point, [[maybe_unused]] const double distance_threshold)
{
  if (voxel_map_ptr_ == NULL) {
    return false;
  }
  const int index =
    voxel_grid_.getCentroidIndexAt(voxel_grid_.getGridCoordinates(point.x, point.y, point.z));
  if (index == -1) {
    return false;
  } else {
    return true;
  }
}

bool VoxelBasedApproximateDynamicMapLoader::is_close_to_map(
  const pcl::PointXYZ & point, [[maybe_unused]] const double distance_threshold)
{
  if (current_voxel_grid_dict_.size() == 0) {
    return false;
  }

  const int map_grid_index = static_cast<int>(
    std::floor((point.x - origin_x_) / map_grid_size_x_) +
    map_grids_x_ * std::floor((point.y - origin_y_) / map_grid_size_y_));

  if (static_cast<size_t>(map_grid_index) >= current_voxel_grid_array_.size()) {
    return false;
  }
  if (current_voxel_grid_array_.at(map_grid_index) != NULL) {
    const int index = current_voxel_grid_array_.at(map_grid_index)
                        ->map_cell_voxel_grid.getCentroidIndexAt(
                          current_voxel_grid_array_.at(map_grid_index)
                            ->map_cell_voxel_grid.getGridCoordinates(point.x, point.y, point.z));
    if (index == -1) {
      return false;
    } else {
      return true;
    }
  }
  return false;
}

VoxelBasedApproximateCompareMapFilterComponent::VoxelBasedApproximateCompareMapFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("VoxelBasedApproximateCompareMapFilter", options)
{
  // initialize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ =
      std::make_unique<DebugPublisher>(this, "voxel_based_approximate_compare_map_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  distance_threshold_ = declare_parameter<double>("distance_threshold");
  bool use_dynamic_map_loading = declare_parameter<bool>("use_dynamic_map_loading");
  double downsize_ratio_z_axis = declare_parameter<bool>("downsize_ratio_z_axis");
  if (downsize_ratio_z_axis <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "downsize_ratio_z_axis should be positive");
    return;
  }
  if (use_dynamic_map_loading) {
    rclcpp::CallbackGroup::SharedPtr main_callback_group;
    main_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    voxel_based_approximate_map_loader_ = std::make_unique<VoxelBasedApproximateDynamicMapLoader>(
      this, distance_threshold_, downsize_ratio_z_axis, &tf_input_frame_, &mutex_,
      main_callback_group);
  } else {
    voxel_based_approximate_map_loader_ = std::make_unique<VoxelBasedApproximateStaticMapLoader>(
      this, distance_threshold_, downsize_ratio_z_axis, &tf_input_frame_, &mutex_);
  }
}

void VoxelBasedApproximateCompareMapFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_output->points.reserve(pcl_input->points.size());
  for (size_t i = 0; i < pcl_input->points.size(); ++i) {
    if (voxel_based_approximate_map_loader_->is_close_to_map(
          pcl_input->points.at(i), distance_threshold_)) {
      continue;
    }
    pcl_output->points.push_back(pcl_input->points.at(i));
  }
  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

}  // namespace compare_map_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  compare_map_segmentation::VoxelBasedApproximateCompareMapFilterComponent)
