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

#include "compare_map_segmentation/voxel_based_compare_map_filter_nodelet.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <string>
#include <vector>

namespace compare_map_segmentation
{
using pointcloud_preprocessor::get_param;

VoxelBasedCompareMapFilterComponent::VoxelBasedCompareMapFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("VoxelBasedCompareMapFilter", options)
{
  // initialize debug tool
  {
    using autoware_universe_utils::DebugPublisher;
    using autoware_universe_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "voxel_based_compare_map_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  distance_threshold_ = declare_parameter<double>("distance_threshold");
  bool use_dynamic_map_loading = declare_parameter<bool>("use_dynamic_map_loading");
  double downsize_ratio_z_axis = declare_parameter<double>("downsize_ratio_z_axis");
  if (downsize_ratio_z_axis <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "downsize_ratio_z_axis should be positive");
    return;
  }
  set_map_in_voxel_grid_ = false;
  if (use_dynamic_map_loading) {
    rclcpp::CallbackGroup::SharedPtr main_callback_group;
    main_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    voxel_grid_map_loader_ = std::make_unique<VoxelGridDynamicMapLoader>(
      this, distance_threshold_, downsize_ratio_z_axis, &tf_input_frame_, &mutex_,
      main_callback_group);
  } else {
    voxel_grid_map_loader_ = std::make_unique<VoxelGridStaticMapLoader>(
      this, distance_threshold_, downsize_ratio_z_axis, &tf_input_frame_, &mutex_);
  }
  tf_input_frame_ = *(voxel_grid_map_loader_->tf_map_input_frame_);
  RCLCPP_INFO(this->get_logger(), "tf_map_input_frame: %s", tf_input_frame_.c_str());
}

void VoxelBasedCompareMapFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  stop_watch_ptr_->toc("processing_time", true);
  int point_step = input->point_step;
  int offset_x = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  int offset_y = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  int offset_z = input->fields[pcl::getFieldIndex(*input, "z")].offset;

  output.data.resize(input->data.size());
  output.point_step = point_step;
  size_t output_size = 0;
  for (size_t global_offset = 0; global_offset < input->data.size(); global_offset += point_step) {
    pcl::PointXYZ point{};
    std::memcpy(&point.x, &input->data[global_offset + offset_x], sizeof(float));
    std::memcpy(&point.y, &input->data[global_offset + offset_y], sizeof(float));
    std::memcpy(&point.z, &input->data[global_offset + offset_z], sizeof(float));
    if (voxel_grid_map_loader_->is_close_to_map(point, distance_threshold_)) {
      continue;
    }
    std::memcpy(&output.data[output_size], &input->data[global_offset], point_step);
    output_size += point_step;
  }
  output.header = input->header;
  output.fields = input->fields;
  output.data.resize(output_size);
  output.height = input->height;
  output.width = output_size / point_step / output.height;
  output.row_step = output_size / output.height;
  output.is_bigendian = input->is_bigendian;
  output.is_dense = input->is_dense;

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
RCLCPP_COMPONENTS_REGISTER_NODE(compare_map_segmentation::VoxelBasedCompareMapFilterComponent)
