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

#include "pointcloud_preprocessor/downsample_filter/pickup_based_voxel_grid_downsample_filter.hpp"

#include "robin_hood.h"

namespace
{
/**
 * @brief Hash function for voxel keys.
 * Utilizes prime numbers to calculate a unique hash for each voxel key.
 */
struct VoxelKeyHash
{
  std::size_t operator()(const std::array<int, 3> & k) const
  {
    // Primes based on the following paper: 'Investigating the Use of Primes in Hashing for
    // Volumetric Data'.
    return (k[0] * 73856093 ^ k[1] * 19349663 ^ k[2] * 83492791);
    // In general, the performance of the search may be improved by restricting the hashkey to the
    // following However, the risk of key collisions also increases, so the value must be
    // appropriate. Enable the following code depending on the situation. return ((1 << 16) - 1) &
    // (k[0] * 73856093 ^ k[1] * 19349663 ^ k[2] * 83492791);
  }
};

/**
 * @brief Equality function for voxel keys.
 * Checks if two voxel keys are equal.
 */
struct VoxelKeyEqual
{
  bool operator()(const std::array<int, 3> & a, const std::array<int, 3> & b) const
  {
    return a == b;
  }
};
}  // namespace

namespace pointcloud_preprocessor
{
PickupBasedVoxelGridDownsampleFilterComponent::PickupBasedVoxelGridDownsampleFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("PickupBasedVoxelGridDownsampleFilterComponent", options)
{
  // initialize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // Initialization of voxel sizes from parameters
  voxel_size_x_ = static_cast<float>(declare_parameter("voxel_size_x", 1.0));
  voxel_size_y_ = static_cast<float>(declare_parameter("voxel_size_y", 1.0));
  voxel_size_z_ = static_cast<float>(declare_parameter("voxel_size_z", 1.0));

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PickupBasedVoxelGridDownsampleFilterComponent::paramCallback, this, _1));
}

void PickupBasedVoxelGridDownsampleFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);

  stop_watch_ptr_->toc("processing_time", true);

  using VoxelKey = std::array<int, 3>;
  // std::unordered_map<VoxelKey, size_t, VoxelKeyHash, VoxelKeyEqual> voxel_map;
  robin_hood::unordered_map<VoxelKey, size_t, VoxelKeyHash, VoxelKeyEqual> voxel_map;

  voxel_map.reserve(input->data.size() / input->point_step);

  constexpr float large_num_offset = 100000.0;
  const float inverse_voxel_size_x = 1.0 / voxel_size_x_;
  const float inverse_voxel_size_y = 1.0 / voxel_size_y_;
  const float inverse_voxel_size_z = 1.0 / voxel_size_z_;

  const int x_offset = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  const int y_offset = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  const int z_offset = input->fields[pcl::getFieldIndex(*input, "z")].offset;

  // Process each point in the point cloud
  for (size_t global_offset = 0; global_offset + input->point_step <= input->data.size();
       global_offset += input->point_step) {
    const float & x = *reinterpret_cast<const float *>(&input->data[global_offset + x_offset]);
    const float & y = *reinterpret_cast<const float *>(&input->data[global_offset + y_offset]);
    const float & z = *reinterpret_cast<const float *>(&input->data[global_offset + z_offset]);

    // The reason for adding a large value is that when converting from float to int, values around
    // -1 to 1 are all rounded down to 0. Therefore, to prevent the numbers from becoming negative,
    // a large value is added. It has been tuned to reduce computational costs, and deliberately
    // avoids using round or floor functions.
    VoxelKey key = {
      static_cast<int>((x + large_num_offset) * inverse_voxel_size_x),
      static_cast<int>((y + large_num_offset) * inverse_voxel_size_y),
      static_cast<int>((z + large_num_offset) * inverse_voxel_size_z)};

    voxel_map.emplace(key, global_offset);
  }

  // Populate the output point cloud
  size_t output_global_offset = 0;
  output.data.resize(voxel_map.size() * input->point_step);
  for (const auto & kv : voxel_map) {
    std::memcpy(
      &output.data[output_global_offset + x_offset], &input->data[kv.second + x_offset],
      sizeof(float));
    std::memcpy(
      &output.data[output_global_offset + y_offset], &input->data[kv.second + y_offset],
      sizeof(float));
    std::memcpy(
      &output.data[output_global_offset + z_offset], &input->data[kv.second + z_offset],
      sizeof(float));
    output_global_offset += input->point_step;
  }

  // Set the output point cloud metadata
  output.header.frame_id = input->header.frame_id;
  output.height = 1;
  output.fields = input->fields;
  output.is_bigendian = input->is_bigendian;
  output.point_step = input->point_step;
  output.is_dense = input->is_dense;
  output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  output.row_step = static_cast<uint32_t>(output.data.size() / output.height);

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);

    auto pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - input->header.stamp).nanoseconds()))
        .count();

    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }
}

rcl_interfaces::msg::SetParametersResult
PickupBasedVoxelGridDownsampleFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  // Handling dynamic updates for the voxel sizes
  if (get_param(p, "voxel_size_x", voxel_size_x_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %f.", voxel_size_x_);
  }
  if (get_param(p, "voxel_size_y", voxel_size_y_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %f.", voxel_size_y_);
  }
  if (get_param(p, "voxel_size_z", voxel_size_z_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %f.", voxel_size_z_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  pointcloud_preprocessor::PickupBasedVoxelGridDownsampleFilterComponent)
