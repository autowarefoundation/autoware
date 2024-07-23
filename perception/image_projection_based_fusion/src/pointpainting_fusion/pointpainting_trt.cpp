// Copyright 2022 TIER IV, Inc.
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

#include "autoware/image_projection_based_fusion/pointpainting_fusion/pointpainting_trt.hpp"

#include <autoware/image_projection_based_fusion/pointpainting_fusion/preprocess_kernel.hpp>
#include <autoware/lidar_centerpoint/centerpoint_config.hpp>
#include <autoware/lidar_centerpoint/network/scatter_kernel.hpp>
#include <autoware/universe_utils/math/constants.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace autoware::image_projection_based_fusion
{
PointPaintingTRT::PointPaintingTRT(
  const autoware::lidar_centerpoint::NetworkParam & encoder_param,
  const autoware::lidar_centerpoint::NetworkParam & head_param,
  const autoware::lidar_centerpoint::DensificationParam & densification_param,
  const autoware::lidar_centerpoint::CenterPointConfig & config)
: autoware::lidar_centerpoint::CenterPointTRT(
    encoder_param, head_param, densification_param, config)
{
  vg_ptr_pp_ =
    std::make_unique<image_projection_based_fusion::VoxelGenerator>(densification_param, config_);
}

bool PointPaintingTRT::preprocess(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer)
{
  bool is_success = vg_ptr_pp_->enqueuePointCloud(input_pointcloud_msg, tf_buffer);
  if (!is_success) {
    return false;
  }
  const auto count = vg_ptr_pp_->generateSweepPoints(points_);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    points_d_.get(), points_.data(), count * config_.point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(num_voxels_d_.get(), 0, sizeof(unsigned int), stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(voxels_buffer_d_.get(), 0, voxels_buffer_size_, stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(mask_d_.get(), 0, mask_size_, stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    voxels_d_.get(), 0,
    config_.max_voxel_size_ * config_.max_point_in_voxel_size_ * config_.point_feature_size_ *
      sizeof(float),
    stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    coordinates_d_.get(), 0, config_.max_voxel_size_ * config_.point_dim_size_ * sizeof(int),
    stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    num_points_per_voxel_d_.get(), 0, config_.max_voxel_size_ * sizeof(float), stream_));

  CHECK_CUDA_ERROR(image_projection_based_fusion::generateVoxels_random_launch(
    points_d_.get(), count, config_.range_min_x_, config_.range_max_x_, config_.range_min_y_,
    config_.range_max_y_, config_.range_min_z_, config_.range_max_z_, config_.voxel_size_x_,
    config_.voxel_size_y_, config_.voxel_size_z_, config_.grid_size_y_, config_.grid_size_x_,
    mask_d_.get(), voxels_buffer_d_.get(), stream_));

  CHECK_CUDA_ERROR(image_projection_based_fusion::generateBaseFeatures_launch(
    mask_d_.get(), voxels_buffer_d_.get(), config_.grid_size_y_, config_.grid_size_x_,
    config_.max_voxel_size_, num_voxels_d_.get(), voxels_d_.get(), num_points_per_voxel_d_.get(),
    coordinates_d_.get(), stream_));

  CHECK_CUDA_ERROR(image_projection_based_fusion::generateFeatures_launch(
    voxels_d_.get(), num_points_per_voxel_d_.get(), coordinates_d_.get(), num_voxels_d_.get(),
    config_.max_voxel_size_, config_.voxel_size_x_, config_.voxel_size_y_, config_.voxel_size_z_,
    config_.range_min_x_, config_.range_min_y_, config_.range_min_z_, encoder_in_features_d_.get(),
    config_.encoder_in_feature_size_, stream_));

  return true;
}

}  // namespace autoware::image_projection_based_fusion
