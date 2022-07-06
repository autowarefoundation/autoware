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

#include "image_projection_based_fusion/pointpainting_fusion/pointpainting_trt.hpp"

#include <image_projection_based_fusion/pointpainting_fusion/preprocess_kernel.hpp>
#include <lidar_centerpoint/centerpoint_config.hpp>
#include <lidar_centerpoint/network/scatter_kernel.hpp>
#include <tier4_autoware_utils/math/constants.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace image_projection_based_fusion
{
PointPaintingTRT::PointPaintingTRT(
  const centerpoint::NetworkParam & encoder_param, const centerpoint::NetworkParam & head_param,
  const centerpoint::DensificationParam & densification_param,
  const centerpoint::CenterPointConfig & config)
: centerpoint::CenterPointTRT(encoder_param, head_param, densification_param, config)
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
  num_voxels_ = vg_ptr_pp_->pointsToVoxels(voxels_, coordinates_, num_points_per_voxel_);
  if (num_voxels_ == 0) {
    return false;
  }
  const auto voxels_size =
    num_voxels_ * config_.max_point_in_voxel_size_ * config_.point_feature_size_;
  const auto coordinates_size = num_voxels_ * config_.point_dim_size_;
  // memcpy from host to device (not copy empty voxels)
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    voxels_d_.get(), voxels_.data(), voxels_size * sizeof(float), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    coordinates_d_.get(), coordinates_.data(), coordinates_size * sizeof(int),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    num_points_per_voxel_d_.get(), num_points_per_voxel_.data(), num_voxels_ * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  CHECK_CUDA_ERROR(image_projection_based_fusion::generateFeatures_launch(
    voxels_d_.get(), num_points_per_voxel_d_.get(), coordinates_d_.get(), num_voxels_,
    config_.max_voxel_size_, config_.voxel_size_x_, config_.voxel_size_y_, config_.voxel_size_z_,
    config_.range_min_x_, config_.range_min_y_, config_.range_min_z_, encoder_in_features_d_.get(),
    stream_));

  return true;
}

}  // namespace image_projection_based_fusion
