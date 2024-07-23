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

#ifndef TEST_PREPROCESS_KERNEL_HPP_
#define TEST_PREPROCESS_KERNEL_HPP_

#include <autoware/lidar_centerpoint/cuda_utils.hpp>

#include <gtest/gtest.h>

namespace autoware::lidar_centerpoint
{

class PreprocessKernelTest : public testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

  cudaStream_t stream_{};

  std::size_t max_voxel_size_{};
  std::size_t max_point_in_voxel_size_{};
  std::size_t point_feature_size_{};
  std::size_t point_dim_size_{};
  std::size_t config_encoder_in_feature_size_{};
  std::size_t encoder_out_feature_size_{};
  std::size_t capacity_{};

  float range_min_x_{};
  float range_min_y_{};
  float range_min_z_{};
  float range_max_x_{};
  float range_max_y_{};
  float range_max_z_{};
  float voxel_size_x_{};
  float voxel_size_y_{};
  float voxel_size_z_{};

  std::size_t grid_size_x_{};
  std::size_t grid_size_y_{};
  std::size_t grid_size_z_{};

  std::size_t voxels_size_{};
  std::size_t coordinates_size_{};
  std::size_t encoder_in_feature_size_{};
  std::size_t pillar_features_size_{};
  std::size_t spatial_features_size_{};
  std::size_t grid_xy_size_{};

  std::size_t voxels_buffer_size_{};
  std::size_t mask_size_{};

  cuda::unique_ptr<float[]> voxels_d_{};
  cuda::unique_ptr<int[]> coordinates_d_{};
  cuda::unique_ptr<float[]> num_points_per_voxel_d_{};
  cuda::unique_ptr<float[]> encoder_in_features_d_{};
  cuda::unique_ptr<float[]> pillar_features_d_{};
  cuda::unique_ptr<float[]> spatial_features_d_{};
  cuda::unique_ptr<float[]> points_d_{};
  cuda::unique_ptr<float[]> voxels_buffer_d_{};
  cuda::unique_ptr<unsigned int[]> mask_d_{};
  cuda::unique_ptr<unsigned int[]> num_voxels_d_{};
};

}  // namespace autoware::lidar_centerpoint

#endif  // TEST_PREPROCESS_KERNEL_HPP_
