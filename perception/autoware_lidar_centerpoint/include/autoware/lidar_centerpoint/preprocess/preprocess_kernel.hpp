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

#ifndef AUTOWARE__LIDAR_CENTERPOINT__PREPROCESS__PREPROCESS_KERNEL_HPP_
#define AUTOWARE__LIDAR_CENTERPOINT__PREPROCESS__PREPROCESS_KERNEL_HPP_

#include "cuda.h"
#include "cuda_runtime_api.h"

namespace autoware::lidar_centerpoint
{
cudaError_t generateSweepPoints_launch(
  const float * input_points, std::size_t points_size, int input_point_step, float time_lag,
  const float * transform, int num_features, float * output_points, cudaStream_t stream);

cudaError_t shufflePoints_launch(
  const float * points, const unsigned int * indices, float * shuffled_points,
  const std::size_t points_size, const std::size_t max_size, const std::size_t offset,
  cudaStream_t stream);

cudaError_t generateVoxels_random_launch(
  const float * points, std::size_t points_size, float min_x_range, float max_x_range,
  float min_y_range, float max_y_range, float min_z_range, float max_z_range, float pillar_x_size,
  float pillar_y_size, float pillar_z_size, int grid_y_size, int grid_x_size, unsigned int * mask,
  float * voxels, cudaStream_t stream);

cudaError_t generateBaseFeatures_launch(
  unsigned int * mask, float * voxels, int grid_y_size, int grid_x_size, int max_voxel_size,
  unsigned int * pillar_num, float * voxel_features, float * voxel_num, int * voxel_idxs,
  cudaStream_t stream);

cudaError_t generateFeatures_launch(
  const float * voxel_features, const float * voxel_num_points, const int * coords,
  const unsigned int * num_voxels, const std::size_t max_voxel_size, const float voxel_size_x,
  const float voxel_size_y, const float voxel_size_z, const float range_min_x,
  const float range_min_y, const float range_min_z, float * features, cudaStream_t stream);

}  // namespace autoware::lidar_centerpoint

#endif  // AUTOWARE__LIDAR_CENTERPOINT__PREPROCESS__PREPROCESS_KERNEL_HPP_
