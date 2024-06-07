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
/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES.
 * All rights reserved. SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lidar_transfusion/cuda_utils.hpp"
#include "lidar_transfusion/preprocess/preprocess_kernel.hpp"

namespace lidar_transfusion
{

PreprocessCuda::PreprocessCuda(const TransfusionConfig & config, cudaStream_t & stream)
: stream_(stream), config_(config)
{
  mask_size_ = config_.grid_z_size_ * config_.grid_y_size_ * config_.grid_x_size_;
  voxels_size_ = config_.grid_z_size_ * config_.grid_y_size_ * config_.grid_x_size_ *
                   config_.max_num_points_per_pillar_ * config_.num_point_feature_size_ +
                 1;
  mask_ = cuda::make_unique<unsigned int[]>(mask_size_);
  voxels_ = cuda::make_unique<float[]>(voxels_size_);
}

void PreprocessCuda::generateVoxels(
  float * points, unsigned int points_size, unsigned int * pillar_num, float * voxel_features,
  unsigned int * voxel_num, unsigned int * voxel_idxs)
{
  cuda::clear_async(mask_.get(), mask_size_, stream_);
  cuda::clear_async(voxels_.get(), voxels_size_, stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  CHECK_CUDA_ERROR(generateVoxels_random_launch(points, points_size, mask_.get(), voxels_.get()));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  CHECK_CUDA_ERROR(generateBaseFeatures_launch(
    mask_.get(), voxels_.get(), pillar_num, voxel_features, voxel_num, voxel_idxs));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

__global__ void generateVoxels_random_kernel(
  float * points, unsigned int points_size, float min_x_range, float max_x_range, float min_y_range,
  float max_y_range, float min_z_range, float max_z_range, float pillar_x_size, float pillar_y_size,
  float pillar_z_size, int grid_y_size, int grid_x_size, int points_per_voxel, unsigned int * mask,
  float * voxels)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= points_size) return;

  float x = points[point_idx * 5];
  float y = points[point_idx * 5 + 1];
  float z = points[point_idx * 5 + 2];
  float i = points[point_idx * 5 + 3];
  float t = points[point_idx * 5 + 4];

  if (
    x <= min_x_range || x >= max_x_range || y <= min_y_range || y >= max_y_range ||
    z <= min_z_range || z >= max_z_range)
    return;

  int voxel_idx = floorf((x - min_x_range) / pillar_x_size);
  int voxel_idy = floorf((y - min_y_range) / pillar_y_size);
  unsigned int voxel_index = voxel_idy * grid_x_size + voxel_idx;

  unsigned int point_id = atomicAdd(&(mask[voxel_index]), 1);

  if (point_id >= points_per_voxel) return;
  float * address = voxels + (voxel_index * points_per_voxel + point_id) * 5;
  atomicExch(address + 0, x);
  atomicExch(address + 1, y);
  atomicExch(address + 2, z);
  atomicExch(address + 3, i);
  atomicExch(address + 4, t);
}

cudaError_t PreprocessCuda::generateVoxels_random_launch(
  float * points, unsigned int points_size, unsigned int * mask, float * voxels)
{
  int threadNum = config_.threads_for_voxel_;
  dim3 blocks((points_size + threadNum - 1) / threadNum);
  dim3 threads(threadNum);
  generateVoxels_random_kernel<<<blocks, threads, 0, stream_>>>(
    points, points_size, config_.min_x_range_, config_.max_x_range_, config_.min_y_range_,
    config_.max_y_range_, config_.min_z_range_, config_.max_z_range_, config_.voxel_x_size_,
    config_.voxel_y_size_, config_.voxel_z_size_, config_.grid_y_size_, config_.grid_x_size_,
    config_.points_per_voxel_, mask, voxels);
  cudaError_t err = cudaGetLastError();
  return err;
}

__global__ void generateBaseFeatures_kernel(
  unsigned int * mask, float * voxels, int grid_y_size, int grid_x_size, float points_per_voxel,
  float max_voxels, unsigned int * pillar_num, float * voxel_features, unsigned int * voxel_num,
  unsigned int * voxel_idxs)
{
  unsigned int voxel_idx = blockIdx.x * blockDim.x + threadIdx.x;
  unsigned int voxel_idy = blockIdx.y * blockDim.y + threadIdx.y;

  if (voxel_idx >= grid_x_size || voxel_idy >= grid_y_size) return;

  unsigned int voxel_index = voxel_idy * grid_x_size + voxel_idx;
  unsigned int count = mask[voxel_index];
  if (!(count > 0)) return;
  count = count < points_per_voxel ? count : points_per_voxel;

  unsigned int current_pillarId = 0;
  current_pillarId = atomicAdd(pillar_num, 1);
  if (current_pillarId >= max_voxels) return;

  voxel_num[current_pillarId] = count;

  uint4 idx = {0, 0, voxel_idy, voxel_idx};
  ((uint4 *)voxel_idxs)[current_pillarId] = idx;

  for (int i = 0; i < count; i++) {
    int inIndex = voxel_index * points_per_voxel + i;
    int outIndex = current_pillarId * points_per_voxel + i;
    voxel_features[outIndex * 5] = voxels[inIndex * 5];
    voxel_features[outIndex * 5 + 1] = voxels[inIndex * 5 + 1];
    voxel_features[outIndex * 5 + 2] = voxels[inIndex * 5 + 2];
    voxel_features[outIndex * 5 + 3] = voxels[inIndex * 5 + 3];
    voxel_features[outIndex * 5 + 4] = voxels[inIndex * 5 + 4];
  }

  // clear buffer for next infer
  atomicExch(mask + voxel_index, 0);
}

// create 4 channels
cudaError_t PreprocessCuda::generateBaseFeatures_launch(
  unsigned int * mask, float * voxels, unsigned int * pillar_num, float * voxel_features,
  unsigned int * voxel_num, unsigned int * voxel_idxs)
{
  dim3 threads = {32, 32};
  dim3 blocks = {divup(config_.grid_x_size_, threads.x), divup(config_.grid_y_size_, threads.y)};

  generateBaseFeatures_kernel<<<blocks, threads, 0, stream_>>>(
    mask, voxels, config_.grid_y_size_, config_.grid_x_size_, config_.points_per_voxel_,
    config_.max_voxels_, pillar_num, voxel_features, voxel_num, voxel_idxs);
  cudaError_t err = cudaGetLastError();
  return err;
}

__global__ void generateSweepPoints_kernel(
  const float * input_points, size_t points_size, int input_point_step, float time_lag,
  const float * transform_array, int num_features, float * output_points)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= points_size) return;

  const float input_x = input_points[point_idx * input_point_step + 0];
  const float input_y = input_points[point_idx * input_point_step + 1];
  const float input_z = input_points[point_idx * input_point_step + 2];
  const float intensity = input_points[point_idx * input_point_step + 3];

  output_points[point_idx * num_features] = transform_array[0] * input_x +
                                            transform_array[4] * input_y +
                                            transform_array[8] * input_z + transform_array[12];
  output_points[point_idx * num_features + 1] = transform_array[1] * input_x +
                                                transform_array[5] * input_y +
                                                transform_array[9] * input_z + transform_array[13];
  output_points[point_idx * num_features + 2] = transform_array[2] * input_x +
                                                transform_array[6] * input_y +
                                                transform_array[10] * input_z + transform_array[14];
  output_points[point_idx * num_features + 3] = intensity;
  output_points[point_idx * num_features + 4] = time_lag;
}

cudaError_t PreprocessCuda::generateSweepPoints_launch(
  const float * input_points, size_t points_size, int input_point_step, float time_lag,
  const float * transform_array, float * output_points)
{
  int threadNum = config_.threads_for_voxel_;
  dim3 blocks((points_size + threadNum - 1) / threadNum);
  dim3 threads(threadNum);

  generateSweepPoints_kernel<<<blocks, threads, 0, stream_>>>(
    input_points, points_size, input_point_step, time_lag, transform_array,
    config_.num_point_feature_size_, output_points);

  cudaError_t err = cudaGetLastError();
  return err;
}

}  // namespace lidar_transfusion
