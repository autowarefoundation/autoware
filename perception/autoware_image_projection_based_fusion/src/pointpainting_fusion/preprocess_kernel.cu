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
/*
 * SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
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

#include "autoware/image_projection_based_fusion/pointpainting_fusion/preprocess_kernel.hpp"

#include <stdexcept>
// #include <autoware/lidar_centerpoint/utils.hpp>

namespace
{
const std::size_t MAX_POINT_IN_VOXEL_SIZE = 32;  // the same as max_point_in_voxel_size_ in config
const std::size_t WARPS_PER_BLOCK = 4;
const std::size_t ENCODER_IN_FEATURE_SIZE = 12;  // same as encoder_in_feature_size_ in config.hpp
const int POINT_FEATURE_SIZE = 7;

// cspell: ignore divup
std::size_t divup(const std::size_t a, const std::size_t b)
{
  if (a == 0) {
    throw std::runtime_error("A dividend of divup isn't positive.");
  }
  if (b == 0) {
    throw std::runtime_error("A divisor of divup isn't positive.");
  }

  return (a + b - 1) / b;
}

}  // namespace

namespace autoware::image_projection_based_fusion
{
__global__ void generateVoxels_random_kernel(
  const float * points, size_t points_size, float min_x_range, float max_x_range, float min_y_range,
  float max_y_range, float min_z_range, float max_z_range, float pillar_x_size, float pillar_y_size,
  float pillar_z_size, int grid_y_size, int grid_x_size, unsigned int * mask, float * voxels)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= points_size) return;

  float x = points[point_idx * POINT_FEATURE_SIZE];
  float y = points[point_idx * POINT_FEATURE_SIZE + 1];
  float z = points[point_idx * POINT_FEATURE_SIZE + 2];

  if (
    x < min_x_range || x >= max_x_range || y < min_y_range || y >= max_y_range || z < min_z_range ||
    z >= max_z_range)
    return;

  int voxel_idx = floorf((x - min_x_range) / pillar_x_size);
  int voxel_idy = floorf((y - min_y_range) / pillar_y_size);
  unsigned int voxel_index = voxel_idy * grid_x_size + voxel_idx;

  unsigned int point_id = atomicAdd(&(mask[voxel_index]), 1);

  if (point_id >= MAX_POINT_IN_VOXEL_SIZE) return;
  float * address =
    voxels + (voxel_index * MAX_POINT_IN_VOXEL_SIZE + point_id) * POINT_FEATURE_SIZE;
  for (unsigned int i = 0; i < POINT_FEATURE_SIZE; ++i) {
    atomicExch(address + i, points[point_idx * POINT_FEATURE_SIZE + i]);
  }
}

cudaError_t generateVoxels_random_launch(
  const float * points, size_t points_size, float min_x_range, float max_x_range, float min_y_range,
  float max_y_range, float min_z_range, float max_z_range, float pillar_x_size, float pillar_y_size,
  float pillar_z_size, int grid_y_size, int grid_x_size, unsigned int * mask, float * voxels,
  cudaStream_t stream)
{
  dim3 blocks((points_size + 256 - 1) / 256);
  dim3 threads(256);
  generateVoxels_random_kernel<<<blocks, threads, 0, stream>>>(
    points, points_size, min_x_range, max_x_range, min_y_range, max_y_range, min_z_range,
    max_z_range, pillar_x_size, pillar_y_size, pillar_z_size, grid_y_size, grid_x_size, mask,
    voxels);
  cudaError_t err = cudaGetLastError();
  return err;
}

__global__ void generateBaseFeatures_kernel(
  unsigned int * mask, float * voxels, int grid_y_size, int grid_x_size, int max_voxel_size,
  unsigned int * pillar_num, float * voxel_features, float * voxel_num, int * voxel_idxs)
{
  unsigned int voxel_idx = blockIdx.x * blockDim.x + threadIdx.x;
  unsigned int voxel_idy = blockIdx.y * blockDim.y + threadIdx.y;

  if (voxel_idx >= grid_x_size || voxel_idy >= grid_y_size) return;

  unsigned int voxel_index = voxel_idy * grid_x_size + voxel_idx;
  unsigned int count = mask[voxel_index];
  if (!(count > 0)) return;
  count = count < MAX_POINT_IN_VOXEL_SIZE ? count : MAX_POINT_IN_VOXEL_SIZE;

  unsigned int current_pillarId = 0;
  current_pillarId = atomicAdd(pillar_num, 1);
  if (current_pillarId > max_voxel_size - 1) return;

  voxel_num[current_pillarId] = count;

  uint3 idx = {0, voxel_idy, voxel_idx};
  ((uint3 *)voxel_idxs)[current_pillarId] = idx;

  for (int i = 0; i < count; i++) {
    int inIndex = voxel_index * MAX_POINT_IN_VOXEL_SIZE + i;
    int outIndex = current_pillarId * MAX_POINT_IN_VOXEL_SIZE + i;
    for (unsigned int j = 0; j < POINT_FEATURE_SIZE; ++j) {
      voxel_features[outIndex * POINT_FEATURE_SIZE + j] = voxels[inIndex * POINT_FEATURE_SIZE + j];
    }
  }

  // clear buffer for next infer
  atomicExch(mask + voxel_index, 0);
}

// create 4 channels
cudaError_t generateBaseFeatures_launch(
  unsigned int * mask, float * voxels, int grid_y_size, int grid_x_size, int max_voxel_size,
  unsigned int * pillar_num, float * voxel_features, float * voxel_num, int * voxel_idxs,
  cudaStream_t stream)
{
  dim3 threads = {32, 32};
  dim3 blocks = {
    (grid_x_size + threads.x - 1) / threads.x, (grid_y_size + threads.y - 1) / threads.y};

  generateBaseFeatures_kernel<<<blocks, threads, 0, stream>>>(
    mask, voxels, grid_y_size, grid_x_size, max_voxel_size, pillar_num, voxel_features, voxel_num,
    voxel_idxs);
  cudaError_t err = cudaGetLastError();
  return err;
}

__global__ void generateFeatures_kernel(
  const float * voxel_features, const float * voxel_num_points, const int * coords,
  const unsigned int * num_voxels, const float voxel_x, const float voxel_y, const float voxel_z,
  const float range_min_x, const float range_min_y, const float range_min_z, float * features,
  const std::size_t encoder_in_feature_size)
{
  // voxel_features (float): (max_num_voxels, max_num_points_per_voxel, point_feature_size)
  // voxel_num_points (int): (max_num_voxels)
  // coords (int): (max_num_voxels, point_dim_size)
  int pillar_idx = blockIdx.x * WARPS_PER_BLOCK + threadIdx.x / MAX_POINT_IN_VOXEL_SIZE;
  int point_idx = threadIdx.x % MAX_POINT_IN_VOXEL_SIZE;
  int pillar_idx_inBlock = threadIdx.x / MAX_POINT_IN_VOXEL_SIZE;

  unsigned int num_pillars = num_voxels[0];
  if (pillar_idx >= num_pillars) return;

  // load src
  __shared__ float pillarSM[WARPS_PER_BLOCK][MAX_POINT_IN_VOXEL_SIZE][POINT_FEATURE_SIZE];
  __shared__ float3 pillarSumSM[WARPS_PER_BLOCK];
  __shared__ int3 cordsSM[WARPS_PER_BLOCK];
  __shared__ int pointsNumSM[WARPS_PER_BLOCK];
  __shared__ float pillarOutSM[WARPS_PER_BLOCK][MAX_POINT_IN_VOXEL_SIZE][ENCODER_IN_FEATURE_SIZE];

  if (threadIdx.x < WARPS_PER_BLOCK) {
    pointsNumSM[threadIdx.x] = voxel_num_points[blockIdx.x * WARPS_PER_BLOCK + threadIdx.x];
    cordsSM[threadIdx.x] = ((int3 *)coords)[blockIdx.x * WARPS_PER_BLOCK + threadIdx.x];
    pillarSumSM[threadIdx.x] = {0, 0, 0};
  }

  for (std::size_t i = 0; i < POINT_FEATURE_SIZE; ++i) {
    pillarSM[pillar_idx_inBlock][point_idx][i] = voxel_features
      [(POINT_FEATURE_SIZE)*pillar_idx * MAX_POINT_IN_VOXEL_SIZE + (POINT_FEATURE_SIZE)*point_idx +
       i];
  }
  __syncthreads();

  // calculate sm in a pillar
  if (point_idx < pointsNumSM[pillar_idx_inBlock]) {
    atomicAdd(&(pillarSumSM[pillar_idx_inBlock].x), pillarSM[pillar_idx_inBlock][point_idx][0]);
    atomicAdd(&(pillarSumSM[pillar_idx_inBlock].y), pillarSM[pillar_idx_inBlock][point_idx][1]);
    atomicAdd(&(pillarSumSM[pillar_idx_inBlock].z), pillarSM[pillar_idx_inBlock][point_idx][2]);
  }
  __syncthreads();

  // feature-mean
  float3 mean;
  float validPoints = pointsNumSM[pillar_idx_inBlock];
  mean.x = pillarSumSM[pillar_idx_inBlock].x / validPoints;
  mean.y = pillarSumSM[pillar_idx_inBlock].y / validPoints;
  mean.z = pillarSumSM[pillar_idx_inBlock].z / validPoints;

  mean.x = pillarSM[pillar_idx_inBlock][point_idx][0] - mean.x;
  mean.y = pillarSM[pillar_idx_inBlock][point_idx][1] - mean.y;
  mean.z = pillarSM[pillar_idx_inBlock][point_idx][2] - mean.z;

  // calculate offset
  float x_offset = voxel_x / 2 + cordsSM[pillar_idx_inBlock].z * voxel_x + range_min_x;
  float y_offset = voxel_y / 2 + cordsSM[pillar_idx_inBlock].y * voxel_y + range_min_y;
  float z_offset = voxel_z / 2 + cordsSM[pillar_idx_inBlock].x * voxel_z + range_min_z;

  // feature-offset
  float3 center;
  center.x = pillarSM[pillar_idx_inBlock][point_idx][0] - x_offset;
  center.y = pillarSM[pillar_idx_inBlock][point_idx][1] - y_offset;
  center.z = pillarSM[pillar_idx_inBlock][point_idx][2] - z_offset;

  // store output
  if (point_idx < pointsNumSM[pillar_idx_inBlock]) {
    for (std::size_t i = 0; i < POINT_FEATURE_SIZE; ++i) {
      pillarOutSM[pillar_idx_inBlock][point_idx][i] = pillarSM[pillar_idx_inBlock][point_idx][i];
    }

    // change index
    pillarOutSM[pillar_idx_inBlock][point_idx][POINT_FEATURE_SIZE] = mean.x;
    pillarOutSM[pillar_idx_inBlock][point_idx][POINT_FEATURE_SIZE + 1] = mean.y;
    pillarOutSM[pillar_idx_inBlock][point_idx][POINT_FEATURE_SIZE + 2] = mean.z;

    pillarOutSM[pillar_idx_inBlock][point_idx][POINT_FEATURE_SIZE + 3] = center.x;
    pillarOutSM[pillar_idx_inBlock][point_idx][POINT_FEATURE_SIZE + 4] = center.y;

  } else {
    for (std::size_t i = 0; i < encoder_in_feature_size; ++i) {
      pillarOutSM[pillar_idx_inBlock][point_idx][i] = 0;
    }
  }

  __syncthreads();

  for (int i = 0; i < encoder_in_feature_size; i++) {
    int outputSMId = pillar_idx_inBlock * MAX_POINT_IN_VOXEL_SIZE * ENCODER_IN_FEATURE_SIZE +
                     i * MAX_POINT_IN_VOXEL_SIZE + point_idx;
    int outputId = pillar_idx * MAX_POINT_IN_VOXEL_SIZE * encoder_in_feature_size +
                   i * MAX_POINT_IN_VOXEL_SIZE + point_idx;
    features[outputId] = ((float *)pillarOutSM)[outputSMId];
  }
}

cudaError_t generateFeatures_launch(
  const float * voxel_features, const float * voxel_num_points, const int * coords,
  const unsigned int * num_voxels, const std::size_t max_voxel_size, const float voxel_size_x,
  const float voxel_size_y, const float voxel_size_z, const float range_min_x,
  const float range_min_y, const float range_min_z, float * features,
  const std::size_t encoder_in_feature_size, cudaStream_t stream)
{
  dim3 blocks(divup(max_voxel_size, WARPS_PER_BLOCK));
  dim3 threads(WARPS_PER_BLOCK * MAX_POINT_IN_VOXEL_SIZE);
  generateFeatures_kernel<<<blocks, threads, 0, stream>>>(
    voxel_features, voxel_num_points, coords, num_voxels, voxel_size_x, voxel_size_y, voxel_size_z,
    range_min_x, range_min_y, range_min_z, features, encoder_in_feature_size);

  return cudaGetLastError();
}

}  // namespace autoware::image_projection_based_fusion
