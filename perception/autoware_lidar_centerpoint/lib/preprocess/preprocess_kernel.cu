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
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "autoware/lidar_centerpoint/cuda_utils.hpp"
#include "autoware/lidar_centerpoint/preprocess/preprocess_kernel.hpp"
#include "autoware/lidar_centerpoint/utils.hpp"

#include <cassert>
#include <cmath>

namespace
{
const std::size_t MAX_POINT_IN_VOXEL_SIZE = 32;  // the same as max_point_in_voxel_size_ in config
const std::size_t WARPS_PER_BLOCK = 4;
const std::size_t ENCODER_IN_FEATURE_SIZE = 9;  // the same as encoder_in_feature_size_ in config
}  // namespace

namespace autoware::lidar_centerpoint
{

__global__ void generateSweepPoints_kernel(
  const float * input_points, std::size_t points_size, int input_point_step, float time_lag,
  const float * transform_array, int num_features, float * output_points)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= points_size) return;

  const float input_x = input_points[point_idx * input_point_step + 0];
  const float input_y = input_points[point_idx * input_point_step + 1];
  const float input_z = input_points[point_idx * input_point_step + 2];

  // transform_array is expected to be column-major
  output_points[point_idx * num_features + 0] = transform_array[0] * input_x +
                                                transform_array[4] * input_y +
                                                transform_array[8] * input_z + transform_array[12];
  output_points[point_idx * num_features + 1] = transform_array[1] * input_x +
                                                transform_array[5] * input_y +
                                                transform_array[9] * input_z + transform_array[13];
  output_points[point_idx * num_features + 2] = transform_array[2] * input_x +
                                                transform_array[6] * input_y +
                                                transform_array[10] * input_z + transform_array[14];
  output_points[point_idx * num_features + 3] = time_lag;
}

cudaError_t generateSweepPoints_launch(
  const float * input_points, std::size_t points_size, int input_point_step, float time_lag,
  const float * transform_array, int num_features, float * output_points, cudaStream_t stream)
{
  auto transform_d = cuda::make_unique<float[]>(16);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    transform_d.get(), transform_array, 16 * sizeof(float), cudaMemcpyHostToDevice, stream));

  dim3 blocks((points_size + 256 - 1) / 256);
  dim3 threads(256);
  assert(num_features == 4);

  generateSweepPoints_kernel<<<blocks, threads, 0, stream>>>(
    input_points, points_size, input_point_step, time_lag, transform_d.get(), num_features,
    output_points);

  cudaError_t err = cudaGetLastError();
  return err;
}

__global__ void shufflePoints_kernel(
  const float * points, const unsigned int * indices, float * shuffled_points,
  const std::size_t points_size, const std::size_t max_size, const std::size_t offset)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= max_size) return;

  int src_idx = indices[(point_idx + offset) % max_size];
  int dst_idx = point_idx;

  if (dst_idx >= points_size) {
    shuffled_points[4 * dst_idx + 0] = INFINITY;
    shuffled_points[4 * dst_idx + 1] = INFINITY;
    shuffled_points[4 * dst_idx + 2] = INFINITY;
    shuffled_points[4 * dst_idx + 3] = INFINITY;
  } else {
    shuffled_points[4 * dst_idx + 0] = points[4 * src_idx + 0];
    shuffled_points[4 * dst_idx + 1] = points[4 * src_idx + 1];
    shuffled_points[4 * dst_idx + 2] = points[4 * src_idx + 2];
    shuffled_points[4 * dst_idx + 3] = points[4 * src_idx + 3];
  }
}

cudaError_t shufflePoints_launch(
  const float * points, const unsigned int * indices, float * shuffled_points,
  const std::size_t points_size, const std::size_t max_size, const std::size_t offset,
  cudaStream_t stream)
{
  dim3 blocks((max_size + 256 - 1) / 256);
  dim3 threads(256);

  if (blocks.x == 0) {
    return cudaGetLastError();
  }

  shufflePoints_kernel<<<blocks, threads, 0, stream>>>(
    points, indices, shuffled_points, points_size, max_size, offset);
  cudaError_t err = cudaGetLastError();
  return err;
}

__global__ void generateVoxels_random_kernel(
  const float * points, std::size_t points_size, float min_x_range, float max_x_range,
  float min_y_range, float max_y_range, float min_z_range, float max_z_range, float pillar_x_size,
  float pillar_y_size, float pillar_z_size, int grid_y_size, int grid_x_size, unsigned int * mask,
  float * voxels)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= points_size) return;

  float4 point = ((float4 *)points)[point_idx];

  if (
    point.x < min_x_range || point.x >= max_x_range || point.y < min_y_range ||
    point.y >= max_y_range || point.z < min_z_range || point.z >= max_z_range)
    return;

  int voxel_idx = floorf((point.x - min_x_range) / pillar_x_size);
  int voxel_idy = floorf((point.y - min_y_range) / pillar_y_size);
  unsigned int voxel_index = voxel_idy * grid_x_size + voxel_idx;

  unsigned int point_id = atomicAdd(&(mask[voxel_index]), 1);

  if (point_id >= MAX_POINT_IN_VOXEL_SIZE) return;
  float * address = voxels + (voxel_index * MAX_POINT_IN_VOXEL_SIZE + point_id) * 4;
  atomicExch(address + 0, point.x);
  atomicExch(address + 1, point.y);
  atomicExch(address + 2, point.z);
  atomicExch(address + 3, point.w);
}

cudaError_t generateVoxels_random_launch(
  const float * points, std::size_t points_size, float min_x_range, float max_x_range,
  float min_y_range, float max_y_range, float min_z_range, float max_z_range, float pillar_x_size,
  float pillar_y_size, float pillar_z_size, int grid_y_size, int grid_x_size, unsigned int * mask,
  float * voxels, cudaStream_t stream)
{
  dim3 blocks((points_size + 256 - 1) / 256);
  dim3 threads(256);

  if (blocks.x == 0) {
    return cudaGetLastError();
  }

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
    ((float4 *)voxel_features)[outIndex] = ((float4 *)voxels)[inIndex];
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
  const float range_min_x, const float range_min_y, const float range_min_z, float * features)
{
  // voxel_features (float): (max_voxel_size, max_point_in_voxel_size, point_feature_size)
  // voxel_num_points (int): (max_voxel_size)
  // coords (int): (max_voxel_size, point_dim_size)
  int pillar_idx = blockIdx.x * WARPS_PER_BLOCK + threadIdx.x / MAX_POINT_IN_VOXEL_SIZE;
  int point_idx = threadIdx.x % MAX_POINT_IN_VOXEL_SIZE;
  int pillar_idx_inBlock = threadIdx.x / MAX_POINT_IN_VOXEL_SIZE;  // max_point_in_voxel_size

  unsigned int num_pillars = num_voxels[0];
  if (pillar_idx >= num_pillars) return;

  // load src
  __shared__ float4 pillarSM[WARPS_PER_BLOCK][MAX_POINT_IN_VOXEL_SIZE];
  __shared__ float3 pillarSumSM[WARPS_PER_BLOCK];
  __shared__ int3 cordsSM[WARPS_PER_BLOCK];
  __shared__ int pointsNumSM[WARPS_PER_BLOCK];
  __shared__ float pillarOutSM[WARPS_PER_BLOCK][MAX_POINT_IN_VOXEL_SIZE][ENCODER_IN_FEATURE_SIZE];

  if (threadIdx.x < WARPS_PER_BLOCK) {
    pointsNumSM[threadIdx.x] = voxel_num_points[blockIdx.x * WARPS_PER_BLOCK + threadIdx.x];
    cordsSM[threadIdx.x] = ((int3 *)coords)[blockIdx.x * WARPS_PER_BLOCK + threadIdx.x];
    pillarSumSM[threadIdx.x] = {0, 0, 0};
  }

  pillarSM[pillar_idx_inBlock][point_idx] =
    ((float4 *)voxel_features)[pillar_idx * MAX_POINT_IN_VOXEL_SIZE + point_idx];
  __syncthreads();

  // calculate sm in a pillar
  if (point_idx < pointsNumSM[pillar_idx_inBlock]) {
    atomicAdd(&(pillarSumSM[pillar_idx_inBlock].x), pillarSM[pillar_idx_inBlock][point_idx].x);
    atomicAdd(&(pillarSumSM[pillar_idx_inBlock].y), pillarSM[pillar_idx_inBlock][point_idx].y);
    atomicAdd(&(pillarSumSM[pillar_idx_inBlock].z), pillarSM[pillar_idx_inBlock][point_idx].z);
  }
  __syncthreads();

  // feature-mean
  float3 mean;
  float validPoints = pointsNumSM[pillar_idx_inBlock];
  mean.x = pillarSumSM[pillar_idx_inBlock].x / validPoints;
  mean.y = pillarSumSM[pillar_idx_inBlock].y / validPoints;
  mean.z = pillarSumSM[pillar_idx_inBlock].z / validPoints;

  mean.x = pillarSM[pillar_idx_inBlock][point_idx].x - mean.x;
  mean.y = pillarSM[pillar_idx_inBlock][point_idx].y - mean.y;
  mean.z = pillarSM[pillar_idx_inBlock][point_idx].z - mean.z;

  // calculate offset
  float x_offset = voxel_x / 2 + cordsSM[pillar_idx_inBlock].z * voxel_x + range_min_x;
  float y_offset = voxel_y / 2 + cordsSM[pillar_idx_inBlock].y * voxel_y + range_min_y;
  float z_offset = voxel_z / 2 + cordsSM[pillar_idx_inBlock].x * voxel_z + range_min_z;

  // feature-offset
  float3 center;
  center.x = pillarSM[pillar_idx_inBlock][point_idx].x - x_offset;
  center.y = pillarSM[pillar_idx_inBlock][point_idx].y - y_offset;
  center.z = pillarSM[pillar_idx_inBlock][point_idx].z - z_offset;

  // store output
  if (point_idx < pointsNumSM[pillar_idx_inBlock]) {
    pillarOutSM[pillar_idx_inBlock][point_idx][0] = pillarSM[pillar_idx_inBlock][point_idx].x;
    pillarOutSM[pillar_idx_inBlock][point_idx][1] = pillarSM[pillar_idx_inBlock][point_idx].y;
    pillarOutSM[pillar_idx_inBlock][point_idx][2] = pillarSM[pillar_idx_inBlock][point_idx].z;
    pillarOutSM[pillar_idx_inBlock][point_idx][3] = pillarSM[pillar_idx_inBlock][point_idx].w;

    pillarOutSM[pillar_idx_inBlock][point_idx][4] = mean.x;
    pillarOutSM[pillar_idx_inBlock][point_idx][5] = mean.y;
    pillarOutSM[pillar_idx_inBlock][point_idx][6] = mean.z;

    pillarOutSM[pillar_idx_inBlock][point_idx][7] = center.x;
    pillarOutSM[pillar_idx_inBlock][point_idx][8] = center.y;

  } else {
    pillarOutSM[pillar_idx_inBlock][point_idx][0] = 0;
    pillarOutSM[pillar_idx_inBlock][point_idx][1] = 0;
    pillarOutSM[pillar_idx_inBlock][point_idx][2] = 0;
    pillarOutSM[pillar_idx_inBlock][point_idx][3] = 0;

    pillarOutSM[pillar_idx_inBlock][point_idx][4] = 0;
    pillarOutSM[pillar_idx_inBlock][point_idx][5] = 0;
    pillarOutSM[pillar_idx_inBlock][point_idx][6] = 0;

    pillarOutSM[pillar_idx_inBlock][point_idx][7] = 0;
    pillarOutSM[pillar_idx_inBlock][point_idx][8] = 0;
  }

  __syncthreads();

  for (int i = 0; i < ENCODER_IN_FEATURE_SIZE; i++) {
    int outputSMId = pillar_idx_inBlock * MAX_POINT_IN_VOXEL_SIZE * ENCODER_IN_FEATURE_SIZE +
                     i * MAX_POINT_IN_VOXEL_SIZE + point_idx;
    int outputId = pillar_idx * MAX_POINT_IN_VOXEL_SIZE * ENCODER_IN_FEATURE_SIZE +
                   i * MAX_POINT_IN_VOXEL_SIZE + point_idx;
    features[outputId] = ((float *)pillarOutSM)[outputSMId];
  }
}

// cspell: ignore divup
cudaError_t generateFeatures_launch(
  const float * voxel_features, const float * voxel_num_points, const int * coords,
  const unsigned int * num_voxels, const std::size_t max_voxel_size, const float voxel_size_x,
  const float voxel_size_y, const float voxel_size_z, const float range_min_x,
  const float range_min_y, const float range_min_z, float * features, cudaStream_t stream)
{
  dim3 blocks(divup(max_voxel_size, WARPS_PER_BLOCK));
  dim3 threads(WARPS_PER_BLOCK * MAX_POINT_IN_VOXEL_SIZE);
  generateFeatures_kernel<<<blocks, threads, 0, stream>>>(
    voxel_features, voxel_num_points, coords, num_voxels, voxel_size_x, voxel_size_y, voxel_size_z,
    range_min_x, range_min_y, range_min_z, features);

  return cudaGetLastError();
}

}  // namespace autoware::lidar_centerpoint
