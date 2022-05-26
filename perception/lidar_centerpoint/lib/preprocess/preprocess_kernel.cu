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

#include "lidar_centerpoint/preprocess/preprocess_kernel.hpp"

#include <lidar_centerpoint/utils.hpp>

namespace
{
const std::size_t MAX_POINT_IN_VOXEL_SIZE = 32;  // the same as max_point_in_voxel_size_ in config
const std::size_t WARPS_PER_BLOCK = 4;
const std::size_t ENCODER_IN_FEATURE_SIZE = 9;  // the same as encoder_in_feature_size_ in config
}  // namespace

namespace centerpoint
{
__global__ void generateFeatures_kernel(
  const float * voxel_features, const float * voxel_num_points, const int * coords,
  const std::size_t num_voxels, const float voxel_x, const float voxel_y, const float voxel_z,
  const float range_min_x, const float range_min_y, const float range_min_z, float * features)
{
  // voxel_features (float): (max_voxel_size, max_point_in_voxel_size, point_feature_size)
  // voxel_num_points (int): (max_voxel_size)
  // coords (int): (max_voxel_size, point_dim_size)
  int pillar_idx = blockIdx.x * WARPS_PER_BLOCK + threadIdx.x / MAX_POINT_IN_VOXEL_SIZE;
  int point_idx = threadIdx.x % MAX_POINT_IN_VOXEL_SIZE;
  int pillar_idx_inBlock = threadIdx.x / MAX_POINT_IN_VOXEL_SIZE;  // max_point_in_voxel_size

  if (pillar_idx >= num_voxels) return;

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

cudaError_t generateFeatures_launch(
  const float * voxel_features, const float * voxel_num_points, const int * coords,
  const std::size_t num_voxels, const std::size_t max_voxel_size, const float voxel_size_x,
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

}  // namespace centerpoint
