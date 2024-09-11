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

#ifndef AUTOWARE__LIDAR_TRANSFUSION__PREPROCESS__PREPROCESS_KERNEL_HPP_
#define AUTOWARE__LIDAR_TRANSFUSION__PREPROCESS__PREPROCESS_KERNEL_HPP_

#include "autoware/lidar_transfusion/cuda_utils.hpp"
#include "autoware/lidar_transfusion/transfusion_config.hpp"
#include "autoware/lidar_transfusion/utils.hpp"

#include <cuda_runtime_api.h>

namespace autoware::lidar_transfusion
{

class PreprocessCuda
{
public:
  PreprocessCuda(const TransfusionConfig & config, cudaStream_t & stream);

  void generateVoxels(
    float * points, unsigned int points_size, unsigned int * pillar_num, float * voxel_features,
    unsigned int * voxel_num, unsigned int * voxel_idxs);

  cudaError_t generateSweepPoints_launch(
    const uint8_t * input_data, std::size_t points_size, int input_point_step, float time_lag,
    const float * transform, float * output_points);

  cudaError_t shufflePoints_launch(
    const float * points, const unsigned int * indices, float * shuffled_points,
    const std::size_t points_size, const std::size_t max_size, const std::size_t offset);

  cudaError_t generateVoxels_random_launch(
    float * points, unsigned int points_size, unsigned int * mask, float * voxels);

  cudaError_t generateBaseFeatures_launch(
    unsigned int * mask, float * voxels, unsigned int * pillar_num, float * voxel_features,
    unsigned int * voxel_num, unsigned int * voxel_idxs);

private:
  TransfusionConfig config_;
  cudaStream_t stream_;
  cuda::unique_ptr<unsigned int[]> mask_{nullptr};
  cuda::unique_ptr<float[]> voxels_{nullptr};
  unsigned int mask_size_;
  unsigned int voxels_size_;
};
}  // namespace autoware::lidar_transfusion

#endif  // AUTOWARE__LIDAR_TRANSFUSION__PREPROCESS__PREPROCESS_KERNEL_HPP_
