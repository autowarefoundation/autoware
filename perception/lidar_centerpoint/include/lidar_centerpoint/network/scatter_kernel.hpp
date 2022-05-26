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

#ifndef LIDAR_CENTERPOINT__NETWORK__SCATTER_KERNEL_HPP_
#define LIDAR_CENTERPOINT__NETWORK__SCATTER_KERNEL_HPP_

#include <cuda.h>
#include <cuda_runtime_api.h>

namespace centerpoint
{
cudaError_t scatterFeatures_launch(
  const float * pillar_features, const int * coords, const std::size_t num_pillars,
  const std::size_t max_voxel_size, const std::size_t encoder_out_feature_size,
  const std::size_t grid_size_x, const std::size_t grid_size_y, float * scattered_features,
  cudaStream_t stream);

}  // namespace centerpoint

#endif  // LIDAR_CENTERPOINT__NETWORK__SCATTER_KERNEL_HPP_
