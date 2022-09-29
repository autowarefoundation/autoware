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

#ifndef LIDAR_CENTERPOINT__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
#define LIDAR_CENTERPOINT__POSTPROCESS__POSTPROCESS_KERNEL_HPP_

#include <lidar_centerpoint/centerpoint_config.hpp>
#include <lidar_centerpoint/utils.hpp>

#include <cuda.h>
#include <cuda_runtime_api.h>
#include <thrust/device_vector.h>

#include <vector>

namespace centerpoint
{
class PostProcessCUDA
{
public:
  explicit PostProcessCUDA(const CenterPointConfig & config);

  cudaError_t generateDetectedBoxes3D_launch(
    const float * out_heatmap, const float * out_offset, const float * out_z, const float * out_dim,
    const float * out_rot, const float * out_vel, std::vector<Box3D> & det_boxes3d,
    cudaStream_t stream);

private:
  CenterPointConfig config_;
  thrust::device_vector<Box3D> boxes3d_d_;
  thrust::device_vector<float> yaw_norm_thresholds_d_;
};

}  // namespace centerpoint

#endif  // LIDAR_CENTERPOINT__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
