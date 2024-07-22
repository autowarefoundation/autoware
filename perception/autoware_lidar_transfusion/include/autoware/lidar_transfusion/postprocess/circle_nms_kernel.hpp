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

#ifndef AUTOWARE__LIDAR_TRANSFUSION__POSTPROCESS__CIRCLE_NMS_KERNEL_HPP_
#define AUTOWARE__LIDAR_TRANSFUSION__POSTPROCESS__CIRCLE_NMS_KERNEL_HPP_

#include "autoware/lidar_transfusion/utils.hpp"

#include <thrust/device_vector.h>

namespace autoware::lidar_transfusion
{
// Non-maximum suppression (NMS) uses the distance on the xy plane instead of
// intersection over union (IoU) to suppress overlapped objects.
std::size_t circleNMS(
  thrust::device_vector<Box3D> & boxes3d, const float distance_threshold,
  thrust::device_vector<bool> & keep_mask, cudaStream_t stream);

}  // namespace autoware::lidar_transfusion

#endif  // AUTOWARE__LIDAR_TRANSFUSION__POSTPROCESS__CIRCLE_NMS_KERNEL_HPP_
