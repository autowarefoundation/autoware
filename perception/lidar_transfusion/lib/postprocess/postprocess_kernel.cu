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

#include "lidar_transfusion/postprocess/circle_nms_kernel.hpp"
#include "lidar_transfusion/postprocess/postprocess_kernel.hpp"

#include <thrust/count.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>

namespace lidar_transfusion
{
const size_t THREADS_PER_BLOCK = 256;

struct is_score_greater
{
  is_score_greater(float t) : t_(t) {}

  __device__ bool operator()(const Box3D & b) { return b.score > t_; }

private:
  float t_{0.0};
};

struct is_kept
{
  __device__ bool operator()(const bool keep) { return keep; }
};

struct score_greater
{
  __device__ bool operator()(const Box3D & lb, const Box3D & rb) { return lb.score > rb.score; }
};

__device__ inline float sigmoid(float x)
{
  return 1.0f / (1.0f + expf(-x));
}

__global__ void generateBoxes3D_kernel(
  const float * __restrict__ cls_output, const float * __restrict__ box_output,
  const float * __restrict__ dir_cls_output, const float voxel_size_x, const float voxel_size_y,
  const float min_x_range, const float min_y_range, const int num_proposals, const int num_classes,
  const int num_point_values, const float * __restrict__ yaw_norm_thresholds,
  Box3D * __restrict__ det_boxes3d)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx >= num_proposals) {
    return;
  }

  int class_id = 0;
  float max_score = cls_output[point_idx];

#pragma unroll
  for (int i = 1; i < num_classes; i++) {
    float score = cls_output[i * num_proposals + point_idx];
    if (score > max_score) {
      max_score = score;
      class_id = i;
    }
  }

  // yaw validation
  const float yaw_sin = dir_cls_output[point_idx];
  const float yaw_cos = dir_cls_output[point_idx + num_proposals];
  const float yaw_norm = sqrtf(yaw_sin * yaw_sin + yaw_cos * yaw_cos);

  det_boxes3d[point_idx].label = class_id;
  det_boxes3d[point_idx].score = yaw_norm >= yaw_norm_thresholds[class_id] ? max_score : 0.f;
  det_boxes3d[point_idx].x = box_output[point_idx] * num_point_values * voxel_size_x + min_x_range;
  det_boxes3d[point_idx].y =
    box_output[point_idx + num_proposals] * num_point_values * voxel_size_y + min_y_range;
  det_boxes3d[point_idx].z = box_output[point_idx + 2 * num_proposals];
  det_boxes3d[point_idx].width = expf(box_output[point_idx + 3 * num_proposals]);
  det_boxes3d[point_idx].length = expf(box_output[point_idx + 4 * num_proposals]);
  det_boxes3d[point_idx].height = expf(box_output[point_idx + 5 * num_proposals]);
  det_boxes3d[point_idx].yaw =
    atan2f(dir_cls_output[point_idx], dir_cls_output[point_idx + num_proposals]);
}

PostprocessCuda::PostprocessCuda(const TransfusionConfig & config, cudaStream_t & stream)
: config_(config), stream_(stream)
{
  boxes3d_d_ = thrust::device_vector<Box3D>(config_.num_proposals_);
  yaw_norm_thresholds_d_ = thrust::device_vector<float>(
    config_.yaw_norm_thresholds_.begin(), config_.yaw_norm_thresholds_.end());
}

// cspell: ignore divup
cudaError_t PostprocessCuda::generateDetectedBoxes3D_launch(
  const float * cls_output, const float * box_output, const float * dir_cls_output,
  std::vector<Box3D> & det_boxes3d, cudaStream_t stream)
{
  dim3 threads = {THREADS_PER_BLOCK};
  dim3 blocks = {divup(config_.num_proposals_, threads.x)};

  generateBoxes3D_kernel<<<blocks, threads, 0, stream>>>(
    cls_output, box_output, dir_cls_output, config_.voxel_x_size_, config_.voxel_y_size_,
    config_.min_x_range_, config_.min_y_range_, config_.num_proposals_, config_.num_classes_,
    config_.num_point_values_, thrust::raw_pointer_cast(yaw_norm_thresholds_d_.data()),
    thrust::raw_pointer_cast(boxes3d_d_.data()));

  // suppress by score
  const auto num_det_boxes3d = thrust::count_if(
    thrust::device, boxes3d_d_.begin(), boxes3d_d_.end(),
    is_score_greater(config_.score_threshold_));
  if (num_det_boxes3d == 0) {
    return cudaGetLastError();
  }
  thrust::device_vector<Box3D> det_boxes3d_d(num_det_boxes3d);
  thrust::copy_if(
    thrust::device, boxes3d_d_.begin(), boxes3d_d_.end(), det_boxes3d_d.begin(),
    is_score_greater(config_.score_threshold_));

  // sort by score
  thrust::sort(det_boxes3d_d.begin(), det_boxes3d_d.end(), score_greater());

  // supress by NMS
  thrust::device_vector<bool> final_keep_mask_d(num_det_boxes3d);
  const auto num_final_det_boxes3d =
    circleNMS(det_boxes3d_d, config_.circle_nms_dist_threshold_, final_keep_mask_d, stream);
  thrust::device_vector<Box3D> final_det_boxes3d_d(num_final_det_boxes3d);
  thrust::copy_if(
    thrust::device, det_boxes3d_d.begin(), det_boxes3d_d.end(), final_keep_mask_d.begin(),
    final_det_boxes3d_d.begin(), is_kept());

  // memcpy device to host
  det_boxes3d.resize(num_final_det_boxes3d);
  thrust::copy(final_det_boxes3d_d.begin(), final_det_boxes3d_d.end(), det_boxes3d.begin());
  return cudaGetLastError();
}

}  // namespace lidar_transfusion
