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

#include "autoware/lidar_centerpoint/postprocess/circle_nms_kernel.hpp"
#include "autoware/lidar_centerpoint/postprocess/postprocess_kernel.hpp"
#include "thrust/count.h"
#include "thrust/device_vector.h"
#include "thrust/sort.h"

namespace
{
const std::size_t THREADS_PER_BLOCK = 32;
}  // namespace

namespace autoware::lidar_centerpoint
{

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
  const float * out_heatmap, const float * out_offset, const float * out_z, const float * out_dim,
  const float * out_rot, const float * out_vel, const float voxel_size_x, const float voxel_size_y,
  const float range_min_x, const float range_min_y, const std::size_t down_grid_size_x,
  const std::size_t down_grid_size_y, const std::size_t downsample_factor, const int class_size,
  const bool has_variance, const float * yaw_norm_thresholds, Box3D * det_boxes3d)
{
  // generate boxes3d from the outputs of the network.
  // shape of out_*: (N, DOWN_GRID_SIZE_Y, DOWN_GRID_SIZE_X)
  // heatmap: N = class_size, offset: N = 2, z: N = 1, dim: N = 3, rot: N = 2, vel: N = 2
  const auto yi = blockIdx.x * THREADS_PER_BLOCK + threadIdx.x;
  const auto xi = blockIdx.y * THREADS_PER_BLOCK + threadIdx.y;
  const auto idx = down_grid_size_x * yi + xi;
  const auto down_grid_size = down_grid_size_y * down_grid_size_x;

  if (yi >= down_grid_size_y || xi >= down_grid_size_x) {
    return;
  }

  int label = -1;
  float max_score = -1;
  for (int ci = 0; ci < class_size; ci++) {
    float score = sigmoid(out_heatmap[down_grid_size * ci + idx]);
    if (score > max_score) {
      label = ci;
      max_score = score;
    }
  }

  const float offset_x = out_offset[down_grid_size * 0 + idx];
  const float offset_y = out_offset[down_grid_size * 1 + idx];
  const float x = voxel_size_x * downsample_factor * (xi + offset_x) + range_min_x;
  const float y = voxel_size_y * downsample_factor * (yi + offset_y) + range_min_y;
  const float z = out_z[idx];
  const float w = out_dim[down_grid_size * 0 + idx];
  const float l = out_dim[down_grid_size * 1 + idx];
  const float h = out_dim[down_grid_size * 2 + idx];
  const float yaw_sin = out_rot[down_grid_size * 0 + idx];
  const float yaw_cos = out_rot[down_grid_size * 1 + idx];
  const float yaw_norm = sqrtf(yaw_sin * yaw_sin + yaw_cos * yaw_cos);
  const float vel_x = out_vel[down_grid_size * 0 + idx];
  const float vel_y = out_vel[down_grid_size * 1 + idx];

  det_boxes3d[idx].label = label;
  det_boxes3d[idx].score = yaw_norm >= yaw_norm_thresholds[label] ? max_score : 0.f;
  det_boxes3d[idx].x = x;
  det_boxes3d[idx].y = y;
  det_boxes3d[idx].z = z;
  det_boxes3d[idx].length = expf(l);
  det_boxes3d[idx].width = expf(w);
  det_boxes3d[idx].height = expf(h);
  det_boxes3d[idx].yaw = atan2f(yaw_sin, yaw_cos);
  det_boxes3d[idx].vel_x = vel_x;
  det_boxes3d[idx].vel_y = vel_y;

  if (has_variance) {
    const float offset_x_variance = out_offset[down_grid_size * 2 + idx];
    const float offset_y_variance = out_offset[down_grid_size * 3 + idx];
    const float z_variance = out_z[down_grid_size * 1 + idx];
    const float w_variance = out_dim[down_grid_size * 3 + idx];
    const float l_variance = out_dim[down_grid_size * 4 + idx];
    const float h_variance = out_dim[down_grid_size * 5 + idx];
    const float yaw_sin_log_variance = out_rot[down_grid_size * 2 + idx];
    const float yaw_cos_log_variance = out_rot[down_grid_size * 3 + idx];
    const float vel_x_variance = out_vel[down_grid_size * 2 + idx];
    const float vel_y_variance = out_vel[down_grid_size * 3 + idx];

    det_boxes3d[idx].x_variance = voxel_size_x * downsample_factor * expf(offset_x_variance);
    det_boxes3d[idx].y_variance = voxel_size_x * downsample_factor * expf(offset_y_variance);
    det_boxes3d[idx].z_variance = expf(z_variance);
    det_boxes3d[idx].length_variance = expf(l_variance);
    det_boxes3d[idx].width_variance = expf(w_variance);
    det_boxes3d[idx].height_variance = expf(h_variance);
    const float yaw_sin_sq = yaw_sin * yaw_sin;
    const float yaw_cos_sq = yaw_cos * yaw_cos;
    const float yaw_norm_sq = (yaw_sin_sq + yaw_cos_sq) * (yaw_sin_sq + yaw_cos_sq);
    det_boxes3d[idx].yaw_variance =
      (yaw_cos_sq * expf(yaw_sin_log_variance) + yaw_sin_sq * expf(yaw_cos_log_variance)) /
      yaw_norm_sq;
    det_boxes3d[idx].vel_x_variance = expf(vel_x_variance);
    det_boxes3d[idx].vel_y_variance = expf(vel_y_variance);
  }
}

PostProcessCUDA::PostProcessCUDA(const CenterPointConfig & config) : config_(config)
{
}

// cspell: ignore divup
cudaError_t PostProcessCUDA::generateDetectedBoxes3D_launch(
  const float * out_heatmap, const float * out_offset, const float * out_z, const float * out_dim,
  const float * out_rot, const float * out_vel, std::vector<Box3D> & det_boxes3d,
  cudaStream_t stream)
{
  dim3 blocks(
    divup(config_.down_grid_size_y_, THREADS_PER_BLOCK),
    divup(config_.down_grid_size_x_, THREADS_PER_BLOCK));
  dim3 threads(THREADS_PER_BLOCK, THREADS_PER_BLOCK);
  auto boxes3d_d =
    thrust::device_vector<Box3D>(config_.down_grid_size_y_ * config_.down_grid_size_x_);
  auto yaw_norm_thresholds_d = thrust::device_vector<float>(
    config_.yaw_norm_thresholds_.begin(), config_.yaw_norm_thresholds_.end());
  generateBoxes3D_kernel<<<blocks, threads, 0, stream>>>(
    out_heatmap, out_offset, out_z, out_dim, out_rot, out_vel, config_.voxel_size_x_,
    config_.voxel_size_y_, config_.range_min_x_, config_.range_min_y_, config_.down_grid_size_x_,
    config_.down_grid_size_y_, config_.downsample_factor_, config_.class_size_,
    config_.has_variance_, thrust::raw_pointer_cast(yaw_norm_thresholds_d.data()),
    thrust::raw_pointer_cast(boxes3d_d.data()));

  // suppress by score
  const auto num_det_boxes3d = thrust::count_if(
    thrust::device, boxes3d_d.begin(), boxes3d_d.end(), is_score_greater(config_.score_threshold_));
  if (num_det_boxes3d == 0) {
    return cudaGetLastError();
  }
  thrust::device_vector<Box3D> det_boxes3d_d(num_det_boxes3d);
  thrust::copy_if(
    thrust::device, boxes3d_d.begin(), boxes3d_d.end(), det_boxes3d_d.begin(),
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

}  // namespace autoware::lidar_centerpoint
