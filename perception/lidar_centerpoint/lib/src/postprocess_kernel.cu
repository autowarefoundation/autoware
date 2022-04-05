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

#include <circle_nms_kernel.hpp>
#include <postprocess_kernel.hpp>

#include <thrust/count.h>
#include <thrust/sort.h>

namespace
{
const std::size_t THREADS_PER_BLOCK = 32;
}  // namespace

namespace centerpoint
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

__device__ inline float sigmoid(float x) { return 1.0f / expf(-x); }

__global__ void generateBoxes3D_kernel(
  const float * out_heatmap, const float * out_offset, const float * out_z, const float * out_dim,
  const float * out_rot, const float * out_vel, const float voxel_size_x, const float voxel_size_y,
  const float range_min_x, const float range_min_y, const std::size_t down_grid_size_x,
  const std::size_t down_grid_size_y, const std::size_t downsample_factor, const int num_class,
  Box3D * det_boxes3d)
{
  // generate boxes3d from the outputs of the network.
  // shape of out_*: (N, DOWN_GRID_SIZE_Y, DOWN_GRID_SIZE_X)
  // heatmap: N = num_class, offset: N = 2, z: N = 1, dim: N = 3, rot: N = 2, vel: N = 2
  const auto yi = blockIdx.x * THREADS_PER_BLOCK + threadIdx.x;
  const auto xi = blockIdx.y * THREADS_PER_BLOCK + threadIdx.y;
  const auto idx = down_grid_size_x * yi + xi;
  const auto down_grid_size = down_grid_size_y * down_grid_size_x;

  if (yi >= down_grid_size_y || xi >= down_grid_size_x) {
    return;
  }

  int label = -1;
  float max_score = -1;
  for (int ci = 0; ci < num_class; ci++) {
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
  const float vel_x = out_vel[down_grid_size * 0 + idx];
  const float vel_y = out_vel[down_grid_size * 1 + idx];

  det_boxes3d[idx].label = label;
  det_boxes3d[idx].score = max_score;
  det_boxes3d[idx].x = x;
  det_boxes3d[idx].y = y;
  det_boxes3d[idx].z = z;
  det_boxes3d[idx].length = expf(l);
  det_boxes3d[idx].width = expf(w);
  det_boxes3d[idx].height = expf(h);
  det_boxes3d[idx].yaw = atan2f(yaw_sin, yaw_cos);
  det_boxes3d[idx].vel_x = vel_x;
  det_boxes3d[idx].vel_y = vel_y;
}

PostProcessCUDA::PostProcessCUDA(const std::size_t num_class, const float score_threshold)
: num_class_(num_class), score_threshold_(score_threshold)
{
  const auto num_raw_boxes3d = Config::down_grid_size_y * Config::down_grid_size_x;
  boxes3d_d_ = thrust::device_vector<Box3D>(num_raw_boxes3d);
}

cudaError_t PostProcessCUDA::generateDetectedBoxes3D_launch(
  const float * out_heatmap, const float * out_offset, const float * out_z, const float * out_dim,
  const float * out_rot, const float * out_vel, std::vector<Box3D> & det_boxes3d,
  cudaStream_t stream)
{
  dim3 blocks(
    divup(Config::down_grid_size_y, THREADS_PER_BLOCK),
    divup(Config::down_grid_size_x, THREADS_PER_BLOCK));
  dim3 threads(THREADS_PER_BLOCK, THREADS_PER_BLOCK);
  generateBoxes3D_kernel<<<blocks, threads, 0, stream>>>(
    out_heatmap, out_offset, out_z, out_dim, out_rot, out_vel, Config::voxel_size_x,
    Config::voxel_size_y, Config::range_min_x, Config::range_min_y, Config::down_grid_size_x,
    Config::down_grid_size_y, Config::downsample_factor, num_class_,
    thrust::raw_pointer_cast(boxes3d_d_.data()));

  // suppress by socre
  const auto num_det_boxes3d = thrust::count_if(
    thrust::device, boxes3d_d_.begin(), boxes3d_d_.end(), is_score_greater(score_threshold_));
  if (num_det_boxes3d == 0) {
    return cudaGetLastError();
  }
  thrust::device_vector<Box3D> det_boxes3d_d(num_det_boxes3d);
  thrust::copy_if(
    thrust::device, boxes3d_d_.begin(), boxes3d_d_.end(), det_boxes3d_d.begin(),
    is_score_greater(score_threshold_));

  // sort by score
  thrust::sort(det_boxes3d_d.begin(), det_boxes3d_d.end(), score_greater());

  // supress by NMS
  thrust::device_vector<bool> final_keep_mask_d(num_det_boxes3d);
  const auto num_final_det_boxes3d =
    circleNMS(det_boxes3d_d, dist_threshold_, final_keep_mask_d, stream);

  thrust::device_vector<Box3D> final_det_boxes3d_d(num_final_det_boxes3d);
  thrust::copy_if(
    thrust::device, det_boxes3d_d.begin(), det_boxes3d_d.end(), final_keep_mask_d.begin(),
    final_det_boxes3d_d.begin(), is_kept());

  // memcpy device to host
  det_boxes3d.resize(num_final_det_boxes3d);
  thrust::copy(final_det_boxes3d_d.begin(), final_det_boxes3d_d.end(), det_boxes3d.begin());

  return cudaGetLastError();
}

}  // namespace centerpoint
