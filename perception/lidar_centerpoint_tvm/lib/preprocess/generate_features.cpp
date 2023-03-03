// Copyright 2021-2022 AutoCore Ltd., TIER IV, Inc.
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

#include "lidar_centerpoint_tvm/preprocess/generate_features.hpp"

#include <lidar_centerpoint_tvm/utils.hpp>

#include <thread>

namespace
{
const std::size_t THREAD_NUM_VFE = 4;
}  // namespace

namespace autoware
{
namespace perception
{
namespace lidar_centerpoint_tvm
{

void generateFeatures_worker(
  const std::vector<float> & voxel_features, const std::vector<float> & voxel_num_points,
  const std::vector<int32_t> & coords, const std::size_t num_voxels,
  const CenterPointConfig & config, std::vector<float> & features, std::size_t thread_idx,
  std::size_t pillars_per_thread)
{
  for (std::size_t idx = 0; idx < pillars_per_thread; idx++) {
    std::size_t pillar_idx = thread_idx * pillars_per_thread + idx;
    if (pillar_idx >= num_voxels) return;

    // voxel/pillar information
    float points_sum[3] = {0.0, 0.0, 0.0};  // sum of x, y, z in the voxel
    int32_t coordinate[3] = {
      coords[pillar_idx * 3], coords[pillar_idx * 3 + 1],
      coords[pillar_idx * 3 + 2]};                            // 3D position(z,y,x) of the voxel
    std::size_t points_count = voxel_num_points[pillar_idx];  // number of points in the voxel

    for (std::size_t i = 0; i < config.max_point_in_voxel_size_; i++) {
      std::size_t point_idx =
        pillar_idx * config.max_point_in_voxel_size_ * config.point_feature_size_ +
        i * config.point_feature_size_;
      for (std::size_t j = 0; j < config.point_feature_size_; j++) {
        // point (x, y, z, intensity)
        if (i < points_count && j < 3) points_sum[j] += voxel_features[point_idx + j];
      }
    }

    // calculate voxel mean
    float mean[3] = {
      points_sum[0] / points_count, points_sum[1] / points_count, points_sum[2] / points_count};
    // calculate offset
    float x_offset = coordinate[2] * config.voxel_size_x_ + config.offset_x_;
    float y_offset = coordinate[1] * config.voxel_size_y_ + config.offset_y_;
    // float z_offset = coordinate[0] * config.voxel_size_z_ + config.offset_z_;

    // build the encoder_in_features
    for (std::size_t i = 0; i < config.max_point_in_voxel_size_; i++) {
      // feature_idx
      std::size_t feature_idx =
        pillar_idx * config.max_point_in_voxel_size_ * config.encoder_in_feature_size_ +
        i * config.encoder_in_feature_size_;
      std::size_t point_idx =
        pillar_idx * config.max_point_in_voxel_size_ * config.point_feature_size_ +
        i * config.point_feature_size_;
      if (i < points_count) {
        features[feature_idx + 0] = voxel_features[point_idx + 0];
        features[feature_idx + 1] = voxel_features[point_idx + 1];
        features[feature_idx + 2] = voxel_features[point_idx + 2];
        features[feature_idx + 3] = voxel_features[point_idx + 3];
        // feature-mean
        features[feature_idx + 4] = voxel_features[point_idx + 0] - mean[0];
        features[feature_idx + 5] = voxel_features[point_idx + 1] - mean[1];
        features[feature_idx + 6] = voxel_features[point_idx + 2] - mean[2];
        // feature-offset
        features[feature_idx + 7] = voxel_features[point_idx + 0] - x_offset;
        features[feature_idx + 8] = voxel_features[point_idx + 1] - y_offset;

      } else {
        features[feature_idx + 0] = 0;
        features[feature_idx + 1] = 0;
        features[feature_idx + 2] = 0;
        features[feature_idx + 3] = 0;
        // feature-mean
        features[feature_idx + 4] = 0;
        features[feature_idx + 5] = 0;
        features[feature_idx + 6] = 0;
        // feature-offset
        features[feature_idx + 7] = 0;
        features[feature_idx + 8] = 0;
      }
    }
  }
}

// cspell: ignore divup
void generateFeatures(
  const std::vector<float> & voxel_features, const std::vector<float> & voxel_num_points,
  const std::vector<int32_t> & coords, const std::size_t num_voxels,
  const CenterPointConfig & config, std::vector<float> & features)
{
  // voxel_features (float): max_voxel_size*max_point_in_voxel_size*point_feature_size
  // voxel_num_points (int): max_voxel_size
  // coords (int): max_voxel_size*point_dim_size
  std::vector<std::thread> threadPool;
  std::size_t pillars_per_thread = divup(config.max_voxel_size_, THREAD_NUM_VFE);
  for (std::size_t idx = 0; idx < THREAD_NUM_VFE; idx++) {
    std::thread worker(
      generateFeatures_worker, std::ref(voxel_features), std::ref(voxel_num_points),
      std::ref(coords), num_voxels, std::ref(config), std::ref(features), idx, pillars_per_thread);
    threadPool.push_back(std::move(worker));
  }
  for (std::size_t idx = 0; idx < THREAD_NUM_VFE; idx++) {
    threadPool[idx].join();
  }
}

}  // namespace lidar_centerpoint_tvm
}  // namespace perception
}  // namespace autoware
