// Copyright 2022 AutoCore Ltd., TIER IV, Inc.
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

#include "lidar_centerpoint_tvm/network/scatter.hpp"

#include <lidar_centerpoint_tvm/utils.hpp>

#include <thread>

namespace
{
const std::size_t THREAD_NUM_SCATTER = 32;
}  // namespace

namespace autoware
{
namespace perception
{
namespace lidar_centerpoint_tvm
{

void scatterFeatures_worker(
  const std::vector<float> & pillar_features, const std::vector<int32_t> & coords,
  const std::size_t num_pillars, const CenterPointConfig & config,
  std::vector<float> & scattered_features, std::size_t thread_idx, std::size_t pillars_per_thread)
{
  // pillar_features: shape of max_num_pillars * encoder_out_feature_size
  // coords: shape of max_num_pillars * 3
  // scattered_features: shape of encoder_out_feature_size * grid_size_y * grid_size_x

  for (std::size_t idx = 0; idx < pillars_per_thread; idx++) {
    std::size_t pillar_idx = thread_idx * pillars_per_thread + idx;
    if (pillar_idx >= num_pillars) {
      return;
    }

    // 3D position(z,y,x) of the voxel/pillar
    int32_t coord[3] = {
      coords[pillar_idx * 3], coords[pillar_idx * 3 + 1], coords[pillar_idx * 3 + 2]};
    if (coord[0] < 0) {
      continue;
    }  // if coord.z < 0 return

    for (std::size_t inner_idx = 0; inner_idx < config.encoder_out_feature_size_; inner_idx++) {
      std::size_t feature_idx = config.encoder_out_feature_size_ * pillar_idx + inner_idx;
      const auto feature = pillar_features[feature_idx];
      int32_t scatter_idx = config.grid_size_y_ * config.grid_size_x_ * inner_idx +
                            config.grid_size_x_ * coord[1] + coord[2];
      scattered_features[scatter_idx] = feature;
    }
  }
}

void scatterFeatures(
  const std::vector<float> & pillar_features, const std::vector<int32_t> & coords,
  const std::size_t num_pillars, const CenterPointConfig & config,
  std::vector<float> & scattered_features)
{
  std::vector<std::thread> threadPool;
  std::size_t pillars_per_thread = divup(config.max_voxel_size_, THREAD_NUM_SCATTER);
  for (std::size_t idx = 0; idx < THREAD_NUM_SCATTER; idx++) {
    std::thread worker(
      scatterFeatures_worker, std::ref(pillar_features), std::ref(coords), num_pillars,
      std::ref(config), std::ref(scattered_features), idx, pillars_per_thread);
    threadPool.push_back(std::move(worker));
  }
  for (std::size_t idx = 0; idx < THREAD_NUM_SCATTER; idx++) {
    threadPool[idx].join();
  }
}

}  // namespace lidar_centerpoint_tvm
}  // namespace perception
}  // namespace autoware
