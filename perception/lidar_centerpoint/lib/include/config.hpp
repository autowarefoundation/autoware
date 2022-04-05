// Copyright 2021 TIER IV, Inc.
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

#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <cstddef>

namespace centerpoint
{
class Config
{
public:
  // input params
  constexpr static std::size_t point_dim_size = 3;      // x, y and z
  constexpr static std::size_t point_feature_size = 4;  // x, y, z and timelag
  constexpr static std::size_t box_feature_size = 9;    // x, y, z, l, w, h, rot, vel_x, vel_y
  constexpr static std::size_t max_num_points_per_voxel = 32;
  constexpr static std::size_t max_num_voxels = 40000;
  constexpr static float range_min_x = -89.6f;
  constexpr static float range_min_y = -89.6f;
  constexpr static float range_min_z = -3.0f;
  constexpr static float range_max_x = 89.6f;
  constexpr static float range_max_y = 89.6f;
  constexpr static float range_max_z = 5.0f;
  constexpr static float voxel_size_x = 0.32f;
  constexpr static float voxel_size_y = 0.32f;
  constexpr static float voxel_size_z = 8.0f;

  // network params
  constexpr static std::size_t batch_size = 1;
  constexpr static std::size_t downsample_factor = 2;
  constexpr static std::size_t encoder_in_feature_size = 9;
  constexpr static std::size_t encoder_out_feature_size = 32;
  constexpr static std::size_t head_out_size = 6;
  constexpr static std::size_t head_out_offset_size = 2;
  constexpr static std::size_t head_out_z_size = 1;
  constexpr static std::size_t head_out_dim_size = 3;
  constexpr static std::size_t head_out_rot_size = 2;
  constexpr static std::size_t head_out_vel_size = 2;

  // calculated params
  constexpr static std::size_t grid_size_x = (range_max_x - range_min_x) / voxel_size_x;
  constexpr static std::size_t grid_size_y = (range_max_y - range_min_y) / voxel_size_y;
  constexpr static std::size_t grid_size_z = (range_max_z - range_min_z) / voxel_size_z;
  constexpr static float offset_x = range_min_x + voxel_size_x / 2;
  constexpr static float offset_y = range_min_y + voxel_size_y / 2;
  constexpr static float offset_z = range_min_z + voxel_size_z / 2;
  constexpr static std::size_t down_grid_size_x = grid_size_x / downsample_factor;
  constexpr static std::size_t down_grid_size_y = grid_size_y / downsample_factor;
};

}  // namespace centerpoint

#endif  // CONFIG_HPP_
