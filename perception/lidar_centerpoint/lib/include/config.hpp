// Copyright 2021 Tier IV, Inc.
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

namespace centerpoint
{
class Config
{
public:
  // input params
  constexpr static int num_point_dims = 3;      // x, y and z
  constexpr static int num_point_features = 4;  // x, y, z and timelag
  constexpr static int max_num_points_per_voxel = 32;
  constexpr static int max_num_voxels = 40000;
  constexpr static float pointcloud_range_xmin = -89.6f;
  constexpr static float pointcloud_range_ymin = -89.6f;
  constexpr static float pointcloud_range_zmin = -3.0f;
  constexpr static float pointcloud_range_xmax = 89.6f;
  constexpr static float pointcloud_range_ymax = 89.6f;
  constexpr static float pointcloud_range_zmax = 5.0f;
  constexpr static float voxel_size_x = 0.32f;
  constexpr static float voxel_size_y = 0.32f;
  constexpr static float voxel_size_z = 8.0f;
  // = (pointcloud_range_xmax - pointcloud_range_xmin) / voxel_size_x
  constexpr static int grid_size_x = 560;
  // = (pointcloud_range_ymax - pointcloud_range_ymin) / voxel_size_y
  constexpr static int grid_size_y = 560;
  // = (pointcloud_range_zmax - pointcloud_range_zmin) / voxel_size_z
  constexpr static int grid_size_z = 1;
  constexpr static float offset_x = -89.44;  // = pointcloud_range_xmin + voxel_size_x / 2
  constexpr static float offset_y = -89.44;  // = pointcloud_range_ymin + voxel_size_y / 2
  constexpr static float offset_z = 1.0f;    // = pointcloud_range_zmin + voxel_size_z / 2

  // output params
  constexpr static int num_box_features = 11;  // score, class, x, y, z, w, l, h, yaw, vel_x, vel_y
  constexpr static int max_num_output_objects = 500;

  // network params
  constexpr static int batch_size = 1;
  constexpr static int downsample_factor = 2;
  constexpr static int num_encoder_input_features = 8;
  constexpr static int num_encoder_output_features = 32;
  constexpr static int num_output_features = 6;
  constexpr static int num_output_offset_features = 2;
  constexpr static int num_output_z_features = 1;
  constexpr static int num_output_dim_features = 3;
  constexpr static int num_output_rot_features = 2;
  constexpr static int num_output_vel_features = 2;
};

}  // namespace centerpoint

#endif  // CONFIG_HPP_
