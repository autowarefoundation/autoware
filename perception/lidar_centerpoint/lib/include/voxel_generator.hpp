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

#ifndef VOXEL_GENERATOR_HPP_
#define VOXEL_GENERATOR_HPP_

#include <config.hpp>
#include <pointcloud_densification.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <torch/torch.h>

#include <memory>

namespace centerpoint
{
class VoxelGeneratorTemplate
{
public:
  explicit VoxelGeneratorTemplate(const DensificationParam & param);

  virtual int pointsToVoxels(
    at::Tensor & voxels, at::Tensor & coordinates, at::Tensor & num_points_per_voxel) = 0;

  std::unique_ptr<PointCloudDensification> pd_ptr_{nullptr};

protected:
  std::array<float, 6> pointcloud_range_{
    Config::pointcloud_range_xmin, Config::pointcloud_range_ymin, Config::pointcloud_range_zmin,
    Config::pointcloud_range_xmax, Config::pointcloud_range_ymax, Config::pointcloud_range_zmax};
  std::array<float, 3> recip_voxel_size_{
    1 / Config::voxel_size_x, 1 / Config::voxel_size_y, 1 / Config::voxel_size_z};
  std::array<int, 3> grid_size_{Config::grid_size_x, Config::grid_size_y, Config::grid_size_z};
};

class VoxelGenerator : public VoxelGeneratorTemplate
{
public:
  using VoxelGeneratorTemplate::VoxelGeneratorTemplate;

  // explicit VoxelGenerator(const DensificationParam & param);

  int pointsToVoxels(
    at::Tensor & voxels, at::Tensor & coordinates, at::Tensor & num_points_per_voxel) override;
};

}  // namespace centerpoint

#endif  // VOXEL_GENERATOR_HPP_
