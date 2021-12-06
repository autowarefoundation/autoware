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

#pragma once

#include "euclidean_cluster/euclidean_cluster_interface.hpp"
#include "euclidean_cluster/utils.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>

#include <vector>

namespace euclidean_cluster
{
class VoxelGridBasedEuclideanCluster : public EuclideanClusterInterface
{
public:
  VoxelGridBasedEuclideanCluster();
  VoxelGridBasedEuclideanCluster(bool use_height, int min_cluster_size, int max_cluster_size);
  VoxelGridBasedEuclideanCluster(
    bool use_height, int min_cluster_size, int max_cluster_size, float tolerance,
    float voxel_leaf_size, int min_points_number_per_voxel);
  bool cluster(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
    std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters) override;
  void setVoxelLeafSize(float voxel_leaf_size) { voxel_leaf_size_ = voxel_leaf_size; }
  void setTolerance(float tolerance) { tolerance_ = tolerance; }
  void setMinPointsNumberPerVoxel(int min_points_number_per_voxel)
  {
    min_points_number_per_voxel_ = min_points_number_per_voxel;
  }

private:
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;
  float tolerance_;
  float voxel_leaf_size_;
  int min_points_number_per_voxel_;
};

}  // namespace euclidean_cluster
