// Copyright 2020 Tier IV, Inc.
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

#ifndef VOXEL_BASED_COMPARE_MAP_FILTER__NODE_HPP_
#define VOXEL_BASED_COMPARE_MAP_FILTER__NODE_HPP_

#include "../voxel_grid_map_loader/voxel_grid_map_loader.hpp"
#include "autoware/pointcloud_preprocessor/filter.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>

#include <memory>
#include <vector>

namespace autoware::compare_map_segmentation
{
class VoxelBasedCompareMapFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

private:
  // pcl::SegmentDifferences<pcl::PointXYZ> impl_;
  std::unique_ptr<VoxelGridMapLoader> voxel_grid_map_loader_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_map_;
  double distance_threshold_;
  bool set_map_in_voxel_grid_;

  bool dynamic_map_load_enable_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit VoxelBasedCompareMapFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::compare_map_segmentation

#endif  // VOXEL_BASED_COMPARE_MAP_FILTER__NODE_HPP_
