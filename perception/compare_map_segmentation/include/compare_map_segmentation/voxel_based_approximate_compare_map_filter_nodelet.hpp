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

#ifndef COMPARE_MAP_SEGMENTATION__VOXEL_BASED_APPROXIMATE_COMPARE_MAP_FILTER_NODELET_HPP_  // NOLINT
#define COMPARE_MAP_SEGMENTATION__VOXEL_BASED_APPROXIMATE_COMPARE_MAP_FILTER_NODELET_HPP_  // NOLINT

#include "pointcloud_preprocessor/filter.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>

#include <vector>

namespace compare_map_segmentation
{
class VoxelBasedApproximateCompareMapFilterComponent : public pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  void input_target_callback(const PointCloud2ConstPtr map);

private:
  // pcl::SegmentDifferences<pcl::PointXYZ> impl_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_map_;
  PointCloudPtr voxel_map_ptr_;
  double distance_threshold_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;
  bool set_map_in_voxel_grid_;

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit VoxelBasedApproximateCompareMapFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace compare_map_segmentation

// clang-format off
#endif  // COMPARE_MAP_SEGMENTATION__VOXEL_BASED_APPROXIMATE_COMPARE_MAP_FILTER_NODELET_HPP_  // NOLINT
// clang-format on
