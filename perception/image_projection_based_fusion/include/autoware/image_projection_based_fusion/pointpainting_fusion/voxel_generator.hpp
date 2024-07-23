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

#ifndef AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__POINTPAINTING_FUSION__VOXEL_GENERATOR_HPP_
#define AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__POINTPAINTING_FUSION__VOXEL_GENERATOR_HPP_

#include <autoware/image_projection_based_fusion/pointpainting_fusion/pointcloud_densification.hpp>
#include <autoware/lidar_centerpoint/preprocess/pointcloud_densification.hpp>
#include <autoware/lidar_centerpoint/preprocess/voxel_generator.hpp>

#include <bitset>
#include <memory>
#include <vector>

namespace autoware::image_projection_based_fusion
{

class VoxelGenerator
{
public:
  explicit VoxelGenerator(
    const autoware::lidar_centerpoint::DensificationParam & param,
    const autoware::lidar_centerpoint::CenterPointConfig & config);

  bool enqueuePointCloud(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer);

  std::size_t generateSweepPoints(std::vector<float> & points);

protected:
  std::unique_ptr<PointCloudDensification> pd_ptr_{nullptr};

  autoware::lidar_centerpoint::CenterPointConfig config_;
  std::array<float, 6> range_;
  std::array<int, 3> grid_size_;
  std::array<float, 3> recip_voxel_size_;
};
}  // namespace autoware::image_projection_based_fusion

#endif  // AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__POINTPAINTING_FUSION__VOXEL_GENERATOR_HPP_
