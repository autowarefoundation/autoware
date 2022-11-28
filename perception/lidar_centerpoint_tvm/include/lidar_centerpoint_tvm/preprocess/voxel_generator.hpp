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

#ifndef LIDAR_CENTERPOINT_TVM__PREPROCESS__VOXEL_GENERATOR_HPP_
#define LIDAR_CENTERPOINT_TVM__PREPROCESS__VOXEL_GENERATOR_HPP_

#include <lidar_centerpoint_tvm/centerpoint_config.hpp>
#include <lidar_centerpoint_tvm/preprocess/pointcloud_densification.hpp>
#include <lidar_centerpoint_tvm/visibility_control.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <vector>

namespace autoware
{
namespace perception
{
namespace lidar_centerpoint_tvm
{
class LIDAR_CENTERPOINT_TVM_LOCAL VoxelGeneratorTemplate
{
public:
  explicit VoxelGeneratorTemplate(
    const DensificationParam & param, const CenterPointConfig & config);

  virtual std::size_t pointsToVoxels(
    std::vector<float> & voxels, std::vector<int32_t> & coordinates,
    std::vector<float> & num_points_per_voxel) = 0;

  bool enqueuePointCloud(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer);

protected:
  std::unique_ptr<PointCloudDensification> pd_ptr_{nullptr};

  CenterPointConfig config_;
  std::array<float, 6> range_;
  std::array<int32_t, 3> grid_size_;
  std::array<float, 3> recip_voxel_size_;
};

class LIDAR_CENTERPOINT_TVM_LOCAL VoxelGenerator : public VoxelGeneratorTemplate
{
public:
  using VoxelGeneratorTemplate::VoxelGeneratorTemplate;

  /**
   * @brief Traverse all the lidar points and put each point into the corresponding voxel.
   *
   * @param voxels To store point info in each voxel
   * @param coordinates To store the index from voxel number to its 3D position
   * @param num_points_per_voxel To store the number of points in each voxel
   * @return The number of non-empty voxel
   */
  std::size_t pointsToVoxels(
    std::vector<float> & voxels, std::vector<int32_t> & coordinates,
    std::vector<float> & num_points_per_voxel) override;
};

}  // namespace lidar_centerpoint_tvm
}  // namespace perception
}  // namespace autoware

#endif  // LIDAR_CENTERPOINT_TVM__PREPROCESS__VOXEL_GENERATOR_HPP_
