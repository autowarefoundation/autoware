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

#include "autoware/lidar_centerpoint/preprocess/voxel_generator.hpp"

#include "autoware/lidar_centerpoint/centerpoint_trt.hpp"
#include "autoware/lidar_centerpoint/preprocess/preprocess_kernel.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <type_traits>

namespace autoware::lidar_centerpoint
{
VoxelGeneratorTemplate::VoxelGeneratorTemplate(
  const DensificationParam & param, const CenterPointConfig & config)
: config_(config)
{
  pd_ptr_ = std::make_unique<PointCloudDensification>(param);
  range_[0] = config.range_min_x_;
  range_[1] = config.range_min_y_;
  range_[2] = config.range_min_z_;
  range_[3] = config.range_max_x_;
  range_[4] = config.range_max_y_;
  range_[5] = config.range_max_z_;
  grid_size_[0] = config.grid_size_x_;
  grid_size_[1] = config.grid_size_y_;
  grid_size_[2] = config.grid_size_z_;
  recip_voxel_size_[0] = 1 / config.voxel_size_x_;
  recip_voxel_size_[1] = 1 / config.voxel_size_y_;
  recip_voxel_size_[2] = 1 / config.voxel_size_z_;
}

bool VoxelGeneratorTemplate::enqueuePointCloud(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer,
  cudaStream_t stream)
{
  return pd_ptr_->enqueuePointCloud(input_pointcloud_msg, tf_buffer, stream);
}

std::size_t VoxelGenerator::generateSweepPoints(float * points_d, cudaStream_t stream)
{
  std::size_t point_counter = 0;
  for (auto pc_cache_iter = pd_ptr_->getPointCloudCacheIter(); !pd_ptr_->isCacheEnd(pc_cache_iter);
       pc_cache_iter++) {
    auto sweep_num_points = pc_cache_iter->num_points;
    auto point_step = pc_cache_iter->point_step;
    auto affine_past2current =
      pd_ptr_->getAffineWorldToCurrent() * pc_cache_iter->affine_past2world;
    float time_lag = static_cast<float>(
      pd_ptr_->getCurrentTimestamp() - rclcpp::Time(pc_cache_iter->header.stamp).seconds());

    if (point_counter + sweep_num_points > config_.cloud_capacity_) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("lidar_centerpoint"),
        "Requested number of points exceeds the maximum capacity. Current points = "
          << point_counter);
      break;
    }

    static_assert(std::is_same<decltype(affine_past2current.matrix()), Eigen::Matrix4f &>::value);
    static_assert(!Eigen::Matrix4f::IsRowMajor, "matrices should be col-major.");
    generateSweepPoints_launch(
      pc_cache_iter->points_d.get(), sweep_num_points, point_step / sizeof(float), time_lag,
      affine_past2current.matrix().data(), config_.point_feature_size_,
      points_d + config_.point_feature_size_ * point_counter, stream);

    point_counter += sweep_num_points;
  }
  return point_counter;
}

}  // namespace autoware::lidar_centerpoint
