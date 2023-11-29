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

#include "lidar_centerpoint/preprocess/voxel_generator.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace centerpoint
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
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer)
{
  return pd_ptr_->enqueuePointCloud(input_pointcloud_msg, tf_buffer);
}

std::size_t VoxelGenerator::generateSweepPoints(std::vector<float> & points)
{
  Eigen::Vector3f point_current, point_past;
  size_t point_counter{};
  for (auto pc_cache_iter = pd_ptr_->getPointCloudCacheIter(); !pd_ptr_->isCacheEnd(pc_cache_iter);
       pc_cache_iter++) {
    auto pc_msg = pc_cache_iter->pointcloud_msg;
    auto affine_past2current =
      pd_ptr_->getAffineWorldToCurrent() * pc_cache_iter->affine_past2world;
    float time_lag = static_cast<float>(
      pd_ptr_->getCurrentTimestamp() - rclcpp::Time(pc_msg.header.stamp).seconds());

    for (sensor_msgs::PointCloud2ConstIterator<float> x_iter(pc_msg, "x"), y_iter(pc_msg, "y"),
         z_iter(pc_msg, "z");
         x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter) {
      point_past << *x_iter, *y_iter, *z_iter;
      point_current = affine_past2current * point_past;

      points.at(point_counter * config_.point_feature_size_) = point_current.x();
      points.at(point_counter * config_.point_feature_size_ + 1) = point_current.y();
      points.at(point_counter * config_.point_feature_size_ + 2) = point_current.z();
      points.at(point_counter * config_.point_feature_size_ + 3) = time_lag;
      ++point_counter;
    }
  }
  return point_counter;
}

}  // namespace centerpoint
