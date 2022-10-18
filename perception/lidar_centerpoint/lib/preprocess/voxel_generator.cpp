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

std::size_t VoxelGenerator::pointsToVoxels(
  std::vector<float> & voxels, std::vector<int> & coordinates,
  std::vector<float> & num_points_per_voxel)
{
  // voxels (float): (max_voxel_size * max_point_in_voxel_size * point_feature_size)
  // coordinates (int): (max_voxel_size * point_dim_size)
  // num_points_per_voxel (float): (max_voxel_size)

  const std::size_t grid_size = config_.grid_size_z_ * config_.grid_size_y_ * config_.grid_size_x_;
  std::vector<int> coord_to_voxel_idx(grid_size, -1);

  std::size_t voxel_cnt = 0;  // @return
  std::vector<float> point;
  point.resize(config_.point_feature_size_);
  std::vector<float> coord_zyx;
  coord_zyx.resize(config_.point_dim_size_);
  bool out_of_range;
  std::size_t point_cnt;
  int c, coord_idx, voxel_idx;
  Eigen::Vector3f point_current, point_past;

  for (auto pc_cache_iter = pd_ptr_->getPointCloudCacheIter(); !pd_ptr_->isCacheEnd(pc_cache_iter);
       pc_cache_iter++) {
    auto pc_msg = pc_cache_iter->pointcloud_msg;
    auto affine_past2current =
      pd_ptr_->pointcloud_cache_size() > 1
        ? pd_ptr_->getAffineWorldToCurrent() * pc_cache_iter->affine_past2world
        : Eigen::Affine3f::Identity();
    float timelag = static_cast<float>(
      pd_ptr_->getCurrentTimestamp() - rclcpp::Time(pc_msg.header.stamp).seconds());

    for (sensor_msgs::PointCloud2ConstIterator<float> x_iter(pc_msg, "x"), y_iter(pc_msg, "y"),
         z_iter(pc_msg, "z");
         x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter) {
      point_past << *x_iter, *y_iter, *z_iter;
      point_current = affine_past2current * point_past;

      point[0] = point_current.x();
      point[1] = point_current.y();
      point[2] = point_current.z();
      point[3] = timelag;

      out_of_range = false;
      for (std::size_t di = 0; di < config_.point_dim_size_; di++) {
        c = static_cast<int>((point[di] - range_[di]) * recip_voxel_size_[di]);
        if (c < 0 || c >= grid_size_[di]) {
          out_of_range = true;
          break;
        }
        coord_zyx[config_.point_dim_size_ - di - 1] = c;
      }
      if (out_of_range) {
        continue;
      }

      coord_idx = coord_zyx[0] * config_.grid_size_y_ * config_.grid_size_x_ +
                  coord_zyx[1] * config_.grid_size_x_ + coord_zyx[2];
      voxel_idx = coord_to_voxel_idx[coord_idx];
      if (voxel_idx == -1) {
        voxel_idx = voxel_cnt;
        if (voxel_cnt >= config_.max_voxel_size_) {
          continue;
        }

        voxel_cnt++;
        coord_to_voxel_idx[coord_idx] = voxel_idx;
        for (std::size_t di = 0; di < config_.point_dim_size_; di++) {
          coordinates[voxel_idx * config_.point_dim_size_ + di] = coord_zyx[di];
        }
      }

      point_cnt = num_points_per_voxel[voxel_idx];
      if (point_cnt < config_.max_point_in_voxel_size_) {
        for (std::size_t fi = 0; fi < config_.point_feature_size_; fi++) {
          voxels
            [voxel_idx * config_.max_point_in_voxel_size_ * config_.point_feature_size_ +
             point_cnt * config_.point_feature_size_ + fi] = point[fi];
        }
        num_points_per_voxel[voxel_idx]++;
      }
    }
  }

  return voxel_cnt;
}

}  // namespace centerpoint
