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

#include "image_projection_based_fusion/pointpainting_fusion/voxel_generator.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace image_projection_based_fusion
{
std::size_t VoxelGenerator::pointsToVoxels(
  std::vector<float> & voxels, std::vector<int> & coordinates,
  std::vector<float> & num_points_per_voxel)
{
  // voxels (float): (max_num_voxels * max_num_points_per_voxel * point_feature_size)
  // coordinates (int): (max_num_voxels * point_dim_size)
  // num_points_per_voxel (float): (max_num_voxels)

  const std::size_t grid_size = config_.grid_size_z_ * config_.grid_size_y_ * config_.grid_size_x_;
  std::vector<std::optional<int>> coord_to_voxel_idx(grid_size, -1);

  std::size_t voxel_cnt = 0;  // @return
  // std::array<float, config_.point_feature_size> point;
  // std::array<float, config_.point_dim_size> coord_zyx;
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
      pd_ptr_->getAffineWorldToCurrent() * pc_cache_iter->affine_past2world;
    float timelag = static_cast<float>(
      pd_ptr_->getCurrentTimestamp() - rclcpp::Time(pc_msg.header.stamp).seconds());

    for (sensor_msgs::PointCloud2ConstIterator<float> x_iter(pc_msg, "x"), y_iter(pc_msg, "y"),
         z_iter(pc_msg, "z"), car_iter(pc_msg, "CAR"), ped_iter(pc_msg, "PEDESTRIAN"),
         bic_iter(pc_msg, "BICYCLE");
         x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter, ++car_iter, ++ped_iter, ++bic_iter) {
      point_past << *x_iter, *y_iter, *z_iter;
      point_current = affine_past2current * point_past;

      point[0] = point_current.x();
      point[1] = point_current.y();
      point[2] = point_current.z();
      point[3] = timelag;
      point[4] = *car_iter;
      point[5] = *ped_iter;
      point[6] = *bic_iter;

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
      voxel_idx = coord_to_voxel_idx[coord_idx].value();
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

}  // namespace image_projection_based_fusion
