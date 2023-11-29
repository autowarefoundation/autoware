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
size_t VoxelGenerator::generateSweepPoints(std::vector<float> & points)
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
         z_iter(pc_msg, "z"), class_iter(pc_msg, "CLASS");
         x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter, ++class_iter) {
      point_past << *x_iter, *y_iter, *z_iter;
      point_current = affine_past2current * point_past;

      points.at(point_counter * config_.point_feature_size_) = point_current.x();
      points.at(point_counter * config_.point_feature_size_ + 1) = point_current.y();
      points.at(point_counter * config_.point_feature_size_ + 2) = point_current.z();
      points.at(point_counter * config_.point_feature_size_ + 3) = time_lag;
      // decode the class value back to one-hot binary and assign it to point
      for (size_t i = 0; i < config_.class_size_; ++i) {
        points.at(point_counter * config_.point_feature_size_ + 4 + i) = 0.f;
      }
      auto class_value = static_cast<int>(*class_iter);
      auto iter = points.begin() + point_counter * config_.point_feature_size_ + 4;
      while (class_value > 0) {
        *iter = class_value % 2;
        class_value /= 2;
        ++iter;
      }
      ++point_counter;
    }
  }
  return point_counter;
}

}  // namespace image_projection_based_fusion
