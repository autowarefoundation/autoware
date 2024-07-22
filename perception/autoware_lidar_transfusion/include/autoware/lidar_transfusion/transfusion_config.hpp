// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__LIDAR_TRANSFUSION__TRANSFUSION_CONFIG_HPP_
#define AUTOWARE__LIDAR_TRANSFUSION__TRANSFUSION_CONFIG_HPP_

#include <cstdint>
#include <vector>

namespace autoware::lidar_transfusion
{

class TransfusionConfig
{
public:
  TransfusionConfig(
    const std::size_t cloud_capacity, const std::vector<int64_t> & voxels_num,
    const std::vector<double> & point_cloud_range, const std::vector<double> & voxel_size,
    const std::size_t num_proposals, const float circle_nms_dist_threshold,
    const std::vector<double> & yaw_norm_thresholds, const float score_threshold)
  {
    cloud_capacity_ = cloud_capacity;

    if (voxels_num.size() == 3) {
      max_voxels_ = voxels_num[2];

      voxels_num_[0] = voxels_num[0];
      voxels_num_[1] = voxels_num[1];
      voxels_num_[2] = voxels_num[2];

      min_voxel_size_ = voxels_num[0];
      opt_voxel_size_ = voxels_num[1];
      max_voxel_size_ = voxels_num[2];

      min_points_size_ = voxels_num[0];
      opt_points_size_ = voxels_num[1];
      max_points_size_ = voxels_num[2];

      min_coors_size_ = voxels_num[0];
      opt_coors_size_ = voxels_num[1];
      max_coors_size_ = voxels_num[2];
    }
    if (point_cloud_range.size() == 6) {
      min_x_range_ = static_cast<float>(point_cloud_range[0]);
      min_y_range_ = static_cast<float>(point_cloud_range[1]);
      min_z_range_ = static_cast<float>(point_cloud_range[2]);
      max_x_range_ = static_cast<float>(point_cloud_range[3]);
      max_y_range_ = static_cast<float>(point_cloud_range[4]);
      max_z_range_ = static_cast<float>(point_cloud_range[5]);
    }
    if (voxel_size.size() == 3) {
      voxel_x_size_ = static_cast<float>(voxel_size[0]);
      voxel_y_size_ = static_cast<float>(voxel_size[1]);
      voxel_z_size_ = static_cast<float>(voxel_size[2]);
    }
    if (num_proposals > 0) {
      num_proposals_ = num_proposals;
    }
    if (score_threshold > 0.0) {
      score_threshold_ = score_threshold;
    }
    if (circle_nms_dist_threshold > 0.0) {
      circle_nms_dist_threshold_ = circle_nms_dist_threshold;
    }
    yaw_norm_thresholds_ =
      std::vector<float>(yaw_norm_thresholds.begin(), yaw_norm_thresholds.end());
    for (auto & yaw_norm_threshold : yaw_norm_thresholds_) {
      yaw_norm_threshold =
        (yaw_norm_threshold >= 0.0 && yaw_norm_threshold < 1.0) ? yaw_norm_threshold : 0.0;
    }
    grid_x_size_ = static_cast<std::size_t>((max_x_range_ - min_x_range_) / voxel_x_size_);
    grid_y_size_ = static_cast<std::size_t>((max_y_range_ - min_y_range_) / voxel_y_size_);
    grid_z_size_ = static_cast<std::size_t>((max_z_range_ - min_z_range_) / voxel_z_size_);

    feature_x_size_ = grid_x_size_ / out_size_factor_;
    feature_y_size_ = grid_y_size_ / out_size_factor_;
  }

  ///// INPUT PARAMETERS /////
  std::size_t cloud_capacity_{};
  ///// KERNEL PARAMETERS /////
  const std::size_t threads_for_voxel_{256};  // threads number for a block
  const std::size_t points_per_voxel_{20};
  const std::size_t warp_size_{32};          // one warp(32 threads) for one pillar
  const std::size_t pillars_per_block_{64};  // one thread deals with one pillar
                                             // and a block has pillars_per_block threads
  const std::size_t pillar_feature_size_{64};
  std::size_t max_voxels_{60000};

  ///// NETWORK PARAMETERS /////
  const std::size_t batch_size_{1};
  const std::size_t num_classes_{5};
  const std::size_t num_point_feature_size_{5};  // x, y, z, intensity, lag
  // the dimension of the input cloud
  float min_x_range_{-76.8};
  float max_x_range_{76.8};
  float min_y_range_{-76.8};
  float max_y_range_{76.8};
  float min_z_range_{-3.0};
  float max_z_range_{5.0};
  // the size of a pillar
  float voxel_x_size_{0.3};
  float voxel_y_size_{0.3};
  float voxel_z_size_{8.0};
  const std::size_t out_size_factor_{4};
  const std::size_t max_num_points_per_pillar_{points_per_voxel_};
  const std::size_t num_point_values_{4};
  std::size_t num_proposals_{200};
  // the number of feature maps for pillar scatter
  const std::size_t num_feature_scatter_{pillar_feature_size_};
  // the score threshold for classification
  float score_threshold_{0.2};
  float circle_nms_dist_threshold_{0.5};
  std::vector<float> yaw_norm_thresholds_{0.3, 0.3, 0.3, 0.3, 0.0};
  std::size_t max_num_pillars_{max_voxels_};
  const std::size_t pillar_points_bev_{max_num_points_per_pillar_ * max_num_pillars_};
  // the detected boxes result decode by (x, y, z, w, l, h, yaw)
  const std::size_t num_box_values_{8};
  // the input size of the 2D backbone network
  std::size_t grid_x_size_{512};
  std::size_t grid_y_size_{512};
  std::size_t grid_z_size_{1};
  // the output size of the 2D backbone network
  std::size_t feature_x_size_{grid_x_size_ / out_size_factor_};
  std::size_t feature_y_size_{grid_y_size_ / out_size_factor_};

  ///// RUNTIME DIMENSIONS /////
  std::vector<std::size_t> voxels_num_{5000, 30000, 60000};
  // voxels
  std::size_t min_voxel_size_{voxels_num_[0]};
  std::size_t opt_voxel_size_{voxels_num_[1]};
  std::size_t max_voxel_size_{voxels_num_[2]};

  std::size_t min_point_in_voxel_size_{points_per_voxel_};
  std::size_t opt_point_in_voxel_size_{points_per_voxel_};
  std::size_t max_point_in_voxel_size_{points_per_voxel_};

  std::size_t min_network_feature_size_{num_point_feature_size_};
  std::size_t opt_network_feature_size_{num_point_feature_size_};
  std::size_t max_network_feature_size_{num_point_feature_size_};

  // num_points
  std::size_t min_points_size_{voxels_num_[0]};
  std::size_t opt_points_size_{voxels_num_[1]};
  std::size_t max_points_size_{voxels_num_[2]};

  // coors
  std::size_t min_coors_size_{voxels_num_[0]};
  std::size_t opt_coors_size_{voxels_num_[1]};
  std::size_t max_coors_size_{voxels_num_[2]};

  std::size_t min_coors_dim_size_{num_point_values_};
  std::size_t opt_coors_dim_size_{num_point_values_};
  std::size_t max_coors_dim_size_{num_point_values_};
};

}  // namespace autoware::lidar_transfusion

#endif  // AUTOWARE__LIDAR_TRANSFUSION__TRANSFUSION_CONFIG_HPP_
