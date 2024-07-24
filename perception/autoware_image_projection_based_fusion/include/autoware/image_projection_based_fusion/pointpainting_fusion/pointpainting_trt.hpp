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

#ifndef AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__POINTPAINTING_FUSION__POINTPAINTING_TRT_HPP_
#define AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__POINTPAINTING_FUSION__POINTPAINTING_TRT_HPP_

#include <autoware/image_projection_based_fusion/pointpainting_fusion/voxel_generator.hpp>
#include <autoware/lidar_centerpoint/centerpoint_trt.hpp>

#include <memory>
#include <string>

namespace autoware::image_projection_based_fusion
{
class PointPaintingTRT : public autoware::lidar_centerpoint::CenterPointTRT
{
public:
  using autoware::lidar_centerpoint::CenterPointTRT::CenterPointTRT;

  explicit PointPaintingTRT(
    const autoware::lidar_centerpoint::NetworkParam & encoder_param,
    const autoware::lidar_centerpoint::NetworkParam & head_param,
    const autoware::lidar_centerpoint::DensificationParam & densification_param,
    const autoware::lidar_centerpoint::CenterPointConfig & config);

protected:
  bool preprocess(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
    const tf2_ros::Buffer & tf_buffer) override;

  std::unique_ptr<image_projection_based_fusion::VoxelGenerator> vg_ptr_pp_{nullptr};
};
}  // namespace autoware::image_projection_based_fusion

#endif  // AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__POINTPAINTING_FUSION__POINTPAINTING_TRT_HPP_
