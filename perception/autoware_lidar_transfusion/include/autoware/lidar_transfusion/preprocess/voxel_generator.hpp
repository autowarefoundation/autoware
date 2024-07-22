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

#ifndef AUTOWARE__LIDAR_TRANSFUSION__PREPROCESS__VOXEL_GENERATOR_HPP_
#define AUTOWARE__LIDAR_TRANSFUSION__PREPROCESS__VOXEL_GENERATOR_HPP_

#include "autoware/lidar_transfusion/cuda_utils.hpp"
#include "autoware/lidar_transfusion/preprocess/pointcloud_densification.hpp"
#include "autoware/lidar_transfusion/preprocess/preprocess_kernel.hpp"
#include "autoware/lidar_transfusion/ros_utils.hpp"
#include "autoware/lidar_transfusion/transfusion_config.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <autoware_point_types/types.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace autoware::lidar_transfusion
{
constexpr size_t AFF_MAT_SIZE = 16;  // 4x4 matrix
constexpr size_t MAX_CLOUD_STEP_SIZE = sizeof(autoware_point_types::PointXYZIRCAEDT);

class VoxelGenerator
{
public:
  explicit VoxelGenerator(
    const DensificationParam & densification_param, const TransfusionConfig & config,
    cudaStream_t & stream);
  std::size_t generateSweepPoints(
    const sensor_msgs::msg::PointCloud2 & msg, cuda::unique_ptr<float[]> & points_d);
  bool enqueuePointCloud(
    const sensor_msgs::msg::PointCloud2 & msg, const tf2_ros::Buffer & tf_buffer);
  void initCloudInfo(const sensor_msgs::msg::PointCloud2 & msg);
  std::tuple<const uint32_t, const uint8_t, const uint8_t> getFieldInfo(
    const sensor_msgs::msg::PointCloud2 & msg, const std::string & field_name);

private:
  std::unique_ptr<PointCloudDensification> pd_ptr_{nullptr};
  std::unique_ptr<PreprocessCuda> pre_ptr_{nullptr};
  TransfusionConfig config_;
  cuda::unique_ptr<unsigned char[]> cloud_data_d_{nullptr};
  cuda::unique_ptr<float[]> affine_past2current_d_{nullptr};
  std::vector<float> points_;
  cudaStream_t stream_;
  CloudInfo cloud_info_;
  bool is_initialized_{false};
};

}  // namespace autoware::lidar_transfusion

#endif  // AUTOWARE__LIDAR_TRANSFUSION__PREPROCESS__VOXEL_GENERATOR_HPP_
