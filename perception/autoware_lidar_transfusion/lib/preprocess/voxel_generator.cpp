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

#include "autoware/lidar_transfusion/preprocess/voxel_generator.hpp"

#include "autoware/lidar_transfusion/preprocess/preprocess_kernel.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <type_traits>

namespace autoware::lidar_transfusion
{

VoxelGenerator::VoxelGenerator(
  const DensificationParam & densification_param, const TransfusionConfig & config,
  cudaStream_t & stream)
: config_(config), stream_(stream)
{
  pd_ptr_ = std::make_unique<PointCloudDensification>(densification_param, stream_);
  pre_ptr_ = std::make_unique<PreprocessCuda>(config_, stream_);
  cloud_data_d_ = cuda::make_unique<unsigned char[]>(config_.cloud_capacity_ * MAX_CLOUD_STEP_SIZE);
  affine_past2current_d_ = cuda::make_unique<float[]>(AFF_MAT_SIZE);
}

bool VoxelGenerator::enqueuePointCloud(
  const sensor_msgs::msg::PointCloud2 & msg, const tf2_ros::Buffer & tf_buffer)
{
  return pd_ptr_->enqueuePointCloud(msg, tf_buffer);
}

std::size_t VoxelGenerator::generateSweepPoints(
  const sensor_msgs::msg::PointCloud2 & msg, cuda::unique_ptr<float[]> & points_d)
{
  if (!is_initialized_) {
    initCloudInfo(msg);
    std::stringstream ss;
    ss << "Input point cloud information: " << std::endl << cloud_info_;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lidar_transfusion"), ss.str());

    CloudInfo default_cloud_info;
    if (cloud_info_ != default_cloud_info) {
      ss << "Expected point cloud information: " << std::endl << default_cloud_info;
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("lidar_transfusion"), ss.str());
      throw std::runtime_error("Input point cloud has unsupported format");
    }
    is_initialized_ = true;
  }

  size_t point_counter{0};
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  for (auto pc_cache_iter = pd_ptr_->getPointCloudCacheIter(); !pd_ptr_->isCacheEnd(pc_cache_iter);
       pc_cache_iter++) {
    auto sweep_num_points = pc_cache_iter->num_points;
    if (point_counter + sweep_num_points >= config_.cloud_capacity_) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("lidar_transfusion"), "Exceeding cloud capacity. Used "
                                                   << pd_ptr_->getIdx(pc_cache_iter) << " out of "
                                                   << pd_ptr_->getCacheSize() << " sweep(s)");
      break;
    }
    auto shift = point_counter * config_.num_point_feature_size_;

    auto affine_past2current =
      pd_ptr_->getAffineWorldToCurrent() * pc_cache_iter->affine_past2world;
    static_assert(std::is_same<decltype(affine_past2current.matrix()), Eigen::Matrix4f &>::value);
    static_assert(!Eigen::Matrix4f::IsRowMajor, "matrices should be col-major.");

    float time_lag = static_cast<float>(
      pd_ptr_->getCurrentTimestamp() - rclcpp::Time(pc_cache_iter->header.stamp).seconds());

    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      affine_past2current_d_.get(), affine_past2current.data(), AFF_MAT_SIZE * sizeof(float),
      cudaMemcpyHostToDevice, stream_));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    pre_ptr_->generateSweepPoints_launch(
      pc_cache_iter->data_d.get(), sweep_num_points, cloud_info_.point_step, time_lag,
      affine_past2current_d_.get(), points_d.get() + shift);
    point_counter += sweep_num_points;
  }

  return point_counter;
}

void VoxelGenerator::initCloudInfo(const sensor_msgs::msg::PointCloud2 & msg)
{
  std::tie(cloud_info_.x_offset, cloud_info_.x_datatype, cloud_info_.x_count) =
    getFieldInfo(msg, "x");
  std::tie(cloud_info_.y_offset, cloud_info_.y_datatype, cloud_info_.y_count) =
    getFieldInfo(msg, "y");
  std::tie(cloud_info_.z_offset, cloud_info_.z_datatype, cloud_info_.z_count) =
    getFieldInfo(msg, "z");
  std::tie(
    cloud_info_.intensity_offset, cloud_info_.intensity_datatype, cloud_info_.intensity_count) =
    getFieldInfo(msg, "intensity");
  cloud_info_.point_step = msg.point_step;
  cloud_info_.is_bigendian = msg.is_bigendian;
}

std::tuple<const uint32_t, const uint8_t, const uint8_t> VoxelGenerator::getFieldInfo(
  const sensor_msgs::msg::PointCloud2 & msg, const std::string & field_name)
{
  for (const auto & field : msg.fields) {
    if (field.name == field_name) {
      return std::make_tuple(field.offset, field.datatype, field.count);
    }
  }
  throw std::runtime_error("Missing field: " + field_name);
}
}  // namespace autoware::lidar_transfusion
