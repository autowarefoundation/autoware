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

#include <centerpoint_trt.hpp>
#include <preprocess_kernel.hpp>
#include <scatter_kernel.hpp>
#include <tier4_autoware_utils/math/constants.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace centerpoint
{
CenterPointTRT::CenterPointTRT(
  const std::size_t num_class, const float score_threshold, const NetworkParam & encoder_param,
  const NetworkParam & head_param, const DensificationParam & densification_param)
: num_class_(num_class)
{
  vg_ptr_ = std::make_unique<VoxelGenerator>(densification_param);
  post_proc_ptr_ = std::make_unique<PostProcessCUDA>(num_class, score_threshold);

  // encoder
  encoder_trt_ptr_ = std::make_unique<VoxelEncoderTRT>(verbose_);
  encoder_trt_ptr_->init(
    encoder_param.onnx_path(), encoder_param.engine_path(), encoder_param.trt_precision());
  encoder_trt_ptr_->context_->setBindingDimensions(
    0,
    nvinfer1::Dims3(
      Config::max_num_voxels, Config::max_num_points_per_voxel, Config::encoder_in_feature_size));

  // head
  head_trt_ptr_ = std::make_unique<HeadTRT>(num_class, verbose_);
  head_trt_ptr_->init(head_param.onnx_path(), head_param.engine_path(), head_param.trt_precision());
  head_trt_ptr_->context_->setBindingDimensions(
    0, nvinfer1::Dims4(
         Config::batch_size, Config::encoder_out_feature_size, Config::grid_size_y,
         Config::grid_size_x));

  initPtr();

  cudaStreamCreate(&stream_);
}

CenterPointTRT::~CenterPointTRT()
{
  if (stream_) {
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
  }
}

void CenterPointTRT::initPtr()
{
  const auto voxels_size =
    Config::max_num_voxels * Config::max_num_points_per_voxel * Config::point_feature_size;
  const auto coordinates_size = Config::max_num_voxels * Config::point_dim_size;
  encoder_in_feature_size_ =
    Config::max_num_voxels * Config::max_num_points_per_voxel * Config::encoder_in_feature_size;
  const auto pillar_features_size = Config::max_num_voxels * Config::encoder_out_feature_size;
  spatial_features_size_ =
    Config::grid_size_x * Config::grid_size_y * Config::encoder_out_feature_size;
  const auto grid_xy = Config::down_grid_size_x * Config::down_grid_size_y;

  // host
  voxels_.resize(voxels_size);
  coordinates_.resize(coordinates_size);
  num_points_per_voxel_.resize(Config::max_num_voxels);

  // device
  voxels_d_ = cuda::make_unique<float[]>(voxels_size);
  coordinates_d_ = cuda::make_unique<int[]>(coordinates_size);
  num_points_per_voxel_d_ = cuda::make_unique<float[]>(Config::max_num_voxels);
  encoder_in_features_d_ = cuda::make_unique<float[]>(encoder_in_feature_size_);
  pillar_features_d_ = cuda::make_unique<float[]>(pillar_features_size);
  spatial_features_d_ = cuda::make_unique<float[]>(spatial_features_size_);
  head_out_heatmap_d_ = cuda::make_unique<float[]>(grid_xy * num_class_);
  head_out_offset_d_ = cuda::make_unique<float[]>(grid_xy * Config::head_out_offset_size);
  head_out_z_d_ = cuda::make_unique<float[]>(grid_xy * Config::head_out_z_size);
  head_out_dim_d_ = cuda::make_unique<float[]>(grid_xy * Config::head_out_dim_size);
  head_out_rot_d_ = cuda::make_unique<float[]>(grid_xy * Config::head_out_rot_size);
  head_out_vel_d_ = cuda::make_unique<float[]>(grid_xy * Config::head_out_vel_size);
}

bool CenterPointTRT::detect(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer,
  std::vector<Box3D> & det_boxes3d)
{
  std::fill(voxels_.begin(), voxels_.end(), 0);
  std::fill(coordinates_.begin(), coordinates_.end(), -1);
  std::fill(num_points_per_voxel_.begin(), num_points_per_voxel_.end(), 0);
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    encoder_in_features_d_.get(), 0, encoder_in_feature_size_ * sizeof(float), stream_));
  CHECK_CUDA_ERROR(
    cudaMemsetAsync(spatial_features_d_.get(), 0, spatial_features_size_ * sizeof(float), stream_));

  if (!preprocess(input_pointcloud_msg, tf_buffer)) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("lidar_centerpoint"), "Fail to preprocess and skip to detect.");
    return false;
  }

  inference();

  postProcess(det_boxes3d);

  return true;
}

bool CenterPointTRT::preprocess(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer)
{
  bool is_success = vg_ptr_->enqueuePointCloud(input_pointcloud_msg, tf_buffer);
  if (!is_success) {
    return false;
  }
  num_voxels_ = vg_ptr_->pointsToVoxels(voxels_, coordinates_, num_points_per_voxel_);
  if (num_voxels_ == 0) {
    return false;
  }

  const auto voxels_size =
    num_voxels_ * Config::max_num_points_per_voxel * Config::point_feature_size;
  const auto coordinates_size = num_voxels_ * Config::point_dim_size;
  // memcpy from host to device (not copy empty voxels)
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    voxels_d_.get(), voxels_.data(), voxels_size * sizeof(float), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    coordinates_d_.get(), coordinates_.data(), coordinates_size * sizeof(int),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    num_points_per_voxel_d_.get(), num_points_per_voxel_.data(), num_voxels_ * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  CHECK_CUDA_ERROR(generateFeatures_launch(
    voxels_d_.get(), num_points_per_voxel_d_.get(), coordinates_d_.get(), num_voxels_,
    encoder_in_features_d_.get(), stream_));

  return true;
}

void CenterPointTRT::inference()
{
  if (!encoder_trt_ptr_->context_ || !head_trt_ptr_->context_) {
    throw std::runtime_error("Failed to create tensorrt context.");
  }

  // pillar encoder network
  std::vector<void *> encoder_buffers{encoder_in_features_d_.get(), pillar_features_d_.get()};
  encoder_trt_ptr_->context_->enqueueV2(encoder_buffers.data(), stream_, nullptr);

  // scatter
  CHECK_CUDA_ERROR(scatterFeatures_launch(
    pillar_features_d_.get(), coordinates_d_.get(), num_voxels_, spatial_features_d_.get(),
    stream_));

  // head network
  std::vector<void *> head_buffers = {spatial_features_d_.get(), head_out_heatmap_d_.get(),
                                      head_out_offset_d_.get(),  head_out_z_d_.get(),
                                      head_out_dim_d_.get(),     head_out_rot_d_.get(),
                                      head_out_vel_d_.get()};
  head_trt_ptr_->context_->enqueueV2(head_buffers.data(), stream_, nullptr);
}

void CenterPointTRT::postProcess(std::vector<Box3D> & det_boxes3d)
{
  CHECK_CUDA_ERROR(post_proc_ptr_->generateDetectedBoxes3D_launch(
    head_out_heatmap_d_.get(), head_out_offset_d_.get(), head_out_z_d_.get(), head_out_dim_d_.get(),
    head_out_rot_d_.get(), head_out_vel_d_.get(), det_boxes3d, stream_));
  if (det_boxes3d.size() == 0) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("lidar_centerpoint"), "No detected boxes.");
  }
}

}  // namespace centerpoint
