// Copyright 2021-2022 AutoCore Ltd., TIER IV, Inc., Arm Ltd.
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

#include "lidar_centerpoint_tvm/centerpoint_tvm.hpp"

#include <centerpoint_backbone/inference_engine_tvm_config.hpp>
#include <centerpoint_encoder/inference_engine_tvm_config.hpp>
#include <lidar_centerpoint_tvm/centerpoint_config.hpp>
#include <lidar_centerpoint_tvm/network/scatter.hpp>
#include <lidar_centerpoint_tvm/preprocess/generate_features.hpp>
#include <tier4_autoware_utils/math/constants.hpp>
#include <tvm_utility/pipeline.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

auto config_en = model_zoo::perception::lidar_obstacle_detection::centerpoint_encoder::
  onnx_centerpoint_encoder::config;
auto config_bk = model_zoo::perception::lidar_obstacle_detection::centerpoint_backbone::
  onnx_centerpoint_backbone::config;

namespace autoware
{
namespace perception
{
namespace lidar_centerpoint_tvm
{

VoxelEncoderPreProcessor::VoxelEncoderPreProcessor(
  const tvm_utility::pipeline::InferenceEngineTVMConfig & config,
  const CenterPointConfig & config_mod)
: max_voxel_size(config.network_inputs[0].second[0]),
  max_point_in_voxel_size(config.network_inputs[0].second[1]),
  encoder_in_feature_size(config.network_inputs[0].second[2]),
  datatype_bytes(config.tvm_dtype_bits / 8),
  config_detail(config_mod)
{
  encoder_in_features.resize(max_voxel_size * max_point_in_voxel_size * encoder_in_feature_size);
  // Allocate input variable
  std::vector<int64_t> shape_x{1, max_voxel_size, max_point_in_voxel_size, encoder_in_feature_size};
  TVMArrayContainer x{
    shape_x,
    config.tvm_dtype_code,
    config.tvm_dtype_bits,
    config.tvm_dtype_lanes,
    config.tvm_device_type,
    config.tvm_device_id};
  output = x;
}

TVMArrayContainerVector VoxelEncoderPreProcessor::schedule(const MixedInputs & voxel_inputs)
{
  /// generate encoder_in_features from the voxels
  generateFeatures(
    voxel_inputs.features, voxel_inputs.num_points_per_voxel, voxel_inputs.coords,
    voxel_inputs.num_voxels, config_detail, encoder_in_features);

  TVMArrayCopyFromBytes(
    output.getArray(), encoder_in_features.data(),
    max_voxel_size * max_point_in_voxel_size * encoder_in_feature_size * datatype_bytes);

  return {output};
}

VoxelEncoderPostProcessor::VoxelEncoderPostProcessor(
  const tvm_utility::pipeline::InferenceEngineTVMConfig & config)
: max_voxel_size(config.network_outputs[0].second[0]),
  encoder_out_feature_size(config.network_outputs[0].second[2]),
  datatype_bytes(config.tvm_dtype_bits / 8)
{
  pillar_features =
    std::make_shared<std::vector<float>>(max_voxel_size * encoder_out_feature_size, 0);
}

std::shared_ptr<std::vector<float>> VoxelEncoderPostProcessor::schedule(
  const TVMArrayContainerVector & input)
{
  TVMArrayCopyToBytes(
    input[0].getArray(), pillar_features->data(), pillar_features->size() * datatype_bytes);

  return pillar_features;
}

BackboneNeckHeadPreProcessor::BackboneNeckHeadPreProcessor(
  const tvm_utility::pipeline::InferenceEngineTVMConfig & config,
  const CenterPointConfig & config_mod)
: input_channels(config.network_inputs[0].second[1]),
  input_height(config.network_inputs[0].second[2]),
  input_width(config.network_inputs[0].second[3]),
  datatype_bytes(config.tvm_dtype_bits / 8),
  config_detail(config_mod)
{
  spatial_features.resize(input_channels * input_height * input_width);
  // Allocate input variable
  std::vector<int64_t> shape_x{1, input_channels, input_height, input_width};
  TVMArrayContainer x{
    shape_x,
    config.tvm_dtype_code,
    config.tvm_dtype_bits,
    config.tvm_dtype_lanes,
    config.tvm_device_type,
    config.tvm_device_id};
  output = x;
}

TVMArrayContainerVector BackboneNeckHeadPreProcessor::schedule(const MixedInputs & pillar_inputs)
{
  // do scatter to convert pillar_features to spatial_features
  scatterFeatures(
    pillar_inputs.features, pillar_inputs.coords, pillar_inputs.num_voxels, config_detail,
    spatial_features);

  TVMArrayCopyFromBytes(
    output.getArray(), spatial_features.data(),
    input_channels * input_height * input_width * datatype_bytes);

  return {output};
}

BackboneNeckHeadPostProcessor::BackboneNeckHeadPostProcessor(
  const tvm_utility::pipeline::InferenceEngineTVMConfig & config,
  const CenterPointConfig & config_mod)
: datatype_bytes(config.tvm_dtype_bits / 8), config_detail(config_mod)
{
  head_out_heatmap.resize(
    config.network_outputs[0].second[1] * config.network_outputs[0].second[2] *
    config.network_outputs[0].second[3]);
  head_out_offset.resize(
    config.network_outputs[1].second[1] * config.network_outputs[1].second[2] *
    config.network_outputs[1].second[3]);
  head_out_z.resize(
    config.network_outputs[2].second[1] * config.network_outputs[2].second[2] *
    config.network_outputs[2].second[3]);
  head_out_dim.resize(
    config.network_outputs[3].second[1] * config.network_outputs[3].second[2] *
    config.network_outputs[3].second[3]);
  head_out_rot.resize(
    config.network_outputs[4].second[1] * config.network_outputs[4].second[2] *
    config.network_outputs[4].second[3]);
  head_out_vel.resize(
    config.network_outputs[5].second[1] * config.network_outputs[5].second[2] *
    config.network_outputs[5].second[3]);
}

std::vector<Box3D> BackboneNeckHeadPostProcessor::schedule(const TVMArrayContainerVector & input)
{
  TVMArrayCopyToBytes(
    input[0].getArray(), head_out_heatmap.data(), head_out_heatmap.size() * datatype_bytes);
  TVMArrayCopyToBytes(
    input[1].getArray(), head_out_offset.data(), head_out_offset.size() * datatype_bytes);
  TVMArrayCopyToBytes(input[2].getArray(), head_out_z.data(), head_out_z.size() * datatype_bytes);
  TVMArrayCopyToBytes(
    input[3].getArray(), head_out_dim.data(), head_out_dim.size() * datatype_bytes);
  TVMArrayCopyToBytes(
    input[4].getArray(), head_out_rot.data(), head_out_rot.size() * datatype_bytes);
  TVMArrayCopyToBytes(
    input[5].getArray(), head_out_vel.data(), head_out_vel.size() * datatype_bytes);

  std::vector<Box3D> det_boxes3d;

  generateDetectedBoxes3D(
    head_out_heatmap, head_out_offset, head_out_z, head_out_dim, head_out_rot, head_out_vel,
    config_detail, det_boxes3d);

  return det_boxes3d;
}

CenterPointTVM::CenterPointTVM(
  const DensificationParam & densification_param, const CenterPointConfig & config)
: config_ve(config_en),
  config_bnh(config_bk),
  VE_PreP(std::make_shared<VE_PrePT>(config_en, config)),
  VE_IE(std::make_shared<IET>(config_en, "lidar_centerpoint_tvm")),
  VE_PostP(std::make_shared<VE_PostPT>(config_en)),
  ve_pipeline(std::make_shared<tvm_utility::pipeline::Pipeline<VE_PrePT, IET, VE_PostPT>>(
    *VE_PreP, *VE_IE, *VE_PostP)),
  BNH_PreP(std::make_shared<BNH_PrePT>(config_bk, config)),
  BNH_IE(std::make_shared<IET>(config_bk, "lidar_centerpoint_tvm")),
  BNH_PostP(std::make_shared<BNH_PostPT>(config_bk, config)),
  bnh_pipeline(std::make_shared<tvm_utility::pipeline::Pipeline<BNH_PrePT, IET, BNH_PostPT>>(
    *BNH_PreP, *BNH_IE, *BNH_PostP)),
  config_(config)
{
  vg_ptr_ = std::make_unique<VoxelGenerator>(densification_param, config_);
  initPtr();
}

void CenterPointTVM::initPtr()
{
  const auto voxels_size =
    config_.max_voxel_size_ * config_.max_point_in_voxel_size_ * config_.point_feature_size_;
  const auto coordinates_size = config_.max_voxel_size_ * config_.point_dim_size_;

  voxels_ = std::make_shared<std::vector<float>>(voxels_size);
  coordinates_ = std::make_shared<std::vector<int32_t>>(coordinates_size);
  num_points_per_voxel_ = std::make_shared<std::vector<float>>(config_.max_voxel_size_);
}

bool CenterPointTVM::detect(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer,
  std::vector<Box3D> & det_boxes3d)
{
  std::fill(voxels_->begin(), voxels_->end(), 0);
  std::fill(coordinates_->begin(), coordinates_->end(), -1);
  std::fill(num_points_per_voxel_->begin(), num_points_per_voxel_->end(), 0);

  if (!preprocess(input_pointcloud_msg, tf_buffer)) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("lidar_centerpoint"), "Fail to preprocess and skip to detect.");
    return false;
  }

  MixedInputs voxel_inputs{num_voxels_, *voxels_, *num_points_per_voxel_, *coordinates_};
  auto ve_output = ve_pipeline->schedule(voxel_inputs);

  MixedInputs pillar_inputs{num_voxels_, *ve_output, *num_points_per_voxel_, *coordinates_};
  auto bnh_output = bnh_pipeline->schedule(pillar_inputs);

  det_boxes3d = bnh_output;
  if (det_boxes3d.size() == 0) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("lidar_centerpoint_tvm"), "No detected boxes.");
  }

  return true;
}

bool CenterPointTVM::preprocess(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer)
{
  bool is_success = vg_ptr_->enqueuePointCloud(input_pointcloud_msg, tf_buffer);
  if (!is_success) {
    return false;
  }
  num_voxels_ = vg_ptr_->pointsToVoxels(*voxels_, *coordinates_, *num_points_per_voxel_);
  if (num_voxels_ == 0) {
    return false;
  }

  return true;
}

}  // namespace lidar_centerpoint_tvm
}  // namespace perception
}  // namespace autoware
