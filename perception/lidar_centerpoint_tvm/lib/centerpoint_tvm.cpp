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
#include <centerpoint_backbone/preprocessing_inference_engine_tvm_config.hpp>
#include <centerpoint_encoder/inference_engine_tvm_config.hpp>
#include <lidar_centerpoint_tvm/centerpoint_config.hpp>
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
auto config_scatter = model_zoo::perception::lidar_obstacle_detection::centerpoint_backbone::
  onnx_centerpoint_backbone::preprocessing::config;

namespace autoware
{
namespace perception
{
namespace lidar_centerpoint_tvm
{

TVMScatterIE::TVMScatterIE(
  tvm_utility::pipeline::InferenceEngineTVMConfig config, const std::string & pkg_name,
  const std::string & data_path, const std::string & function_name)
: config_(config)
{
  std::string network_prefix = data_path + "/" + pkg_name + "/models/" + config.network_name + "/";
  std::string network_module_path = network_prefix + config.network_module_path;

  std::ifstream module(network_module_path);
  if (!module.good()) {
    throw std::runtime_error("File " + network_module_path + " specified in config.hpp not found");
  }
  module.close();
  tvm::runtime::Module runtime_mod = tvm::runtime::Module::LoadFromFile(network_module_path);

  scatter_function = runtime_mod.GetFunction(function_name);

  for (auto & output_config : config.network_outputs) {
    output_.push_back(TVMArrayContainer(
      output_config.node_shape, output_config.tvm_dtype_code, output_config.tvm_dtype_bits,
      output_config.tvm_dtype_lanes, config.tvm_device_type, config.tvm_device_id));
  }
}

TVMArrayContainerVector TVMScatterIE::schedule(const TVMArrayContainerVector & input)
{
  scatter_function(input[0].getArray(), coords_.getArray(), output_[0].getArray());

  return output_;
}

VoxelEncoderPreProcessor::VoxelEncoderPreProcessor(
  const tvm_utility::pipeline::InferenceEngineTVMConfig & config,
  const CenterPointConfig & config_mod)
: max_voxel_size(config.network_inputs[0].node_shape[0]),
  max_point_in_voxel_size(config.network_inputs[0].node_shape[1]),
  encoder_in_feature_size(config.network_inputs[0].node_shape[2]),
  input_datatype_bytes(config.network_inputs[0].tvm_dtype_bits / 8),
  config_detail(config_mod)
{
  encoder_in_features.resize(max_voxel_size * max_point_in_voxel_size * encoder_in_feature_size);
  // Allocate input variable
  std::vector<int64_t> shape_x{1, max_voxel_size, max_point_in_voxel_size, encoder_in_feature_size};
  TVMArrayContainer x{
    shape_x,
    config.network_inputs[0].tvm_dtype_code,
    config.network_inputs[0].tvm_dtype_bits,
    config.network_inputs[0].tvm_dtype_lanes,
    config.tvm_device_type,
    config.tvm_device_id};
  output = x;
}

TVMArrayContainerVector VoxelEncoderPreProcessor::schedule(const MixedInputs & voxel_inputs)
{
  // generate encoder_in_features from the voxels
  generateFeatures(
    voxel_inputs.features, voxel_inputs.num_points_per_voxel, voxel_inputs.coords,
    voxel_inputs.num_voxels, config_detail, encoder_in_features);

  TVMArrayCopyFromBytes(
    output.getArray(), encoder_in_features.data(),
    max_voxel_size * max_point_in_voxel_size * encoder_in_feature_size * input_datatype_bytes);

  return {output};
}

BackboneNeckHeadPostProcessor::BackboneNeckHeadPostProcessor(
  const tvm_utility::pipeline::InferenceEngineTVMConfig & config,
  const CenterPointConfig & config_mod)
: output_datatype_bytes(config.network_outputs[0].tvm_dtype_bits / 8), config_detail(config_mod)
{
  head_out_heatmap.resize(
    config.network_outputs[0].node_shape[1] * config.network_outputs[0].node_shape[2] *
    config.network_outputs[0].node_shape[3]);
  head_out_offset.resize(
    config.network_outputs[1].node_shape[1] * config.network_outputs[1].node_shape[2] *
    config.network_outputs[1].node_shape[3]);
  head_out_z.resize(
    config.network_outputs[2].node_shape[1] * config.network_outputs[2].node_shape[2] *
    config.network_outputs[2].node_shape[3]);
  head_out_dim.resize(
    config.network_outputs[3].node_shape[1] * config.network_outputs[3].node_shape[2] *
    config.network_outputs[3].node_shape[3]);
  head_out_rot.resize(
    config.network_outputs[4].node_shape[1] * config.network_outputs[4].node_shape[2] *
    config.network_outputs[4].node_shape[3]);
  head_out_vel.resize(
    config.network_outputs[5].node_shape[1] * config.network_outputs[5].node_shape[2] *
    config.network_outputs[5].node_shape[3]);
}

std::vector<Box3D> BackboneNeckHeadPostProcessor::schedule(const TVMArrayContainerVector & input)
{
  TVMArrayCopyToBytes(
    input[0].getArray(), head_out_heatmap.data(), head_out_heatmap.size() * output_datatype_bytes);
  TVMArrayCopyToBytes(
    input[1].getArray(), head_out_offset.data(), head_out_offset.size() * output_datatype_bytes);
  TVMArrayCopyToBytes(
    input[2].getArray(), head_out_z.data(), head_out_z.size() * output_datatype_bytes);
  TVMArrayCopyToBytes(
    input[3].getArray(), head_out_dim.data(), head_out_dim.size() * output_datatype_bytes);
  TVMArrayCopyToBytes(
    input[4].getArray(), head_out_rot.data(), head_out_rot.size() * output_datatype_bytes);
  TVMArrayCopyToBytes(
    input[5].getArray(), head_out_vel.data(), head_out_vel.size() * output_datatype_bytes);

  std::vector<Box3D> det_boxes3d;

  generateDetectedBoxes3D(
    head_out_heatmap, head_out_offset, head_out_z, head_out_dim, head_out_rot, head_out_vel,
    config_detail, det_boxes3d);

  return det_boxes3d;
}

CenterPointTVM::CenterPointTVM(
  const DensificationParam & densification_param, const CenterPointConfig & config,
  const std::string & data_path)
: config_ve(config_en),
  config_bnh(config_bk),
  VE_PreP(std::make_shared<VE_PrePT>(config_en, config)),
  VE_IE(std::make_shared<IET>(config_en, "lidar_centerpoint_tvm", data_path)),
  BNH_IE(std::make_shared<IET>(config_bk, "lidar_centerpoint_tvm", data_path)),
  BNH_PostP(std::make_shared<BNH_PostPT>(config_bk, config)),
  scatter_ie(std::make_shared<TSE>(config_scatter, "lidar_centerpoint_tvm", data_path, "scatter")),
  TSP_pipeline(std::make_shared<TSP>(VE_PreP, VE_IE, scatter_ie, BNH_IE, BNH_PostP)),
  config_(config)
{
  vg_ptr_ = std::make_unique<VoxelGenerator>(densification_param, config_);
  initPtr();
}

CenterPointTVM::~CenterPointTVM()
{
}

void CenterPointTVM::initPtr()
{
  const auto voxels_size =
    config_.max_voxel_size_ * config_.max_point_in_voxel_size_ * config_.point_feature_size_;
  const auto coordinates_size = config_.max_voxel_size_ * config_.point_dim_size_;

  voxels_ = std::make_shared<std::vector<float>>(voxels_size);
  coordinates_ = std::make_shared<std::vector<int32_t>>(coordinates_size);
  num_points_per_voxel_ = std::make_shared<std::vector<float>>(config_.max_voxel_size_);
  std::vector<int64_t> shape_coords{
    config_scatter.network_inputs[1].node_shape[0], config_scatter.network_inputs[1].node_shape[1]};
  coords_tvm_ = TVMArrayContainer(
    shape_coords, config_scatter.network_inputs[1].tvm_dtype_code,
    config_scatter.network_inputs[1].tvm_dtype_bits,
    config_scatter.network_inputs[1].tvm_dtype_lanes, config_scatter.tvm_device_type,
    config_scatter.tvm_device_id);
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
  auto bnh_output = TSP_pipeline->schedule(voxel_inputs);

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

  TVMArrayCopyFromBytes(
    coords_tvm_.getArray(), coordinates_->data(), coordinates_->size() * sizeof(int32_t));

  scatter_ie->set_coords(coords_tvm_);

  return true;
}

}  // namespace lidar_centerpoint_tvm
}  // namespace perception
}  // namespace autoware
