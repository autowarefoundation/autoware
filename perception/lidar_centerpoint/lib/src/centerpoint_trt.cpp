// Copyright 2021 Tier IV, Inc.
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
#include <heatmap_utils.hpp>
#include <tier4_autoware_utils/math/constants.hpp>

#include <ATen/cuda/CUDAContext.h>
#include <NvOnnxParser.h>
#include <c10/cuda/CUDAStream.h>
#include <torch/script.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#define DEBUG_NAN 0

namespace centerpoint
{
using torch::indexing::Slice;

CenterPointTRT::CenterPointTRT(
  const NetworkParam & encoder_param, const NetworkParam & head_param, const bool verbose)
{
  vg_ptr_ = std::make_unique<VoxelGenerator>();

  if (encoder_param.use_trt()) {
    encoder_trt_ptr_ = std::make_unique<VoxelEncoderTRT>(verbose);
    encoder_trt_ptr_->init(
      encoder_param.onnx_path(), encoder_param.engine_path(), encoder_param.trt_precision());
  } else {
    loadTorchScript(encoder_pt_, encoder_param.pt_path());
  }

  if (head_param.use_trt()) {
    head_trt_ptr_ = std::make_unique<HeadTRT>(verbose);
    head_trt_ptr_->init(
      head_param.onnx_path(), head_param.engine_path(), head_param.trt_precision());
    head_trt_ptr_->context_->setBindingDimensions(
      0, nvinfer1::Dims4(
           1, Config::num_encoder_output_features, Config::grid_size_y, Config::grid_size_x));
  } else {
    loadTorchScript(head_pt_, head_param.pt_path());
  }

  initPtr(encoder_param.use_trt(), head_param.use_trt());

  torch::set_num_threads(1);  // disable CPU parallelization

  cudaStreamCreate(&stream_);
}

CenterPointTRT::~CenterPointTRT()
{
  if (stream_) {
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
  }
}

bool CenterPointTRT::initPtr(const bool use_encoder_trt, const bool use_head_trt)
{
  if (use_encoder_trt) {
    output_pillar_feature_t_ = torch::zeros(
      {Config::max_num_voxels, Config::num_encoder_output_features},
      torch::TensorOptions().device(device_).dtype(torch::kFloat));
  }

  if (use_head_trt) {
    const int downsample_grid_x =
      static_cast<int>(static_cast<float>(Config::grid_size_x) / Config::downsample_factor);
    const int downsample_grid_y =
      static_cast<int>(static_cast<float>(Config::grid_size_y) / Config::downsample_factor);
    const int batch_size = 1;
    auto torch_options = torch::TensorOptions().device(device_).dtype(torch::kFloat);
    output_heatmap_t_ = torch::zeros(
      {batch_size, Config::num_class, downsample_grid_y, downsample_grid_x}, torch_options);
    output_offset_t_ = torch::zeros(
      {batch_size, Config::num_output_offset_features, downsample_grid_y, downsample_grid_x},
      torch_options);
    output_z_t_ = torch::zeros(
      {batch_size, Config::num_output_z_features, downsample_grid_y, downsample_grid_x},
      torch_options);
    output_dim_t_ = torch::zeros(
      {batch_size, Config::num_output_dim_features, downsample_grid_y, downsample_grid_x},
      torch_options);
    output_rot_t_ = torch::zeros(
      {batch_size, Config::num_output_rot_features, downsample_grid_y, downsample_grid_x},
      torch_options);
    output_vel_t_ = torch::zeros(
      {batch_size, Config::num_output_vel_features, downsample_grid_y, downsample_grid_x},
      torch_options);
  }

  return true;
}

std::vector<float> CenterPointTRT::detect(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg)
{
  voxels_t_ = torch::zeros(
    {Config::max_num_voxels, Config::max_num_points_per_voxel, Config::num_point_features},
    torch::TensorOptions().device(torch::kCPU).dtype(torch::kFloat));
  coordinates_t_ = torch::zeros(
    {Config::max_num_voxels, Config::num_point_dims},
    torch::TensorOptions().device(torch::kCPU).dtype(torch::kInt));
  num_points_per_voxel_t_ = torch::zeros(
    {Config::max_num_voxels}, torch::TensorOptions().device(torch::kCPU).dtype(torch::kInt));

  int num_voxels = vg_ptr_->pointsToVoxels(
    input_pointcloud_msg, voxels_t_, coordinates_t_, num_points_per_voxel_t_);
  // Note: unlike python implementation, no slicing by num_voxels
  //       .s.t voxels_t_ = voxels_t_[:num_voxels].
  //       w/ slicing more GPU memories are allocated

  voxels_t_ = voxels_t_.to(device_);
  coordinates_t_ = coordinates_t_.to(device_);
  num_points_per_voxel_t_ = num_points_per_voxel_t_.to(device_);
  at::Tensor input_features =
    createInputFeatures(voxels_t_, coordinates_t_, num_points_per_voxel_t_);

  // Note: num_voxels <= max_num_voxels, so input_features[num_voxels:] are invalid features.
  input_features.index_put_({Slice(num_voxels)}, 0);

  if (encoder_trt_ptr_ && encoder_trt_ptr_->context_) {
    std::vector<void *> encoder_buffers{
      input_features.data_ptr(), output_pillar_feature_t_.data_ptr()};
    encoder_trt_ptr_->context_->setBindingDimensions(
      0, nvinfer1::Dims3(
           Config::max_num_voxels, Config::max_num_points_per_voxel,
           Config::num_encoder_input_features));
    encoder_trt_ptr_->context_->enqueueV2(encoder_buffers.data(), stream_, nullptr);
  } else {
    std::vector<torch::jit::IValue> batch_input_features;
    batch_input_features.emplace_back(input_features);
    batch_input_features.emplace_back(num_points_per_voxel_t_);
    batch_input_features.emplace_back(coordinates_t_);
    {
      torch::NoGradGuard no_grad;
      output_pillar_feature_t_ = encoder_pt_.forward(batch_input_features).toTensor();
    }
  }

  at::Tensor spatial_features =
    scatterPillarFeatures(output_pillar_feature_t_, coordinates_t_.to(torch::kLong));

  if (head_trt_ptr_ && head_trt_ptr_->context_) {
    std::vector<void *> head_buffers = {spatial_features.data_ptr(), output_heatmap_t_.data_ptr(),
                                        output_offset_t_.data_ptr(), output_z_t_.data_ptr(),
                                        output_dim_t_.data_ptr(),    output_rot_t_.data_ptr(),
                                        output_vel_t_.data_ptr()};
    head_trt_ptr_->context_->enqueueV2(head_buffers.data(), stream_, nullptr);
  } else {
    std::vector<torch::jit::IValue> batch_spatial_features;
    batch_spatial_features.emplace_back(spatial_features);

    {
      torch::NoGradGuard no_grad;
      auto pred_arr = head_pt_.forward(batch_spatial_features).toTuple()->elements();
      output_heatmap_t_ = pred_arr[0].toTensor();
      output_offset_t_ = pred_arr[1].toTensor();
      output_z_t_ = pred_arr[2].toTensor();
      output_dim_t_ = pred_arr[3].toTensor();
      output_rot_t_ = pred_arr[4].toTensor();
      output_vel_t_ = pred_arr[5].toTensor();
    }
  }

  at::Tensor boxes3d = generatePredictedBoxes();
  std::vector<float> boxes3d_vec =
    std::vector<float>(boxes3d.data_ptr<float>(), boxes3d.data_ptr<float>() + boxes3d.numel());

  return boxes3d_vec;
}

at::Tensor CenterPointTRT::createInputFeatures(
  const at::Tensor & voxels, const at::Tensor & coords, const at::Tensor & voxel_num_points)
{
  // voxels (float): (num_pillars, num_max_points, num_point_features)
  // coordinates (int): (num_pillars, num_point_dims)
  // voxel_num_points (int): (num_pillars,)

  at::Tensor coords_f = coords.to(torch::kFloat);
  at::Tensor voxel_num_points_f = voxel_num_points.to(torch::kFloat);

  at::Tensor points_mean =
    voxels.slice(/*dim=*/2, /*start=*/0, /*end=*/3).sum({1}, /*keepdim=*/true) /
    voxel_num_points_f.view({-1, 1, 1});
  at::Tensor cluster = voxels.slice(2, 0, 3) - points_mean;

  // Note: unlike python implementation, batch_index isn't used in coords,
  at::Tensor center_x =
    voxels.slice(2, 0, 1) -
    (coords_f.slice(1, 2, 3).unsqueeze(2) * Config::voxel_size_x + Config::offset_x);
  at::Tensor center_y =
    voxels.slice(2, 1, 2) -
    (coords_f.slice(1, 1, 2).unsqueeze(2) * Config::voxel_size_y + Config::offset_y);
  at::Tensor input_features = torch::cat({voxels, cluster, center_x, center_y}, /*dim=*/2);

  // paddings_indicator
  const size_t axis = 0;
  const int voxel_cnt = input_features.sizes()[1];
  at::Tensor actual_num = voxel_num_points.unsqueeze(axis + 1);
  at::Tensor max_num =
    torch::arange(
      voxel_cnt, torch::TensorOptions().dtype(torch::kInt32).device(actual_num.device()))
      .view({1, -1});
  at::Tensor mask = actual_num.to(torch::kInt32) > max_num;
  mask = mask.unsqueeze(-1).to(torch::kFloat);
  input_features *= mask;

  return input_features;  // (num_pillars, num_max_points, num_voxel_features)
}

at::Tensor CenterPointTRT::scatterPillarFeatures(
  const at::Tensor & pillar_features, const at::Tensor & coordinates)
{
  // pillar_features (float): (num_pillars, num_encoder_output_features)
  // coordinates (float): (num_pillars, num_point_dims)

  at::Tensor spatial_feature = torch::zeros(
    {Config::num_encoder_output_features, Config::grid_size_y * Config::grid_size_x},
    torch::TensorOptions().dtype(pillar_features.dtype()).device(pillar_features.device()));
  auto index = coordinates.select(1, 1) * Config::grid_size_x + coordinates.select(1, 2);
  spatial_feature.index_put_({"...", index}, pillar_features.t());

  return spatial_feature.view({1 /*batch size*/, -1, Config::grid_size_y, Config::grid_size_x})
    .contiguous();
}

at::Tensor CenterPointTRT::generatePredictedBoxes()
{
  // output_heatmap (float): (batch_size, num_class, H, W)
  // output_offset (float): (batch_size, num_offset_features, H, W)
  // output_z (float): (batch_size, num_z_features, H, W)
  // output_dim (float): (batch_size, num_dim_features, H, W)
  // output_rot (float): (batch_size, num_rot_features, H, W)
  // output_vel (float): (batch_size, num_vel_features, H, W)

  at::Tensor heatmap_pred = output_heatmap_t_.clone();
  heatmap_pred = sigmoid_hm(heatmap_pred);
  heatmap_pred = nms_hm(heatmap_pred);

  auto topk_tuple = select_topk(heatmap_pred, Config::max_num_output_objects);
  at::Tensor scores = std::get<0>(topk_tuple);
  at::Tensor index = std::get<1>(topk_tuple);
  at::Tensor classes = std::get<2>(topk_tuple);
  at::Tensor ys = std::get<3>(topk_tuple);
  at::Tensor xs = std::get<4>(topk_tuple);

  at::Tensor offset_poi = select_point_of_interest(index, output_offset_t_);
  at::Tensor z_poi = select_point_of_interest(index, output_z_t_);
  at::Tensor dim_poi = select_point_of_interest(index, output_dim_t_);
  at::Tensor rot_poi = select_point_of_interest(index, output_rot_t_);
  at::Tensor vel_poi = select_point_of_interest(index, output_vel_t_);

  at::Tensor x = Config::voxel_size_x * Config::downsample_factor *
                   (xs.view({1, -1, 1}) + offset_poi.slice(2, 0, 1)) +
                 Config::pointcloud_range_xmin;
  at::Tensor y = Config::voxel_size_y * Config::downsample_factor *
                   (ys.view({1, -1, 1}) + offset_poi.slice(2, 1, 2)) +
                 Config::pointcloud_range_ymin;
  dim_poi = torch::exp(dim_poi);
  at::Tensor rot = torch::atan2(rot_poi.slice(2, 0, 1), rot_poi.slice(2, 1, 2));
  rot = -rot - tier4_autoware_utils::pi / 2;

  at::Tensor boxes3d =
    torch::cat(
      {scores.view({1, -1, 1}), classes.view({1, -1, 1}), x, y, z_poi, dim_poi, rot, vel_poi},
      /*dim=*/2)
      .contiguous()
      .to(torch::kCPU)
      .to(torch::kFloat);

  return boxes3d;
}

bool CenterPointTRT::loadTorchScript(
  torch::jit::script::Module & module, const std::string & model_path)
{
  try {
    module = torch::jit::load(model_path, device_);
    module.eval();
  } catch (const c10::Error & e) {
    std::cout << "LOADING ERROR: " << e.msg() << std::endl;
    return false;
  }
  std::cout << "Loading from " << model_path << std::endl;
  return true;
}

}  // namespace centerpoint
