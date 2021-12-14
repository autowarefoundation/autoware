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

#ifndef CENTERPOINT_TRT_HPP_
#define CENTERPOINT_TRT_HPP_

#include <config.hpp>
#include <cuda_utils.hpp>
#include <network_trt.hpp>
#include <tier4_autoware_utils/math/constants.hpp>
#include <voxel_generator.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <torch/script.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace centerpoint
{
class NetworkParam
{
public:
  NetworkParam(
    std::string onnx_path, std::string engine_path, std::string pt_path, std::string trt_precision,
    const bool use_trt)
  : onnx_path_(std::move(onnx_path)),
    engine_path_(std::move(engine_path)),
    pt_path_(std::move(pt_path)),
    trt_precision_(std::move(trt_precision)),
    use_trt_(use_trt)
  {
  }

  std::string onnx_path() const { return onnx_path_; }
  std::string engine_path() const { return engine_path_; }
  std::string pt_path() const { return pt_path_; }
  std::string trt_precision() const { return trt_precision_; }
  bool use_trt() const { return use_trt_; }

private:
  std::string onnx_path_;
  std::string engine_path_;
  std::string pt_path_;
  std::string trt_precision_;
  bool use_trt_;
};

class CenterPointTRT
{
public:
  explicit CenterPointTRT(
    const NetworkParam & encoder_param, const NetworkParam & head_param, bool verbose);

  ~CenterPointTRT();

  std::vector<float> detect(const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg);

private:
  bool initPtr(bool use_encoder_trt, bool use_head_trt);

  bool loadTorchScript(torch::jit::script::Module & module, const std::string & model_path);

  static at::Tensor createInputFeatures(
    const at::Tensor & voxels, const at::Tensor & coords, const at::Tensor & voxel_num_points);

  static at::Tensor scatterPillarFeatures(
    const at::Tensor & pillar_features, const at::Tensor & coordinates);

  at::Tensor generatePredictedBoxes();

  std::unique_ptr<VoxelGeneratorTemplate> vg_ptr_ = nullptr;
  torch::jit::script::Module encoder_pt_;
  torch::jit::script::Module head_pt_;
  std::unique_ptr<VoxelEncoderTRT> encoder_trt_ptr_ = nullptr;
  std::unique_ptr<HeadTRT> head_trt_ptr_ = nullptr;
  c10::Device device_ = torch::kCUDA;
  cudaStream_t stream_ = nullptr;

  at::Tensor voxels_t_;
  at::Tensor coordinates_t_;
  at::Tensor num_points_per_voxel_t_;
  at::Tensor output_pillar_feature_t_;
  at::Tensor output_heatmap_t_;
  at::Tensor output_offset_t_;
  at::Tensor output_z_t_;
  at::Tensor output_dim_t_;
  at::Tensor output_rot_t_;
  at::Tensor output_vel_t_;
};

}  // namespace centerpoint

#endif  // CENTERPOINT_TRT_HPP_
