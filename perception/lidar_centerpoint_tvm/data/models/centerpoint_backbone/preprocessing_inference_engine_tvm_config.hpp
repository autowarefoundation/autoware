// Copyright 2021 Arm Limited and Contributors.
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

#include "tvm_utility/pipeline.hpp"

#ifndef PERCEPTION__LIDAR_CENTERPOINT_TVM__DATA__MODELS__CENTERPOINT_BACKBONE__PREPROCESSING_INFERENCE_ENGINE_TVM_CONFIG_HPP_  // NOLINT
#define PERCEPTION__LIDAR_CENTERPOINT_TVM__DATA__MODELS__CENTERPOINT_BACKBONE__PREPROCESSING_INFERENCE_ENGINE_TVM_CONFIG_HPP_  // NOLINT

namespace model_zoo
{
namespace perception
{
namespace lidar_obstacle_detection
{
namespace centerpoint_backbone
{
namespace onnx_centerpoint_backbone
{
namespace preprocessing
{

static const tvm_utility::pipeline::InferenceEngineTVMConfig config{
  {3, 0, 0},  // modelzoo_version

  "centerpoint_backbone",  // network_name
  "llvm",                  // network_backend

  "./preprocess.so",  // network_module_path
  "./",               // network_graph_path
  "./",               // network_params_path

  kDLCPU,  // tvm_device_type
  0,       // tvm_device_id

  {{"pillar_features", kDLFloat, 32, 1, {40000, 1, 32}},
   {"coords", kDLInt, 32, 1, {40000, 3}}},  // network_inputs

  {{"spatial_features", kDLFloat, 32, 1, {1, 32, 560, 560}}}  // network_outputs
};

}  // namespace preprocessing
}  // namespace onnx_centerpoint_backbone
}  // namespace centerpoint_backbone
}  // namespace lidar_obstacle_detection
}  // namespace perception
}  // namespace model_zoo
// NOLINTNEXTLINE
#endif  // PERCEPTION__LIDAR_CENTERPOINT_TVM__DATA__MODELS__CENTERPOINT_BACKBONE__PREPROCESSING_INFERENCE_ENGINE_TVM_CONFIG_HPP_
