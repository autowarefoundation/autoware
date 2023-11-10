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

#ifndef PERCEPTION__LIDAR_CENTERPOINT_TVM__DATA__MODELS__CENTERPOINT_ENCODER__INFERENCE_ENGINE_TVM_CONFIG_HPP_  // NOLINT
#define PERCEPTION__LIDAR_CENTERPOINT_TVM__DATA__MODELS__CENTERPOINT_ENCODER__INFERENCE_ENGINE_TVM_CONFIG_HPP_  // NOLINT

namespace model_zoo
{
namespace perception
{
namespace lidar_obstacle_detection
{
namespace centerpoint_encoder
{
namespace onnx_centerpoint_encoder
{

static const tvm_utility::pipeline::InferenceEngineTVMConfig config{
  {3, 0, 0},  // modelzoo_version

  "centerpoint_encoder",  // network_name
  "llvm",                 // network_backend

  "./deploy_lib.so",        // network_module_path
  "./deploy_graph.json",    // network_graph_path
  "./deploy_param.params",  // network_params_path

  kDLCPU,  // tvm_device_type
  0,       // tvm_device_id

  {{"input_features", kDLFloat, 32, 1, {40000, 32, 9}}},  // network_inputs

  {{"pillar_features", kDLFloat, 32, 1, {40000, 1, 32}}}  // network_outputs
};

}  // namespace onnx_centerpoint_encoder
}  // namespace centerpoint_encoder
}  // namespace lidar_obstacle_detection
}  // namespace perception
}  // namespace model_zoo
// NOLINTNEXTLINE
#endif  // PERCEPTION__LIDAR_CENTERPOINT_TVM__DATA__MODELS__CENTERPOINT_ENCODER__INFERENCE_ENGINE_TVM_CONFIG_HPP_
