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

#ifndef COMMON__TVM_UTILITY__DATA__MODELS__ABS_MODEL_X86_64__INFERENCE_ENGINE_TVM_CONFIG_HPP_  // NOLINT
#define COMMON__TVM_UTILITY__DATA__MODELS__ABS_MODEL_X86_64__INFERENCE_ENGINE_TVM_CONFIG_HPP_

namespace model_zoo
{
namespace inf_test
{
namespace engine_load
{
namespace abs_model
{

static const tvm_utility::pipeline::InferenceEngineTVMConfig config{
  {0, 0, 0},  // modelzoo_version

  "abs_model_x86_64",  // network_name
  "llvm",              // network_backend

  "deploy_lib.so",        // network_module_path
  "deploy_graph.json",    // network_graph_path
  "deploy_param.params",  // network_params_path

  // cspell: ignore DLCPU
  kDLCPU,  // tvm_device_type
  0,       // tvm_device_id

  {{"a", kDLFloat, 32, 1, {2, 2}}},  // network_inputs

  {{"output", kDLFloat, 32, 1, {2, 2}}}  // network_outputs
};

}  // namespace abs_model
}  // namespace engine_load
}  // namespace inf_test
}  // namespace model_zoo
#endif  // COMMON__TVM_UTILITY__DATA__MODELS__ABS_MODEL_X86_64__INFERENCE_ENGINE_TVM_CONFIG_HPP_
        // NOLINT
