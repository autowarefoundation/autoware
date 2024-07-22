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

#ifndef AUTOWARE__LIDAR_TRANSFUSION__NETWORK__NETWORK_TRT_HPP_
#define AUTOWARE__LIDAR_TRANSFUSION__NETWORK__NETWORK_TRT_HPP_

#include "autoware/lidar_transfusion/transfusion_config.hpp"
#include "autoware/lidar_transfusion/utils.hpp"

#include <tensorrt_common/tensorrt_common.hpp>

#include <NvInfer.h>

#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::lidar_transfusion
{

struct ProfileDimension
{
  nvinfer1::Dims min;
  nvinfer1::Dims opt;
  nvinfer1::Dims max;

  bool operator!=(const ProfileDimension & rhs) const
  {
    return min.nbDims != rhs.min.nbDims || opt.nbDims != rhs.opt.nbDims ||
           max.nbDims != rhs.max.nbDims || !std::equal(min.d, min.d + min.nbDims, rhs.min.d) ||
           !std::equal(opt.d, opt.d + opt.nbDims, rhs.opt.d) ||
           !std::equal(max.d, max.d + max.nbDims, rhs.max.d);
  }
};

class NetworkTRT
{
public:
  explicit NetworkTRT(const TransfusionConfig & config);
  ~NetworkTRT();

  bool init(
    const std::string & onnx_path, const std::string & engine_path, const std::string & precision);
  const char * getTensorName(NetworkIO name);

  tensorrt_common::TrtUniquePtr<nvinfer1::ICudaEngine> engine{nullptr};
  tensorrt_common::TrtUniquePtr<nvinfer1::IExecutionContext> context{nullptr};

private:
  bool parseONNX(
    const std::string & onnx_path, const std::string & engine_path, const std::string & precision,
    size_t workspace_size = (1ULL << 30));
  bool saveEngine(const std::string & engine_path);
  bool loadEngine(const std::string & engine_path);
  bool createContext();
  bool setProfile(
    nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
    nvinfer1::IBuilderConfig & config);
  bool validateNetworkIO();
  nvinfer1::Dims validateTensorShape(NetworkIO name, const std::vector<int> shape);

  tensorrt_common::TrtUniquePtr<nvinfer1::IRuntime> runtime_{nullptr};
  tensorrt_common::TrtUniquePtr<nvinfer1::IHostMemory> plan_{nullptr};
  tensorrt_common::Logger logger_;
  TransfusionConfig config_;
  std::vector<const char *> tensors_names_;

  std::array<ProfileDimension, 3> in_profile_dims_;
};

}  // namespace autoware::lidar_transfusion

#endif  // AUTOWARE__LIDAR_TRANSFUSION__NETWORK__NETWORK_TRT_HPP_
