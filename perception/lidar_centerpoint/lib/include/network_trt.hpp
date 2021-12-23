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

#ifndef NETWORK_TRT_HPP_
#define NETWORK_TRT_HPP_

#include <tensorrt_wrapper.hpp>

namespace centerpoint
{
class VoxelEncoderTRT : public TensorRTWrapper
{
public:
  using TensorRTWrapper::TensorRTWrapper;

protected:
  bool setProfile(
    nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
    nvinfer1::IBuilderConfig & config) override;
};

class HeadTRT : public TensorRTWrapper
{
public:
  using TensorRTWrapper::TensorRTWrapper;

  HeadTRT(const int num_class, const bool verbose);

protected:
  bool setProfile(
    nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
    nvinfer1::IBuilderConfig & config) override;

  int num_class_{0};
};

}  // namespace centerpoint

#endif  // NETWORK_TRT_HPP_
