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

#ifndef LIDAR_CENTERPOINT__NETWORK__NETWORK_TRT_HPP_
#define LIDAR_CENTERPOINT__NETWORK__NETWORK_TRT_HPP_

#include <lidar_centerpoint/centerpoint_config.hpp>
#include <lidar_centerpoint/network/tensorrt_wrapper.hpp>

#include <vector>

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

  HeadTRT(const std::vector<std::size_t> & out_channel_sizes, const CenterPointConfig & config);

protected:
  bool setProfile(
    nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
    nvinfer1::IBuilderConfig & config) override;

  std::vector<std::size_t> out_channel_sizes_;
};

}  // namespace centerpoint

#endif  // LIDAR_CENTERPOINT__NETWORK__NETWORK_TRT_HPP_
