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

#include "lidar_centerpoint/network/network_trt.hpp"

namespace centerpoint
{
bool VoxelEncoderTRT::setProfile(
  nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
  nvinfer1::IBuilderConfig & config)
{
  auto profile = builder.createOptimizationProfile();
  auto in_name = network.getInput(0)->getName();
  auto in_dims = nvinfer1::Dims3(
    config_.max_voxel_size_, config_.max_point_in_voxel_size_, config_.encoder_in_feature_size_);
  profile->setDimensions(in_name, nvinfer1::OptProfileSelector::kMIN, in_dims);
  profile->setDimensions(in_name, nvinfer1::OptProfileSelector::kOPT, in_dims);
  profile->setDimensions(in_name, nvinfer1::OptProfileSelector::kMAX, in_dims);

  auto out_name = network.getOutput(0)->getName();
  auto out_dims = nvinfer1::Dims2(config_.max_voxel_size_, config_.encoder_out_feature_size_);
  profile->setDimensions(out_name, nvinfer1::OptProfileSelector::kMIN, out_dims);
  profile->setDimensions(out_name, nvinfer1::OptProfileSelector::kOPT, out_dims);
  profile->setDimensions(out_name, nvinfer1::OptProfileSelector::kMAX, out_dims);
  config.addOptimizationProfile(profile);

  return true;
}

HeadTRT::HeadTRT(
  const std::vector<std::size_t> & out_channel_sizes, const CenterPointConfig & config,
  const bool verbose)
: TensorRTWrapper(config, verbose), out_channel_sizes_(out_channel_sizes)
{
}

bool HeadTRT::setProfile(
  nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
  nvinfer1::IBuilderConfig & config)
{
  auto profile = builder.createOptimizationProfile();
  auto in_name = network.getInput(0)->getName();
  auto in_dims = nvinfer1::Dims4(
    config_.batch_size_, config_.encoder_out_feature_size_, config_.grid_size_y_,
    config_.grid_size_x_);
  profile->setDimensions(in_name, nvinfer1::OptProfileSelector::kMIN, in_dims);
  profile->setDimensions(in_name, nvinfer1::OptProfileSelector::kOPT, in_dims);
  profile->setDimensions(in_name, nvinfer1::OptProfileSelector::kMAX, in_dims);

  for (std::size_t ci = 0; ci < out_channel_sizes_.size(); ci++) {
    auto out_name = network.getOutput(ci)->getName();
    auto out_dims = nvinfer1::Dims4(
      config_.batch_size_, out_channel_sizes_[ci], config_.down_grid_size_y_,
      config_.down_grid_size_x_);
    profile->setDimensions(out_name, nvinfer1::OptProfileSelector::kMIN, out_dims);
    profile->setDimensions(out_name, nvinfer1::OptProfileSelector::kOPT, out_dims);
    profile->setDimensions(out_name, nvinfer1::OptProfileSelector::kMAX, out_dims);
  }
  config.addOptimizationProfile(profile);

  return true;
}

}  // namespace centerpoint
