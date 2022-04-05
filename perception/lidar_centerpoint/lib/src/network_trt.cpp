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

#include <config.hpp>
#include <network_trt.hpp>

namespace centerpoint
{
bool VoxelEncoderTRT::setProfile(
  nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
  nvinfer1::IBuilderConfig & config)
{
  auto profile = builder.createOptimizationProfile();
  auto in_name = network.getInput(0)->getName();
  auto in_dims = nvinfer1::Dims3(
    Config::max_num_voxels, Config::max_num_points_per_voxel, Config::encoder_in_feature_size);
  profile->setDimensions(in_name, nvinfer1::OptProfileSelector::kMIN, in_dims);
  profile->setDimensions(in_name, nvinfer1::OptProfileSelector::kOPT, in_dims);
  profile->setDimensions(in_name, nvinfer1::OptProfileSelector::kMAX, in_dims);

  auto out_name = network.getOutput(0)->getName();
  auto out_dims = nvinfer1::Dims2(Config::max_num_voxels, Config::encoder_out_feature_size);
  profile->setDimensions(out_name, nvinfer1::OptProfileSelector::kMIN, out_dims);
  profile->setDimensions(out_name, nvinfer1::OptProfileSelector::kOPT, out_dims);
  profile->setDimensions(out_name, nvinfer1::OptProfileSelector::kMAX, out_dims);
  config.addOptimizationProfile(profile);

  return true;
}

HeadTRT::HeadTRT(const std::size_t num_class, const bool verbose)
: TensorRTWrapper(verbose), num_class_(num_class)
{
}

bool HeadTRT::setProfile(
  nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
  nvinfer1::IBuilderConfig & config)
{
  auto profile = builder.createOptimizationProfile();
  auto in_name = network.getInput(0)->getName();
  auto in_dims = nvinfer1::Dims4(
    Config::batch_size, Config::encoder_out_feature_size, Config::grid_size_y, Config::grid_size_x);
  profile->setDimensions(in_name, nvinfer1::OptProfileSelector::kMIN, in_dims);
  profile->setDimensions(in_name, nvinfer1::OptProfileSelector::kOPT, in_dims);
  profile->setDimensions(in_name, nvinfer1::OptProfileSelector::kMAX, in_dims);

  std::array<std::size_t, Config::head_out_size> output_channels = {
    num_class_,
    Config::head_out_offset_size,
    Config::head_out_z_size,
    Config::head_out_dim_size,
    Config::head_out_rot_size,
    Config::head_out_vel_size};
  for (std::size_t ci = 0; ci < Config::head_out_size; ci++) {
    auto out_name = network.getOutput(ci)->getName();
    auto out_dims = nvinfer1::Dims4(
      Config::batch_size, output_channels[ci], Config::down_grid_size_x, Config::down_grid_size_y);
    profile->setDimensions(out_name, nvinfer1::OptProfileSelector::kMIN, out_dims);
    profile->setDimensions(out_name, nvinfer1::OptProfileSelector::kOPT, out_dims);
    profile->setDimensions(out_name, nvinfer1::OptProfileSelector::kMAX, out_dims);
  }
  config.addOptimizationProfile(profile);

  return true;
}

}  // namespace centerpoint
