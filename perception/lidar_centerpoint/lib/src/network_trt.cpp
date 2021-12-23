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

#include <config.hpp>
#include <network_trt.hpp>

namespace centerpoint
{
bool VoxelEncoderTRT::setProfile(
  nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
  nvinfer1::IBuilderConfig & config)
{
  auto profile = builder.createOptimizationProfile();
  auto input_name = network.getInput(0)->getName();
  auto in_dims = nvinfer1::Dims3(
    Config::max_num_voxels, Config::max_num_points_per_voxel, Config::num_encoder_input_features);
  profile->setDimensions(input_name, nvinfer1::OptProfileSelector::kMIN, in_dims);
  profile->setDimensions(input_name, nvinfer1::OptProfileSelector::kOPT, in_dims);
  profile->setDimensions(input_name, nvinfer1::OptProfileSelector::kMAX, in_dims);
  auto output_name = network.getOutput(0)->getName();
  auto out_dims = nvinfer1::Dims2(Config::max_num_voxels, Config::num_encoder_output_features);
  profile->setDimensions(output_name, nvinfer1::OptProfileSelector::kMIN, out_dims);
  profile->setDimensions(output_name, nvinfer1::OptProfileSelector::kOPT, out_dims);
  profile->setDimensions(output_name, nvinfer1::OptProfileSelector::kMAX, out_dims);
  config.addOptimizationProfile(profile);

  return true;
}

HeadTRT::HeadTRT(const int num_class, const bool verbose)
: TensorRTWrapper(verbose), num_class_(num_class)
{
}

bool HeadTRT::setProfile(
  nvinfer1::IBuilder & builder, nvinfer1::INetworkDefinition & network,
  nvinfer1::IBuilderConfig & config)
{
  auto profile = builder.createOptimizationProfile();
  auto input_name = network.getInput(0)->getName();
  auto in_dims = nvinfer1::Dims4(
    Config::batch_size, Config::num_encoder_output_features, Config::grid_size_y,
    Config::grid_size_x);
  profile->setDimensions(input_name, nvinfer1::OptProfileSelector::kMIN, in_dims);
  profile->setDimensions(input_name, nvinfer1::OptProfileSelector::kOPT, in_dims);
  profile->setDimensions(input_name, nvinfer1::OptProfileSelector::kMAX, in_dims);

  std::array<int, Config::num_output_features> output_channels = {
    num_class_,
    Config::num_output_offset_features,
    Config::num_output_z_features,
    Config::num_output_dim_features,
    Config::num_output_rot_features,
    Config::num_output_vel_features};
  const int downsample_grid_y =
    static_cast<int>(static_cast<float>(Config::grid_size_y) / Config::downsample_factor);
  const int downsample_grid_x =
    static_cast<int>(static_cast<float>(Config::grid_size_x) / Config::downsample_factor);
  for (int ci = 0; ci < Config::num_output_features; ci++) {
    auto output_name = network.getOutput(ci)->getName();
    auto out_dims = nvinfer1::Dims4(1, output_channels[ci], downsample_grid_y, downsample_grid_x);
    profile->setDimensions(output_name, nvinfer1::OptProfileSelector::kMIN, out_dims);
    profile->setDimensions(output_name, nvinfer1::OptProfileSelector::kOPT, out_dims);
    profile->setDimensions(output_name, nvinfer1::OptProfileSelector::kMAX, out_dims);
  }
  config.addOptimizationProfile(profile);
  return true;
}

}  // namespace centerpoint
