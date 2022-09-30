// Copyright 2020-2022 Arm Ltd., TierIV
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

#ifndef LIDAR_APOLLO_SEGMENTATION_TVM__FEATURE_MAP_HPP_
#define LIDAR_APOLLO_SEGMENTATION_TVM__FEATURE_MAP_HPP_

#include <common/types.hpp>

#include <memory>
#include <vector>

namespace autoware
{
namespace perception
{
namespace lidar_apollo_segmentation_tvm
{
using autoware::common::types::float32_t;

/// \brief Abstract interface for FeatureMap.
struct FeatureMapInterface
{
public:
  const int32_t channels;
  const int32_t width;
  const int32_t height;
  const int32_t range;
  float32_t * max_height_data;      // channel 0
  float32_t * mean_height_data;     // channel 1
  float32_t * count_data;           // channel 2
  float32_t * direction_data;       // channel 3
  float32_t * top_intensity_data;   // channel 4
  float32_t * mean_intensity_data;  // channel 5
  float32_t * distance_data;        // channel 6
  float32_t * nonempty_data;        // channel 7
  std::vector<float32_t> map_data;
  virtual void initializeMap(std::vector<float32_t> & map) = 0;
  virtual void resetMap(std::vector<float32_t> & map) = 0;
  explicit FeatureMapInterface(int32_t _channels, int32_t _width, int32_t _height, int32_t _range);
};

/// \brief FeatureMap with no extra feature channels.
struct FeatureMap : public FeatureMapInterface
{
  explicit FeatureMap(int32_t width, int32_t height, int32_t range);
  void initializeMap(std::vector<float32_t> & map) override;
  void resetMap(std::vector<float32_t> & map) override;
};

/// \brief FeatureMap with an intensity feature channel.
struct FeatureMapWithIntensity : public FeatureMapInterface
{
  explicit FeatureMapWithIntensity(int32_t width, int32_t height, int32_t range);
  void initializeMap(std::vector<float32_t> & map) override;
  void resetMap(std::vector<float32_t> & map) override;
};

/// \brief FeatureMap with a constant feature channel.
struct FeatureMapWithConstant : public FeatureMapInterface
{
  explicit FeatureMapWithConstant(int32_t width, int32_t height, int32_t range);
  void initializeMap(std::vector<float32_t> & map) override;
  void resetMap(std::vector<float32_t> & map) override;
};

/// \brief FeatureMap with constant and intensity feature channels.
struct FeatureMapWithConstantAndIntensity : public FeatureMapInterface
{
  explicit FeatureMapWithConstantAndIntensity(int32_t width, int32_t height, int32_t range);
  void initializeMap(std::vector<float32_t> & map) override;
  void resetMap(std::vector<float32_t> & map) override;
};
}  // namespace lidar_apollo_segmentation_tvm
}  // namespace perception
}  // namespace autoware
#endif  // LIDAR_APOLLO_SEGMENTATION_TVM__FEATURE_MAP_HPP_
