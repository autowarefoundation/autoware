// Copyright 2020-2023 TIER IV, Inc.
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

#ifndef AUTOWARE__LIDAR_APOLLO_INSTANCE_SEGMENTATION__FEATURE_MAP_HPP_
#define AUTOWARE__LIDAR_APOLLO_INSTANCE_SEGMENTATION__FEATURE_MAP_HPP_

#include <memory>
#include <vector>

namespace autoware
{
namespace lidar_apollo_instance_segmentation
{
struct FeatureMapInterface
{
public:
  int channels;
  int width;
  int height;
  int range;
  float * max_height_data;      // channel 0
  float * mean_height_data;     // channel 1
  float * count_data;           // channel 2
  float * direction_data;       // channel 3
  float * top_intensity_data;   // channel 4
  float * mean_intensity_data;  // channel 5
  float * distance_data;        // channel 6
  float * nonempty_data;        // channel 7
  std::vector<float> map_data;
  virtual void initializeMap(std::vector<float> & map) = 0;
  virtual void resetMap(std::vector<float> & map) = 0;
  FeatureMapInterface(const int _channels, const int _width, const int _height, const int _range);
};

struct FeatureMap : public FeatureMapInterface
{
  FeatureMap(const int width, const int height, const int range);
  void initializeMap(std::vector<float> & map) override;
  void resetMap(std::vector<float> & map) override;
};

struct FeatureMapWithIntensity : public FeatureMapInterface
{
  FeatureMapWithIntensity(const int width, const int height, const int range);
  void initializeMap(std::vector<float> & map) override;
  void resetMap(std::vector<float> & map) override;
};

struct FeatureMapWithConstant : public FeatureMapInterface
{
  FeatureMapWithConstant(const int width, const int height, const int range);
  void initializeMap(std::vector<float> & map) override;
  void resetMap(std::vector<float> & map) override;
};

struct FeatureMapWithConstantAndIntensity : public FeatureMapInterface
{
  FeatureMapWithConstantAndIntensity(const int width, const int height, const int range);
  void initializeMap(std::vector<float> & map) override;
  void resetMap(std::vector<float> & map) override;
};
}  // namespace lidar_apollo_instance_segmentation
}  // namespace autoware
#endif  // AUTOWARE__LIDAR_APOLLO_INSTANCE_SEGMENTATION__FEATURE_MAP_HPP_
