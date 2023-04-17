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

#include <lidar_apollo_segmentation_tvm/feature_map.hpp>
#include <lidar_apollo_segmentation_tvm/util.hpp>

#include <cmath>
#include <vector>

namespace autoware
{
namespace perception
{
namespace lidar_apollo_segmentation_tvm
{
FeatureMapInterface::FeatureMapInterface(
  const int32_t _channels, const int32_t _width, const int32_t _height, const int32_t _range)
: channels(_channels),
  width(_width),
  height(_height),
  range(_range),
  max_height_data(nullptr),
  mean_height_data(nullptr),
  count_data(nullptr),
  direction_data(nullptr),
  top_intensity_data(nullptr),
  mean_intensity_data(nullptr),
  distance_data(nullptr),
  nonempty_data(nullptr)
{
  map_data.resize(width * height * channels);
}

FeatureMap::FeatureMap(const int32_t width, const int32_t height, const int32_t range)
: FeatureMapInterface(4, width, height, range)
{
  max_height_data = &(map_data[0]) + width * height * 0;
  mean_height_data = &(map_data[0]) + width * height * 1;
  count_data = &(map_data[0]) + width * height * 2;
  nonempty_data = &(map_data[0]) + width * height * 3;
}
void FeatureMap::initializeMap(std::vector<float> & map)
{
  (void)map;
}
void FeatureMap::resetMap(std::vector<float> & map)
{
  const int32_t size = width * height;
  (void)map;
  for (int32_t i = 0; i < size; ++i) {
    max_height_data[i] = -5.0f;
    mean_height_data[i] = 0.0f;
    count_data[i] = 0.0f;
    nonempty_data[i] = 0.0f;
  }
}

FeatureMapWithIntensity::FeatureMapWithIntensity(
  const int32_t width, const int32_t height, const int32_t range)
: FeatureMapInterface(6, width, height, range)
{
  max_height_data = &(map_data[0]) + width * height * 0;
  mean_height_data = &(map_data[0]) + width * height * 1;
  count_data = &(map_data[0]) + width * height * 2;
  top_intensity_data = &(map_data[0]) + width * height * 3;
  mean_intensity_data = &(map_data[0]) + width * height * 4;
  nonempty_data = &(map_data[0]) + width * height * 5;
}
void FeatureMapWithIntensity::initializeMap(std::vector<float> & map)
{
  (void)map;
}
void FeatureMapWithIntensity::resetMap(std::vector<float> & map)
{
  const int32_t size = width * height;
  (void)map;
  for (int32_t i = 0; i < size; ++i) {
    max_height_data[i] = -5.0f;
    mean_height_data[i] = 0.0f;
    count_data[i] = 0.0f;
    top_intensity_data[i] = 0.0f;
    mean_intensity_data[i] = 0.0f;
    nonempty_data[i] = 0.0f;
  }
}

FeatureMapWithConstant::FeatureMapWithConstant(
  const int32_t width, const int32_t height, const int32_t range)
: FeatureMapInterface(6, width, height, range)
{
  max_height_data = &(map_data[0]) + width * height * 0;
  mean_height_data = &(map_data[0]) + width * height * 1;
  count_data = &(map_data[0]) + width * height * 2;
  direction_data = &(map_data[0]) + width * height * 3;
  distance_data = &(map_data[0]) + width * height * 4;
  nonempty_data = &(map_data[0]) + width * height * 5;
}
void FeatureMapWithConstant::initializeMap(std::vector<float> & map)
{
  (void)map;
  for (int32_t row = 0; row < height; ++row) {
    for (int32_t col = 0; col < width; ++col) {
      int32_t idx = row * width + col;
      // * row <-> x, column <-> y
      // return the distance from the car to center of the grid.
      // Pc means point cloud = real world scale. so transform pixel scale to
      // real world scale
      float center_x = Pixel2Pc(row, height, range);
      float center_y = Pixel2Pc(col, width, range);
      // normalization. -0.5~0.5
      direction_data[idx] = std::atan2(center_y, center_x) / static_cast<float>(2.0 * M_PI);
      distance_data[idx] = std::hypot(center_x, center_y) / 60.0f - 0.5f;
    }
  }
}

void FeatureMapWithConstant::resetMap(std::vector<float> & map)
{
  const int32_t size = width * height;
  (void)map;
  for (int32_t i = 0; i < size; ++i) {
    max_height_data[i] = -5.0f;
    mean_height_data[i] = 0.0f;
    count_data[i] = 0.0f;
    nonempty_data[i] = 0.0f;
  }
}

FeatureMapWithConstantAndIntensity::FeatureMapWithConstantAndIntensity(
  const int32_t width, const int32_t height, const int32_t range)
: FeatureMapInterface(8, width, height, range)
{
  max_height_data = &(map_data[0]) + width * height * 0;
  mean_height_data = &(map_data[0]) + width * height * 1;
  count_data = &(map_data[0]) + width * height * 2;
  direction_data = &(map_data[0]) + width * height * 3;
  top_intensity_data = &(map_data[0]) + width * height * 4;
  mean_intensity_data = &(map_data[0]) + width * height * 5;
  distance_data = &(map_data[0]) + width * height * 6;
  nonempty_data = &(map_data[0]) + width * height * 7;
}
void FeatureMapWithConstantAndIntensity::initializeMap(std::vector<float> & map)
{
  (void)map;
  for (int32_t row = 0; row < height; ++row) {
    for (int32_t col = 0; col < width; ++col) {
      int32_t idx = row * width + col;
      // * row <-> x, column <-> y
      // return the distance from the car to center of the grid.
      // Pc means point cloud = real world scale. so transform pixel scale to
      // real world scale
      float center_x = Pixel2Pc(row, height, range);
      float center_y = Pixel2Pc(col, width, range);
      // normalization. -0.5~0.5
      direction_data[idx] = std::atan2(center_y, center_x) / static_cast<float>(2.0 * M_PI);
      distance_data[idx] = std::hypot(center_x, center_y) / 60.0f - 0.5f;
    }
  }
}

void FeatureMapWithConstantAndIntensity::resetMap(std::vector<float> & map)
{
  const int32_t size = width * height;
  (void)map;
  for (int32_t i = 0; i < size; ++i) {
    max_height_data[i] = -5.0f;
    mean_height_data[i] = 0.0f;
    count_data[i] = 0.0f;
    top_intensity_data[i] = 0.0f;
    mean_intensity_data[i] = 0.0f;
    nonempty_data[i] = 0.0f;
  }
}
}  // namespace lidar_apollo_segmentation_tvm
}  // namespace perception
}  // namespace autoware
