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

#include "lidar_apollo_instance_segmentation/feature_map.hpp"

#include "lidar_apollo_instance_segmentation/util.hpp"

#include <cmath>

namespace lidar_apollo_instance_segmentation
{
FeatureMapInterface::FeatureMapInterface(
  const int _channels, const int _width, const int _height, const int _range)
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

FeatureMap::FeatureMap(const int width, const int height, const int range)
: FeatureMapInterface(4, width, height, range)
{
  max_height_data = &(map_data[0]) + width * height * 0;
  mean_height_data = &(map_data[0]) + width * height * 1;
  count_data = &(map_data[0]) + width * height * 2;
  nonempty_data = &(map_data[0]) + width * height * 3;
}
void FeatureMap::initializeMap([[maybe_unused]] std::vector<float> & map)
{
}
void FeatureMap::resetMap([[maybe_unused]] std::vector<float> & map)
{
  const int size = width * height;
  for (int i = 0; i < size; ++i) {
    max_height_data[i] = -5.0f;
    mean_height_data[i] = 0.0f;
    count_data[i] = 0.0f;
    nonempty_data[i] = 0.0f;
  }
}

FeatureMapWithIntensity::FeatureMapWithIntensity(const int width, const int height, const int range)
: FeatureMapInterface(6, width, height, range)
{
  max_height_data = &(map_data[0]) + width * height * 0;
  mean_height_data = &(map_data[0]) + width * height * 1;
  count_data = &(map_data[0]) + width * height * 2;
  top_intensity_data = &(map_data[0]) + width * height * 3;
  mean_intensity_data = &(map_data[0]) + width * height * 4;
  nonempty_data = &(map_data[0]) + width * height * 5;
}
void FeatureMapWithIntensity::initializeMap([[maybe_unused]] std::vector<float> & map)
{
}
void FeatureMapWithIntensity::resetMap([[maybe_unused]] std::vector<float> & map)
{
  const int size = width * height;
  for (int i = 0; i < size; ++i) {
    max_height_data[i] = -5.0f;
    mean_height_data[i] = 0.0f;
    count_data[i] = 0.0f;
    top_intensity_data[i] = 0.0f;
    mean_intensity_data[i] = 0.0f;
    nonempty_data[i] = 0.0f;
  }
}

FeatureMapWithConstant::FeatureMapWithConstant(const int width, const int height, const int range)
: FeatureMapInterface(6, width, height, range)
{
  max_height_data = &(map_data[0]) + width * height * 0;
  mean_height_data = &(map_data[0]) + width * height * 1;
  count_data = &(map_data[0]) + width * height * 2;
  direction_data = &(map_data[0]) + width * height * 3;
  distance_data = &(map_data[0]) + width * height * 4;
  nonempty_data = &(map_data[0]) + width * height * 5;
}
void FeatureMapWithConstant::initializeMap([[maybe_unused]] std::vector<float> & map)
{
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      int idx = row * width + col;
      // * row <-> x, column <-> y
      // return the distance from my car to center of the grid.
      // Pc means point cloud = real world scale. so transform pixel scale to
      // real world scale
      float center_x = Pixel2Pc(row, height, range);
      float center_y = Pixel2Pc(col, width, range);
      // normalization. -0.5~0.5
      direction_data[idx] = static_cast<float>(std::atan2(center_y, center_x) / (2.0 * M_PI));
      distance_data[idx] = static_cast<float>(std::hypot(center_x, center_y) / 60.0 - 0.5);
    }
  }
}

void FeatureMapWithConstant::resetMap([[maybe_unused]] std::vector<float> & map)
{
  const int size = width * height;
  for (int i = 0; i < size; ++i) {
    max_height_data[i] = -5.0f;
    mean_height_data[i] = 0.0f;
    count_data[i] = 0.0f;
    nonempty_data[i] = 0.0f;
  }
}

FeatureMapWithConstantAndIntensity::FeatureMapWithConstantAndIntensity(
  const int width, const int height, const int range)
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
void FeatureMapWithConstantAndIntensity::initializeMap([[maybe_unused]] std::vector<float> & map)
{
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      int idx = row * width + col;
      // * row <-> x, column <-> y
      // return the distance from my car to center of the grid.
      // Pc means point cloud = real world scale. so transform pixel scale to
      // real world scale
      float center_x = Pixel2Pc(row, height, range);
      float center_y = Pixel2Pc(col, width, range);
      // normalization. -0.5~0.5
      direction_data[idx] = static_cast<float>(std::atan2(center_y, center_x) / (2.0 * M_PI));
      distance_data[idx] = static_cast<float>(std::hypot(center_x, center_y) / 60.0 - 0.5);
    }
  }
}

void FeatureMapWithConstantAndIntensity::resetMap([[maybe_unused]] std::vector<float> & map)
{
  const int size = width * height;
  for (int i = 0; i < size; ++i) {
    max_height_data[i] = -5.0f;
    mean_height_data[i] = 0.0f;
    count_data[i] = 0.0f;
    top_intensity_data[i] = 0.0f;
    mean_intensity_data[i] = 0.0f;
    nonempty_data[i] = 0.0f;
  }
}
}  // namespace lidar_apollo_instance_segmentation
