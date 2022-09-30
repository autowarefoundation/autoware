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

#include <common/types.hpp>
#include <lidar_apollo_segmentation_tvm/feature_generator.hpp>
#include <lidar_apollo_segmentation_tvm/log_table.hpp>

#include <memory>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace perception
{
namespace lidar_apollo_segmentation_tvm
{
namespace
{
inline float32_t normalizeIntensity(float32_t intensity) { return intensity / 255; }
}  // namespace

FeatureGenerator::FeatureGenerator(
  const int32_t width, const int32_t height, const int32_t range,
  const bool8_t use_intensity_feature, const bool8_t use_constant_feature,
  const float32_t min_height, const float32_t max_height)
: use_intensity_feature_(use_intensity_feature),
  use_constant_feature_(use_constant_feature),
  min_height_(min_height),
  max_height_(max_height)
{
  // select feature map type
  if (use_constant_feature && use_intensity_feature) {
    map_ptr_ = std::make_shared<FeatureMapWithConstantAndIntensity>(width, height, range);
  } else if (use_constant_feature) {
    map_ptr_ = std::make_shared<FeatureMapWithConstant>(width, height, range);
  } else if (use_intensity_feature) {
    map_ptr_ = std::make_shared<FeatureMapWithIntensity>(width, height, range);
  } else {
    map_ptr_ = std::make_shared<FeatureMap>(width, height, range);
  }
  map_ptr_->initializeMap(map_ptr_->map_data);
}

std::shared_ptr<FeatureMapInterface> FeatureGenerator::generate(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & pc_ptr)
{
  const float64_t epsilon = 1e-6;
  map_ptr_->resetMap(map_ptr_->map_data);

  const int32_t size = map_ptr_->height * map_ptr_->width;

  const float32_t inv_res_x = 0.5f * map_ptr_->width / map_ptr_->range;
  const float32_t inv_res_y = 0.5f * map_ptr_->height / map_ptr_->range;

  for (size_t i = 0; i < pc_ptr->points.size(); ++i) {
    if (pc_ptr->points[i].z <= min_height_ || max_height_ <= pc_ptr->points[i].z) {
      continue;
    }

    // x on grid
    const int32_t pos_x = static_cast<int32_t>(
      std::floor((static_cast<float32_t>(map_ptr_->range) - pc_ptr->points[i].y) * inv_res_x));
    // y on grid
    const int32_t pos_y = static_cast<int32_t>(
      std::floor((static_cast<float32_t>(map_ptr_->range) - pc_ptr->points[i].x) * inv_res_y));
    if (pos_x < 0 || map_ptr_->width <= pos_x || pos_y < 0 || map_ptr_->height <= pos_y) {
      continue;
    }

    const int32_t idx = pos_y * map_ptr_->width + pos_x;

    if (map_ptr_->max_height_data[idx] < pc_ptr->points[i].z) {
      map_ptr_->max_height_data[idx] = pc_ptr->points[i].z;
      if (map_ptr_->top_intensity_data != nullptr) {
        map_ptr_->top_intensity_data[idx] = normalizeIntensity(pc_ptr->points[i].intensity);
      }
    }
    map_ptr_->mean_height_data[idx] += pc_ptr->points[i].z;
    if (map_ptr_->mean_intensity_data != nullptr) {
      map_ptr_->mean_intensity_data[idx] += normalizeIntensity(pc_ptr->points[i].intensity);
    }
    map_ptr_->count_data[idx] += 1.0f;
  }

  for (int32_t i = 0; i < size; ++i) {
    if (static_cast<float64_t>(map_ptr_->count_data[i]) < epsilon) {
      map_ptr_->max_height_data[i] = 0.0f;
    } else {
      map_ptr_->mean_height_data[i] /= map_ptr_->count_data[i];
      if (map_ptr_->mean_intensity_data != nullptr) {
        map_ptr_->mean_intensity_data[i] /= map_ptr_->count_data[i];
      }
      map_ptr_->nonempty_data[i] = 1.0f;
    }
    map_ptr_->count_data[i] = calcApproximateLog(map_ptr_->count_data[i]);
  }
  return map_ptr_;
}
}  // namespace lidar_apollo_segmentation_tvm
}  // namespace perception
}  // namespace autoware
