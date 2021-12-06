// Copyright 2020 TierIV
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

#include "lidar_apollo_instance_segmentation/feature_generator.hpp"

#include "lidar_apollo_instance_segmentation/log_table.hpp"

namespace
{
inline float normalizeIntensity(float intensity) { return intensity / 255.0f; }
}  // namespace

FeatureGenerator::FeatureGenerator(
  const int width, const int height, const int range, const bool use_intensity_feature,
  const bool use_constant_feature)
: min_height_(-5.0),
  max_height_(5.0),
  use_intensity_feature_(use_intensity_feature),
  use_constant_feature_(use_constant_feature)
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
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & pc_ptr)
{
  const double epsilon = 1e-6;
  map_ptr_->resetMap(map_ptr_->map_data);

  int size = map_ptr_->height * map_ptr_->width;

  const float inv_res_x = 0.5 * map_ptr_->width / map_ptr_->range;
  const float inv_res_y = 0.5 * map_ptr_->height / map_ptr_->range;

  for (size_t i = 0; i < pc_ptr->points.size(); ++i) {
    if (pc_ptr->points[i].z <= min_height_ || max_height_ <= pc_ptr->points[i].z) {
      continue;
    }

    const int pos_x = std::floor((map_ptr_->range - pc_ptr->points[i].y) * inv_res_x);  // x on grid
    const int pos_y = std::floor((map_ptr_->range - pc_ptr->points[i].x) * inv_res_y);  // y on grid
    if (pos_x < 0 || map_ptr_->width <= pos_x || pos_y < 0 || map_ptr_->height <= pos_y) {
      continue;
    }

    const int idx = pos_y * map_ptr_->width + pos_x;

    if (map_ptr_->max_height_data[idx] < pc_ptr->points[i].z) {
      map_ptr_->max_height_data[idx] = pc_ptr->points[i].z;
      if (map_ptr_->top_intensity_data != nullptr) {
        map_ptr_->top_intensity_data[idx] = normalizeIntensity(pc_ptr->points[i].intensity);
      }
    }
    map_ptr_->mean_height_data[idx] += static_cast<float>(pc_ptr->points[i].z);
    if (map_ptr_->mean_intensity_data != nullptr) {
      map_ptr_->mean_intensity_data[idx] += normalizeIntensity(pc_ptr->points[i].intensity);
    }
    map_ptr_->count_data[idx] += 1.0f;
  }

  for (int i = 0; i < size; ++i) {
    if (map_ptr_->count_data[i] < epsilon) {
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
