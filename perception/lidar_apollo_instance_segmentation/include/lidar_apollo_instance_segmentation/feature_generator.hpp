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

#ifndef LIDAR_APOLLO_INSTANCE_SEGMENTATION__FEATURE_GENERATOR_HPP_
#define LIDAR_APOLLO_INSTANCE_SEGMENTATION__FEATURE_GENERATOR_HPP_

#include "lidar_apollo_instance_segmentation/feature_map.hpp"
#include "util.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

namespace lidar_apollo_instance_segmentation
{
class FeatureGenerator
{
private:
  float min_height_;
  float max_height_;
  bool use_intensity_feature_;
  bool use_constant_feature_;
  std::shared_ptr<FeatureMapInterface> map_ptr_;

public:
  FeatureGenerator(
    const int width, const int height, const int range, const bool use_intensity_feature,
    const bool use_constant_feature);
  ~FeatureGenerator() {}

  std::shared_ptr<FeatureMapInterface> generate(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr & pc_ptr);
};
}  // namespace lidar_apollo_instance_segmentation

#endif  // LIDAR_APOLLO_INSTANCE_SEGMENTATION__FEATURE_GENERATOR_HPP_
