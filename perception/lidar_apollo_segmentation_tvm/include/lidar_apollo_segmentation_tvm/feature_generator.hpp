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

#ifndef LIDAR_APOLLO_SEGMENTATION_TVM__FEATURE_GENERATOR_HPP_
#define LIDAR_APOLLO_SEGMENTATION_TVM__FEATURE_GENERATOR_HPP_

#include <common/types.hpp>
#include <lidar_apollo_segmentation_tvm/feature_map.hpp>
#include <lidar_apollo_segmentation_tvm/util.hpp>
#include <lidar_apollo_segmentation_tvm/visibility_control.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <memory>

namespace autoware
{
namespace perception
{
namespace lidar_apollo_segmentation_tvm
{
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

/// \brief A FeatureMap generator based on channel feature information.
class LIDAR_APOLLO_SEGMENTATION_TVM_LOCAL FeatureGenerator
{
private:
  const bool8_t use_intensity_feature_;
  const bool8_t use_constant_feature_;
  const float32_t min_height_;
  const float32_t max_height_;
  std::shared_ptr<FeatureMapInterface> map_ptr_;

public:
  /// \brief Constructor
  /// \param[in] width The number of cells in X (column) axis of the 2D grid.
  /// \param[in] height The number of cells in Y (row) axis of the 2D grid.
  /// \param[in] range The range of the 2D grid.
  /// \param[in] use_intensity_feature Enable input channel intensity feature.
  /// \param[in] use_constant_feature Enable input channel constant feature.
  /// \param[in] min_height The minimum height.
  /// \param[in] max_height The maximum height.
  explicit FeatureGenerator(
    int32_t width, int32_t height, int32_t range, bool8_t use_intensity_feature,
    bool8_t use_constant_feature, float32_t min_height, float32_t max_height);

  /// \brief Generate a FeatureMap based on the configured features of this object.
  /// \param[in] pc_ptr Pointcloud used to populate the generated FeatureMap.
  /// \return A shared pointer to the generated FeatureMap.
  std::shared_ptr<FeatureMapInterface> generate(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & pc_ptr);
};
}  // namespace lidar_apollo_segmentation_tvm
}  // namespace perception
}  // namespace autoware
#endif  // LIDAR_APOLLO_SEGMENTATION_TVM__FEATURE_GENERATOR_HPP_
