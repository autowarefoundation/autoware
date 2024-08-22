// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__VECTOR_MAP_FILTER__VECTOR_MAP_INSIDE_AREA_FILTER_NODE_HPP_  // NOLINT
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__VECTOR_MAP_FILTER__VECTOR_MAP_INSIDE_AREA_FILTER_NODE_HPP_  // NOLINT

#include "autoware/pointcloud_preprocessor/filter.hpp"
#include "autoware/pointcloud_preprocessor/utility/geometry.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <memory>
#include <string>

using autoware::universe_utils::MultiPoint2d;

namespace autoware::pointcloud_preprocessor
{
class VectorMapInsideAreaFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
private:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;
  lanelet::ConstPolygons3d polygon_lanelets_;

  void mapCallback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);

  // parameter
  std::string polygon_type_;
  bool use_z_filter_ = false;
  float z_threshold_;

  // tf2 listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit VectorMapInsideAreaFilterComponent(const rclcpp::NodeOptions & options);
};

}  // namespace autoware::pointcloud_preprocessor

// clang-format off
#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__VECTOR_MAP_FILTER__VECTOR_MAP_INSIDE_AREA_FILTER_NODE_HPP_  // NOLINT
// clang-format on
