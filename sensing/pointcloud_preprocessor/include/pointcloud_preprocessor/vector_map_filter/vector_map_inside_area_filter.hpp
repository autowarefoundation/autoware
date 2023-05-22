// Copyright 2022 Tier IV, Inc.
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

#ifndef POINTCLOUD_PREPROCESSOR__VECTOR_MAP_FILTER__VECTOR_MAP_INSIDE_AREA_FILTER_HPP_
#define POINTCLOUD_PREPROCESSOR__VECTOR_MAP_FILTER__VECTOR_MAP_INSIDE_AREA_FILTER_HPP_

#include "pointcloud_preprocessor/filter.hpp"
#include "pointcloud_preprocessor/utility/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <string>

using tier4_autoware_utils::MultiPoint2d;

namespace pointcloud_preprocessor
{
class VectorMapInsideAreaFilterComponent : public pointcloud_preprocessor::Filter
{
private:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  lanelet::ConstPolygons3d polygon_lanelets_;

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);

  // parameter
  std::string polygon_type_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit VectorMapInsideAreaFilterComponent(const rclcpp::NodeOptions & options);
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__VECTOR_MAP_FILTER__VECTOR_MAP_INSIDE_AREA_FILTER_HPP_
