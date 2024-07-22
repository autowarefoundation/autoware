// Copyright 2022 The Autoware Contributors
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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__POLYGON_REMOVER__POLYGON_REMOVER_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__POLYGON_REMOVER__POLYGON_REMOVER_HPP_

#include "autoware/pointcloud_preprocessor/filter.hpp"
#include "autoware/pointcloud_preprocessor/utility/geometry.hpp"

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>

#include <vector>

namespace autoware::pointcloud_preprocessor
{
class PolygonRemoverComponent : public autoware::pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  void publishRemovedPolygon();

  void update_polygon(const geometry_msgs::msg::Polygon::ConstSharedPtr & polygon_in);
  static PolygonCgal polygon_geometry_to_cgal(
    const geometry_msgs::msg::Polygon::ConstSharedPtr & polygon_in);
  PointCloud2 remove_updated_polygon_from_cloud(const PointCloud2ConstPtr & cloud_in);
  PointCloud2 remove_polygon_cgal_from_cloud(
    const PointCloud2::ConstSharedPtr & cloud_in_ptr, const PolygonCgal & polyline_polygon);

private:
  rclcpp::Parameter param;
  std::vector<double> polygon_vertices_;
  geometry_msgs::msg::Polygon::SharedPtr polygon_;

  bool polygon_is_initialized_;
  bool will_visualize_;
  PolygonCgal polygon_cgal_;
  visualization_msgs::msg::Marker marker_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_ptr_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit PolygonRemoverComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__POLYGON_REMOVER__POLYGON_REMOVER_HPP_
