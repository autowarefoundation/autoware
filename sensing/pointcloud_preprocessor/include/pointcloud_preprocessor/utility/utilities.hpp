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

#ifndef POINTCLOUD_PREPROCESSOR__UTILITY__UTILITIES_HPP_
#define POINTCLOUD_PREPROCESSOR__UTILITY__UTILITIES_HPP_

#include <geometry_msgs/msg/polygon.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using PointCgal = K::Point_2;
using PolygonCgal = std::vector<PointCgal>;

namespace pointcloud_preprocessor::utils
{
/**
 * @brief convert ROS polygon to CGAL polygon
 */
void to_cgal_polygon(const geometry_msgs::msg::Polygon & polygon_in, PolygonCgal & polygon_out);

/**
 * @brief convert lanelet polygon to CGAL polygon
 */
void to_cgal_polygon(const lanelet::BasicPolygon2d & polygon_in, PolygonCgal & polygon_out);

/**
 * @brief remove points in the given polygon
 */
void remove_polygon_cgal_from_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud_in, const PolygonCgal & polyline_polygon,
  sensor_msgs::msg::PointCloud2 & cloud_out);

/**
 * @brief remove points in the given polygon
 */
void remove_polygon_cgal_from_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in, const PolygonCgal & polyline_polygon,
  pcl::PointCloud<pcl::PointXYZ> & cloud_out);

/**
 * @brief remove points in the given polygons
 */
void remove_polygon_cgal_from_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud_in,
  const std::vector<PolygonCgal> & polyline_polygons, sensor_msgs::msg::PointCloud2 & cloud_out);

/**
 * @brief remove points in the given polygons
 */
void remove_polygon_cgal_from_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in,
  const std::vector<PolygonCgal> & polyline_polygons, pcl::PointCloud<pcl::PointXYZ> & cloud_out);

/**
 * @brief return true if the given point is inside the at least one of the polygons
 */
bool point_within_cgal_polys(
  const pcl::PointXYZ & point, const std::vector<PolygonCgal> & polyline_polygons);

}  // namespace pointcloud_preprocessor::utils

#endif  // POINTCLOUD_PREPROCESSOR__UTILITY__UTILITIES_HPP_
