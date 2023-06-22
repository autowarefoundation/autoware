// Copyright 2023 TIER IV, Inc.
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

#include "yabloc_common/ground_server/polygon_operation.hpp"

#include <rclcpp/logging.hpp>

namespace yabloc::ground_server
{
pcl::PointCloud<pcl::PointXYZ> sample_from_polygons(const lanelet::PolygonLayer & polygons)
{
  // NOTE: Under construction
  pcl::PointCloud<pcl::PointXYZ> raw_cloud;

  if (polygons.size() >= 2) {
    RCLCPP_FATAL_STREAM(
      rclcpp::get_logger("polygon"), "current ground server does not handle multi polygons");
  }

  for (const lanelet::ConstPolygon3d & polygon : polygons) {
    for (const lanelet::ConstPoint3d & p : polygon) {
      pcl::PointXYZ xyz;
      xyz.x = p.x();
      xyz.y = p.y();
      xyz.z = p.z();
      raw_cloud.push_back(xyz);
    }
  }
  return fill_points_in_polygon(raw_cloud);
}

void push_back_line(
  pcl::PointCloud<pcl::PointXYZ> & dst_cloud, const pcl::PointXYZ & from, const pcl::PointXYZ & to)
{
  Eigen::Vector3f f = from.getVector3fMap();
  Eigen::Vector3f t = to.getVector3fMap();
  const float L = (f - t).norm();

  for (float l = 0.f; l < L; l += 0.25f) {
    Eigen::Vector3f xyz = f + (t - f) * l;
    dst_cloud.emplace_back(xyz.x(), xyz.y(), xyz.z());
  }
}

void push_back_contour(
  pcl::PointCloud<pcl::PointXYZ> & dst_cloud, const pcl::PointCloud<pcl::PointXYZ> & vertices)
{
  const int N = vertices.size();
  for (int i = 0; i < N - 1; ++i) {
    push_back_line(dst_cloud, vertices.at(i), vertices.at(i + 1));
  }
  push_back_line(dst_cloud, vertices.at(0), vertices.at(N - 1));
}

pcl::PointCloud<pcl::PointXYZ> shrink_vertices(
  const pcl::PointCloud<pcl::PointXYZ> & vertices, float rate)
{
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  for (const pcl::PointXYZ p : vertices) center += p.getVector3fMap();
  center /= vertices.size();

  pcl::PointCloud<pcl::PointXYZ> dst_cloud;
  for (const pcl::PointXYZ p : vertices) {
    Eigen::Vector3f xyz = (p.getVector3fMap() - center) * rate + center;
    dst_cloud.emplace_back(xyz.x(), xyz.y(), xyz.z());
  }
  return dst_cloud;
}

pcl::PointCloud<pcl::PointXYZ> fill_points_in_polygon(
  const pcl::PointCloud<pcl::PointXYZ> & src_cloud)
{
  pcl::PointCloud<pcl::PointXYZ> dst_cloud;

  std::vector<float> shrink_rates = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1};
  for (float rate : shrink_rates) {
    push_back_contour(dst_cloud, shrink_vertices(src_cloud, rate));
  }

  return dst_cloud;
}
}  // namespace yabloc::ground_server
