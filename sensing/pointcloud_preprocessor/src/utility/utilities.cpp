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

#include "pointcloud_preprocessor/utility/utilities.hpp"

namespace pointcloud_preprocessor::utils
{
void to_cgal_polygon(const geometry_msgs::msg::Polygon & polygon_in, PolygonCgal & polygon_out)
{
  if (polygon_in.points.size() < 3) {
    throw std::length_error("Polygon vertex count should be larger than 2.");
  }

  const auto & vertices_in = polygon_in.points;
  polygon_out.resize(vertices_in.size());
  std::transform(
    polygon_in.points.cbegin(), polygon_in.points.cend(), polygon_out.begin(),
    [](const geometry_msgs::msg::Point32 & p_in) { return PointCgal(p_in.x, p_in.y); });
}

void to_cgal_polygon(const lanelet::BasicPolygon2d & polygon_in, PolygonCgal & polygon_out)
{
  if (polygon_in.size() < 3) {
    throw std::length_error("Polygon vertex count should be larger than 2.");
  }

  const auto & vertices_in = polygon_in;
  polygon_out.resize(vertices_in.size());
  std::transform(
    polygon_in.cbegin(), polygon_in.cend(), polygon_out.begin(),
    [](const Eigen::Matrix<double, 2, 1> & p_in) { return PointCgal(p_in.x(), p_in.y()); });
}

void remove_polygon_cgal_from_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud_in, const PolygonCgal & polyline_polygon,
  sensor_msgs::msg::PointCloud2 & cloud_out)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_output;

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x"), iter_y(cloud_in, "y"),
       iter_z(cloud_in, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // check if the point is inside the polygon
    if (
      CGAL::bounded_side_2(
        polyline_polygon.begin(), polyline_polygon.end(), PointCgal(*iter_x, *iter_y), K()) ==
      CGAL::ON_UNBOUNDED_SIDE) {
      pcl::PointXYZ p;
      p.x = *iter_x;
      p.y = *iter_y;
      p.z = *iter_z;
      pcl_output.emplace_back(p);
    }
  }

  pcl::toROSMsg(pcl_output, cloud_out);
  cloud_out.header = cloud_in.header;
}

void remove_polygon_cgal_from_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in, const PolygonCgal & polyline_polygon,
  pcl::PointCloud<pcl::PointXYZ> & cloud_out)
{
  cloud_out.clear();
  cloud_out.header = cloud_in.header;

  for (const auto & p : cloud_in) {
    // check if the point is inside the polygon
    if (
      CGAL::bounded_side_2(
        polyline_polygon.begin(), polyline_polygon.end(), PointCgal(p.x, p.y), K()) ==
      CGAL::ON_UNBOUNDED_SIDE) {
      cloud_out.emplace_back(p);
    }
  }
}

void remove_polygon_cgal_from_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud_in,
  const std::vector<PolygonCgal> & polyline_polygons, sensor_msgs::msg::PointCloud2 & cloud_out)
{
  if (polyline_polygons.empty()) {
    cloud_out = cloud_in;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x"), iter_y(cloud_in, "y"),
       iter_z(cloud_in, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // if the point is inside the polygon, skip inserting and check the next point
    pcl::PointXYZ p(*iter_x, *iter_y, *iter_z);
    if (point_within_cgal_polys(p, polyline_polygons)) {
      continue;
    }
    filtered_cloud.emplace_back(p);
  }

  pcl::toROSMsg(filtered_cloud, cloud_out);
  cloud_out.header = cloud_in.header;
}

void remove_polygon_cgal_from_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in,
  const std::vector<PolygonCgal> & polyline_polygons, pcl::PointCloud<pcl::PointXYZ> & cloud_out)
{
  if (polyline_polygons.empty()) {
    cloud_out = cloud_in;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  for (const auto & p : cloud_in) {
    // if the point is inside the polygon, skip inserting and check the next point
    if (point_within_cgal_polys(p, polyline_polygons)) {
      continue;
    }
    filtered_cloud.emplace_back(p);
  }

  cloud_out = filtered_cloud;
  cloud_out.header = cloud_in.header;
}

bool point_within_cgal_polys(
  const pcl::PointXYZ & point, const std::vector<PolygonCgal> & polyline_polygons)
{
  for (const auto & polygon : polyline_polygons) {
    if (
      CGAL::bounded_side_2(polygon.cbegin(), polygon.cend(), PointCgal(point.x, point.y), K()) ==
      CGAL::ON_BOUNDED_SIDE) {
      return true;
    }
  }

  return false;
}

}  // namespace pointcloud_preprocessor::utils
