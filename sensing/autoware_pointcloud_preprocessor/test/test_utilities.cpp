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

#include "autoware/pointcloud_preprocessor/utility/geometry.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <CGAL/Polygon_2.h>
#include <CGAL/Simple_cartesian.h>
#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <optional>

constexpr double EPSILON = 1e-6;

class RemovePolygonCgalFromCloudTest : public ::testing::Test
{
protected:
  PolygonCgal polygon;
  sensor_msgs::msg::PointCloud2 cloud_out;

  virtual void SetUp()
  {
    // Set up a simple square polygon
    polygon.push_back(PointCgal(0.0, 0.0));
    polygon.push_back(PointCgal(0.0, 1.0));
    polygon.push_back(PointCgal(1.0, 1.0));
    polygon.push_back(PointCgal(1.0, 0.0));
    polygon.push_back(PointCgal(0.0, 0.0));
  }

  void CreatePointCloud2(sensor_msgs::msg::PointCloud2 & cloud, double x, double y, double z)
  {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.push_back(pcl::PointXYZ(x, y, z));
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.frame_id = "map";
  }
};

// Test case 1: without max_z, points inside the polygon are removed
TEST_F(RemovePolygonCgalFromCloudTest, PointsInsidePolygonAreRemoved)
{
  sensor_msgs::msg::PointCloud2 cloud_in;
  CreatePointCloud2(cloud_in, 0.5, 0.5, 0.1);  // point inside the polygon

  autoware::pointcloud_preprocessor::utils::remove_polygon_cgal_from_cloud(
    cloud_in, polygon, cloud_out);

  pcl::PointCloud<pcl::PointXYZ> pcl_output;
  pcl::fromROSMsg(cloud_out, pcl_output);

  EXPECT_EQ(pcl_output.size(), 0);
}

// Test case 2: without max_z, points outside the polygon remain
TEST_F(RemovePolygonCgalFromCloudTest, PointsOutsidePolygonRemain)
{
  sensor_msgs::msg::PointCloud2 cloud_in;
  CreatePointCloud2(cloud_in, 1.5, 1.5, 0.1);  // point outside the polygon

  autoware::pointcloud_preprocessor::utils::remove_polygon_cgal_from_cloud(
    cloud_in, polygon, cloud_out);

  pcl::PointCloud<pcl::PointXYZ> pcl_output;
  pcl::fromROSMsg(cloud_out, pcl_output);

  EXPECT_EQ(pcl_output.size(), 1);
  EXPECT_NEAR(pcl_output.points[0].x, 1.5, EPSILON);
  EXPECT_NEAR(pcl_output.points[0].y, 1.5, EPSILON);
  EXPECT_NEAR(pcl_output.points[0].z, 0.1, EPSILON);
}

// Test case 3: with max_z, points inside the polygon and below max_z are removed
TEST_F(RemovePolygonCgalFromCloudTest, PointsBelowMaxZAreRemoved)
{
  sensor_msgs::msg::PointCloud2 cloud_in;
  CreatePointCloud2(cloud_in, 0.5, 0.5, 0.1);  // point inside the polygon, below max_z

  std::optional<float> max_z = 1.0f;
  autoware::pointcloud_preprocessor::utils::remove_polygon_cgal_from_cloud(
    cloud_in, polygon, cloud_out, max_z);

  pcl::PointCloud<pcl::PointXYZ> pcl_output;
  pcl::fromROSMsg(cloud_out, pcl_output);

  EXPECT_EQ(pcl_output.size(), 0);
}

// Test case 4: with max_z, points inside the polygon but above max_z remain
TEST_F(RemovePolygonCgalFromCloudTest, PointsAboveMaxZRemain)
{
  sensor_msgs::msg::PointCloud2 cloud_in;
  CreatePointCloud2(cloud_in, 0.5, 0.5, 1.5);  // point inside the polygon, above max_z

  std::optional<float> max_z = 1.0f;
  autoware::pointcloud_preprocessor::utils::remove_polygon_cgal_from_cloud(
    cloud_in, polygon, cloud_out, max_z);

  pcl::PointCloud<pcl::PointXYZ> pcl_output;
  pcl::fromROSMsg(cloud_out, pcl_output);

  EXPECT_EQ(pcl_output.size(), 1);
  EXPECT_NEAR(pcl_output.points[0].x, 0.5, EPSILON);
  EXPECT_NEAR(pcl_output.points[0].y, 0.5, EPSILON);
  EXPECT_NEAR(pcl_output.points[0].z, 1.5, EPSILON);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
