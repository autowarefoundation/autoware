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

#include <autoware/image_projection_based_fusion/utils/utils.hpp>
#include <autoware_point_types/types.hpp>

#include <gtest/gtest.h>

using autoware_point_types::PointXYZI;

void setPointCloud2Fields(sensor_msgs::msg::PointCloud2 & pointcloud)
{
  pointcloud.fields.resize(4);
  pointcloud.fields[0].name = "x";
  pointcloud.fields[1].name = "y";
  pointcloud.fields[2].name = "z";
  pointcloud.fields[3].name = "intensity";
  pointcloud.fields[0].offset = 0;
  pointcloud.fields[1].offset = 4;
  pointcloud.fields[2].offset = 8;
  pointcloud.fields[3].offset = 12;
  pointcloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  pointcloud.fields[0].count = 1;
  pointcloud.fields[1].count = 1;
  pointcloud.fields[2].count = 1;
  pointcloud.fields[3].count = 1;
  pointcloud.height = 1;
  pointcloud.point_step = 16;
  pointcloud.is_bigendian = false;
  pointcloud.is_dense = true;
  pointcloud.header.frame_id = "dummy_frame_id";
  pointcloud.header.stamp.sec = 0;
  pointcloud.header.stamp.nanosec = 0;
}

// Test case 1: Test case when the input pointcloud is empty
TEST(UtilsTest, closest_cluster_empty_cluster_test_case1)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  setPointCloud2Fields(pointcloud);
  pointcloud.data.resize(0);
  pointcloud.width = 0;
  pointcloud.row_step = 0;

  pcl::PointXYZ center;
  center.x = 0.0;
  center.y = 0.0;
  center.z = 0.0;

  // testing closest_cluster function
  double cluster_2d_tolerance = 0.5;
  int min_cluster_size = 3;
  sensor_msgs::msg::PointCloud2 output_cluster;
  autoware::image_projection_based_fusion::closest_cluster(
    pointcloud, cluster_2d_tolerance, min_cluster_size, center, output_cluster);
  EXPECT_EQ(output_cluster.data.size(), std::size_t(0));
}

// Test case 2: Test case when the input pointcloud as one cluster
TEST(UtilsTest, closest_cluster_test_case2)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  setPointCloud2Fields(pointcloud);
  std::vector<float> dummy_x = {0.0, 0.1, 0.2};
  pointcloud.data.resize(dummy_x.size() * pointcloud.point_step);
  size_t global_offset = 0;
  for (auto x : dummy_x) {
    PointXYZI point;
    point.x = x;
    point.y = 0.0;
    point.z = 0.0;
    point.intensity = 0.0;
    memcpy(&pointcloud.data[global_offset], &point, pointcloud.point_step);
    global_offset += pointcloud.point_step;
  }
  pcl::PointXYZ center;
  center.x = 0.0;
  center.y = 0.0;
  center.z = 0.0;

  // testing closest_cluster function
  double cluster_2d_tolerance = 0.5;
  int min_cluster_size = 3;
  sensor_msgs::msg::PointCloud2 output_cluster;
  autoware::image_projection_based_fusion::closest_cluster(
    pointcloud, cluster_2d_tolerance, min_cluster_size, center, output_cluster);
  EXPECT_EQ(output_cluster.data.size(), dummy_x.size() * pointcloud.point_step);
}

// Test case 3: Test case when the input pointcloud as two clusters
TEST(UtilsTest, closest_cluster_test_case3)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  setPointCloud2Fields(pointcloud);
  std::vector<float> dummy_x = {0.0, 0.1, 0.2, 1.0, 1.1, 1.2, 1.21, 1.22};
  pointcloud.data.resize(dummy_x.size() * pointcloud.point_step);
  size_t global_offset = 0;
  for (auto x : dummy_x) {
    PointXYZI point;
    point.x = x;
    point.y = 0.0;
    point.z = 0.0;
    point.intensity = 0.0;
    memcpy(&pointcloud.data[global_offset], &point, pointcloud.point_step);
    global_offset += pointcloud.point_step;
  }
  pcl::PointXYZ center;
  center.x = 0.0;
  center.y = 0.0;
  center.z = 0.0;

  // testing closest_cluster function
  double cluster_2d_tolerance = 0.5;
  int min_cluster_size = 3;
  sensor_msgs::msg::PointCloud2 output_cluster;
  autoware::image_projection_based_fusion::closest_cluster(
    pointcloud, cluster_2d_tolerance, min_cluster_size, center, output_cluster);
  EXPECT_EQ(output_cluster.data.size(), 3 * pointcloud.point_step);
}

// Test case 4: Test case when the input pointcloud as two clusters
TEST(UtilsTest, closest_cluster_test_case4)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  setPointCloud2Fields(pointcloud);
  std::vector<float> dummy_x = {0.0, 0.1, 1.0, 1.01, 1.1, 1.2, 1.21, 1.22};
  pointcloud.data.resize(dummy_x.size() * pointcloud.point_step);
  size_t global_offset = 0;
  for (auto x : dummy_x) {
    PointXYZI point;
    point.x = x;
    point.y = 0.0;
    point.z = 0.0;
    point.intensity = 0.0;
    memcpy(&pointcloud.data[global_offset], &point, pointcloud.point_step);
    global_offset += pointcloud.point_step;
  }
  pcl::PointXYZ center;
  center.x = 0.0;
  center.y = 0.0;
  center.z = 0.0;

  // testing closest_cluster function
  double cluster_2d_tolerance = 0.5;
  int min_cluster_size = 3;
  sensor_msgs::msg::PointCloud2 output_cluster;
  autoware::image_projection_based_fusion::closest_cluster(
    pointcloud, cluster_2d_tolerance, min_cluster_size, center, output_cluster);
  EXPECT_EQ(output_cluster.data.size(), 6 * pointcloud.point_step);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
