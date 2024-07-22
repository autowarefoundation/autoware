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

#include "autoware/euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"

#include <autoware_point_types/types.hpp>
#include <experimental/random>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

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

sensor_msgs::msg::PointCloud2 generateClusterWithinVoxel(const int nb_points)
{
  sensor_msgs::msg::PointCloud2 pointcloud;
  setPointCloud2Fields(pointcloud);
  pointcloud.data.resize(nb_points * pointcloud.point_step);

  // generate one cluster with specified number of points within 1 voxel
  for (int i = 0; i < nb_points; ++i) {
    PointXYZI point;
    point.x = std::experimental::randint(0, 30) / 100.0;  // point.x within 0.0 to 0.3
    point.y = std::experimental::randint(0, 30) / 100.0;  // point.y within 0.0 to 0.3
    point.z = std::experimental::randint(0, 30) / 1.0;
    point.intensity = 0.0;
    memcpy(&pointcloud.data[i * pointcloud.point_step], &point, pointcloud.point_step);
  }
  pointcloud.width = nb_points;
  pointcloud.row_step = pointcloud.point_step * nb_points;
  return pointcloud;
}

// Test case 1: Test case when the input pointcloud has only one cluster with points number equal to
// max_cluster_size
TEST(VoxelGridBasedEuclideanClusterTest, testcase1)
{
  int nb_generated_points = 100;
  sensor_msgs::msg::PointCloud2 pointcloud = generateClusterWithinVoxel(nb_generated_points);

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output;
  std::shared_ptr<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster> cluster_;
  float tolerance = 0.7;
  float voxel_leaf_size = 0.3;
  int min_points_number_per_voxel = 1;
  int min_cluster_size = 1;
  int max_cluster_size = 100;
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel);
  if (cluster_->cluster(pointcloud_msg, output)) {
    std::cout << "cluster success" << std::endl;
  } else {
    std::cout << "cluster failed" << std::endl;
  }
  std::cout << "number of output clusters " << output.feature_objects.size() << std::endl;
  std::cout << "number points of first cluster: " << output.feature_objects[0].feature.cluster.width
            << std::endl;
  // the output clusters should has only one cluster with nb_generated_points points
  EXPECT_EQ(output.feature_objects.size(), 1);
  EXPECT_EQ(output.feature_objects[0].feature.cluster.width, nb_generated_points);
}

// Test case 2: Test case when the input pointcloud has only one cluster with points number less
// than min_cluster_size
TEST(VoxelGridBasedEuclideanClusterTest, testcase2)
{
  int nb_generated_points = 1;

  sensor_msgs::msg::PointCloud2 pointcloud = generateClusterWithinVoxel(nb_generated_points);

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output;
  std::shared_ptr<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster> cluster_;
  float tolerance = 0.7;
  float voxel_leaf_size = 0.3;
  int min_points_number_per_voxel = 1;
  int min_cluster_size = 2;
  int max_cluster_size = 100;
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel);
  if (cluster_->cluster(pointcloud_msg, output)) {
    std::cout << "cluster success" << std::endl;
  } else {
    std::cout << "cluster failed" << std::endl;
  }
  std::cout << "number of output clusters " << output.feature_objects.size() << std::endl;
  // the output clusters should be empty
  EXPECT_EQ(output.feature_objects.size(), 0);
}

// Test case 3: Test case when the input pointcloud has two clusters with points number greater to
// max_cluster_size
TEST(VoxelGridBasedEuclideanClusterTest, testcase3)
{
  int nb_generated_points = 100;
  sensor_msgs::msg::PointCloud2 pointcloud = generateClusterWithinVoxel(nb_generated_points);

  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg =
    std::make_shared<sensor_msgs::msg::PointCloud2>(pointcloud);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output;
  std::shared_ptr<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster> cluster_;
  float tolerance = 0.7;
  float voxel_leaf_size = 0.3;
  int min_points_number_per_voxel = 1;
  int min_cluster_size = 1;
  int max_cluster_size = 99;  // max_cluster_size is less than nb_generated_points
  bool use_height = false;
  cluster_ = std::make_shared<autoware::euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel);
  if (cluster_->cluster(pointcloud_msg, output)) {
    std::cout << "cluster success" << std::endl;
  } else {
    std::cout << "cluster failed" << std::endl;
  }
  std::cout << "number of output clusters " << output.feature_objects.size() << std::endl;
  // the output clusters should be emtpy
  EXPECT_EQ(output.feature_objects.size(), 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
