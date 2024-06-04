// Copyright 2020 Tier IV, Inc.
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

#include "euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <unordered_map>

namespace euclidean_cluster
{
VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster()
{
}

VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster(
  bool use_height, int min_cluster_size, int max_cluster_size)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size)
{
}

VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster(
  bool use_height, int min_cluster_size, int max_cluster_size, float tolerance,
  float voxel_leaf_size, int min_points_number_per_voxel)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size),
  tolerance_(tolerance),
  voxel_leaf_size_(voxel_leaf_size),
  min_points_number_per_voxel_(min_points_number_per_voxel)
{
}
// TODO(badai-nguyen): remove this function when field copying also implemented for
// euclidean_cluster.cpp
bool VoxelGridBasedEuclideanCluster::cluster(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
  std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters)
{
  (void)pointcloud;
  (void)clusters;
  return false;
}

bool VoxelGridBasedEuclideanCluster::cluster(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pointcloud_msg,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature & objects)
{
  // TODO(Saito) implement use_height is false version

  // create voxel
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  int point_step = pointcloud_msg->point_step;
  pcl::fromROSMsg(*pointcloud_msg, *pointcloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, 100000.0);
  voxel_grid_.setMinimumPointsNumberPerVoxel(min_points_number_per_voxel_);
  voxel_grid_.setInputCloud(pointcloud);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr);

  // voxel is pressed 2d
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & point : voxel_map_ptr->points) {
    pcl::PointXYZ point2d;
    point2d.x = point.x;
    point2d.y = point.y;
    point2d.z = 0.0;
    pointcloud_2d_ptr->push_back(point2d);
  }

  // create tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_2d_ptr);

  // clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  pcl_euclidean_cluster.setClusterTolerance(tolerance_);
  pcl_euclidean_cluster.setMinClusterSize(1);
  pcl_euclidean_cluster.setMaxClusterSize(max_cluster_size_);
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_2d_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);

  // create map to search cluster index from voxel grid index
  std::unordered_map</* voxel grid index */ int, /* cluster index */ int> map;
  std::vector<sensor_msgs::msg::PointCloud2> temporary_clusters;  // no check about cluster size
  std::vector<size_t> clusters_data_size;
  temporary_clusters.resize(cluster_indices.size());
  for (size_t cluster_idx = 0; cluster_idx < cluster_indices.size(); ++cluster_idx) {
    const auto & cluster = cluster_indices.at(cluster_idx);
    auto & temporary_cluster = temporary_clusters.at(cluster_idx);
    for (const auto & point_idx : cluster.indices) {
      map[point_idx] = cluster_idx;
    }
    temporary_cluster.height = pointcloud_msg->height;
    temporary_cluster.fields = pointcloud_msg->fields;
    temporary_cluster.point_step = point_step;
    temporary_cluster.data.resize(max_cluster_size_ * point_step);
    clusters_data_size.push_back(0);
  }

  // create vector of point cloud cluster. vector index is voxel grid index.
  for (size_t i = 0; i < pointcloud->points.size(); ++i) {
    const auto & point = pointcloud->points.at(i);
    const int index =
      voxel_grid_.getCentroidIndexAt(voxel_grid_.getGridCoordinates(point.x, point.y, point.z));
    if (map.find(index) != map.end()) {
      auto & cluster_data_size = clusters_data_size.at(map[index]);
      if (cluster_data_size + point_step > std::size_t(max_cluster_size_ * point_step)) {
        continue;
      }
      std::memcpy(
        &temporary_clusters.at(map[index]).data[cluster_data_size],
        &pointcloud_msg->data[i * point_step], point_step);
      cluster_data_size += point_step;
    }
  }

  // build output and check cluster size
  {
    for (size_t i = 0; i < temporary_clusters.size(); ++i) {
      auto & i_cluster_data_size = clusters_data_size.at(i);
      if (!(min_cluster_size_ <= static_cast<int>(i_cluster_data_size / point_step) &&
            static_cast<int>(i_cluster_data_size / point_step) <= max_cluster_size_)) {
        continue;
      }
      const auto & cluster = temporary_clusters.at(i);
      tier4_perception_msgs::msg::DetectedObjectWithFeature feature_object;
      feature_object.feature.cluster = cluster;
      feature_object.feature.cluster.data.resize(i_cluster_data_size);
      feature_object.feature.cluster.header = pointcloud_msg->header;
      feature_object.feature.cluster.is_bigendian = pointcloud_msg->is_bigendian;
      feature_object.feature.cluster.is_dense = pointcloud_msg->is_dense;
      feature_object.feature.cluster.point_step = point_step;
      feature_object.feature.cluster.row_step = i_cluster_data_size / pointcloud_msg->height;
      feature_object.feature.cluster.width =
        i_cluster_data_size / point_step / pointcloud_msg->height;

      feature_object.object.kinematics.pose_with_covariance.pose.position =
        getCentroid(feature_object.feature.cluster);
      autoware_perception_msgs::msg::ObjectClassification classification;
      classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      classification.probability = 1.0f;
      feature_object.object.classification.emplace_back(classification);

      objects.feature_objects.push_back(feature_object);
    }
    objects.header = pointcloud_msg->header;
  }

  return true;
}

}  // namespace euclidean_cluster
