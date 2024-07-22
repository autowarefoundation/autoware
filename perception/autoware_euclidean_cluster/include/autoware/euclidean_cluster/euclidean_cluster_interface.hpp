// Copyright 2021 Tier IV, Inc.
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

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace autoware::euclidean_cluster
{
class EuclideanClusterInterface
{
public:
  EuclideanClusterInterface() = default;
  EuclideanClusterInterface(bool use_height, int min_cluster_size, int max_cluster_size)
  : use_height_(use_height),
    min_cluster_size_(min_cluster_size),
    max_cluster_size_(max_cluster_size)
  {
  }
  virtual ~EuclideanClusterInterface() = default;
  void setUseHeight(bool use_height) { use_height_ = use_height; }
  void setMinClusterSize(int size) { min_cluster_size_ = size; }
  void setMaxClusterSize(int size) { max_cluster_size_ = size; }
  virtual bool cluster(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
    std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters) = 0;

  virtual bool cluster(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_msg,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature & output_clusters) = 0;

protected:
  bool use_height_ = true;
  int min_cluster_size_;
  int max_cluster_size_;
};

}  // namespace autoware::euclidean_cluster
