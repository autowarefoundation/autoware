// Copyright 2022 TIER IV, Inc.
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

#include "obstacle_velocity_limiter/pointcloud_utils.hpp"

#include "obstacle_velocity_limiter/types.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl/Vertices.h>
#include <pcl/filters/crop_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <vector>

namespace obstacle_velocity_limiter
{

pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
  const PointCloud & pointcloud_msg, tier4_autoware_utils::TransformListener & transform_listener,
  const std::string & target_frame)
{
  const auto & header = pointcloud_msg.header;
  const auto transform = transform_listener.getTransform(
    target_frame, header.frame_id, header.stamp, rclcpp::Duration::from_nanoseconds(0));
  const Eigen::Matrix4f transform_matrix =
    tf2::transformToEigen(transform->transform).matrix().cast<float>();

  PointCloud transformed_msg;
  pcl_ros::transformPointCloud(transform_matrix, pointcloud_msg, transformed_msg);
  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(transformed_msg, transformed_pointcloud);
  return pcl::PointCloud<pcl::PointXYZ>::Ptr(
    new pcl::PointCloud<pcl::PointXYZ>(std::move(transformed_pointcloud)));
}

void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const ObstacleMasks & masks)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (!masks.positive_mask.outer().empty()) {
    for (const auto & p : masks.positive_mask.outer())
      polygon_cloud_ptr->push_back(pcl::PointXYZ(p.x(), p.y(), 0));
    std::vector<pcl::Vertices> polygons_idx(1);
    polygons_idx.front().vertices.resize(masks.positive_mask.outer().size());
    std::iota(polygons_idx.front().vertices.begin(), polygons_idx.front().vertices.end(), 0);
    pcl::CropHull<pcl::PointXYZ> crop;
    crop.setInputCloud(pointcloud);
    crop.setDim(2);
    crop.setHullCloud(polygon_cloud_ptr);
    crop.setHullIndices(polygons_idx);
    crop.setCropOutside(true);
    crop.filter(*pointcloud);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr mask_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> masks_idx(masks.negative_masks.size());
  size_t start_idx = 0;
  for (size_t i = 0; i < masks.negative_masks.size(); ++i) {
    const auto & polygon_mask = masks.negative_masks[i];
    const auto mask_size = polygon_mask.outer().size();
    for (const auto & p : polygon_mask.outer())
      mask_cloud_ptr->push_back(pcl::PointXYZ(p.x(), p.y(), 0));
    auto & mask_idx = masks_idx[i];
    mask_idx.vertices.resize(mask_size);
    std::iota(mask_idx.vertices.begin(), mask_idx.vertices.end(), start_idx);
    start_idx += mask_size;
  }
  pcl::CropHull<pcl::PointXYZ> crop_masks;
  crop_masks.setInputCloud(pointcloud);
  crop_masks.setDim(2);
  crop_masks.setHullCloud(mask_cloud_ptr);
  crop_masks.setHullIndices(masks_idx);
  crop_masks.setCropOutside(false);
  crop_masks.filter(*pointcloud);
}

multipoint_t extractObstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr)
{
  multipoint_t obstacles;
  if (pointcloud_ptr->empty()) return obstacles;
  obstacles.reserve(pointcloud_ptr->size());

  for (const auto & point : *pointcloud_ptr) {
    obstacles.push_back({point_t{point.x, point.y}});
  }
  return obstacles;
}

}  // namespace obstacle_velocity_limiter
