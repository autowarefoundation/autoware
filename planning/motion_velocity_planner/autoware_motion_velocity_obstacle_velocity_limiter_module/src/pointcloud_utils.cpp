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

#include "pointcloud_utils.hpp"

#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/Vertices.h>
#include <pcl/filters/crop_hull.h>
#include <pcl_conversions/pcl_conversions.h>

#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
{

void filterPointCloud(PointCloud::Ptr pointcloud, const ObstacleMasks & masks)
{
  PointCloud::Ptr polygon_cloud_ptr(new PointCloud);
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

  PointCloud::Ptr mask_cloud_ptr(new PointCloud);
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

multipoint_t extractObstacles(const PointCloud & pointcloud)
{
  multipoint_t obstacles;
  if (pointcloud.empty()) return obstacles;
  obstacles.reserve(pointcloud.size());

  for (const auto & point : pointcloud) {
    obstacles.push_back({point_t{point.x, point.y}});
  }
  return obstacles;
}

}  // namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
