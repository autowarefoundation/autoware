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

#include "dummy_perception_publisher/node.hpp"
#include "dummy_perception_publisher/signed_distance_function.hpp"

#include <pcl/impl/point_types.hpp>

#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <functional>
#include <limits>
#include <memory>

namespace
{

static constexpr double epsilon = 0.001;
static constexpr double step = 0.05;
static constexpr double vertical_theta_step = (1.0 / 180.0) * M_PI;
static constexpr double vertical_min_theta = (-15.0 / 180.0) * M_PI;
static constexpr double vertical_max_theta = (15.0 / 180.0) * M_PI;
static constexpr double horizontal_theta_step = (0.1 / 180.0) * M_PI;
static constexpr double horizontal_min_theta = (-180.0 / 180.0) * M_PI;
static constexpr double horizontal_max_theta = (180.0 / 180.0) * M_PI;

pcl::PointXYZ getPointWrtBaseLink(
  const tf2::Transform & tf_base_link2moved_object, double x, double y, double z)
{
  const auto p_wrt_base = tf_base_link2moved_object(tf2::Vector3(x, y, z));
  return pcl::PointXYZ(p_wrt_base.x(), p_wrt_base.y(), p_wrt_base.z());
}

}  // namespace

void ObjectCentricPointCloudCreator::create_object_pointcloud(
  const ObjectInfo & obj_info, const tf2::Transform & tf_base_link2map,
  std::mt19937 & random_generator, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) const
{
  std::normal_distribution<> x_random(0.0, obj_info.std_dev_x);
  std::normal_distribution<> y_random(0.0, obj_info.std_dev_y);
  std::normal_distribution<> z_random(0.0, obj_info.std_dev_z);

  const auto tf_base_link2moved_object = tf_base_link2map * obj_info.tf_map2moved_object;

  const double min_z = -1.0 * (obj_info.height / 2.0) + tf_base_link2moved_object.getOrigin().z();
  const double max_z = 1.0 * (obj_info.height / 2.0) + tf_base_link2moved_object.getOrigin().z();
  pcl::PointCloud<pcl::PointXYZ> horizontal_candidate_pointcloud;
  pcl::PointCloud<pcl::PointXYZ> horizontal_pointcloud;
  {
    const double y = -1.0 * (obj_info.width / 2.0);
    for (double x = -1.0 * (obj_info.length / 2.0); x <= ((obj_info.length / 2.0) + epsilon);
         x += step) {
      horizontal_candidate_pointcloud.push_back(
        getPointWrtBaseLink(tf_base_link2moved_object, x, y, 0.0));
    }
  }
  {
    const double y = 1.0 * (obj_info.width / 2.0);
    for (double x = -1.0 * (obj_info.length / 2.0); x <= ((obj_info.length / 2.0) + epsilon);
         x += step) {
      horizontal_candidate_pointcloud.push_back(
        getPointWrtBaseLink(tf_base_link2moved_object, x, y, 0.0));
    }
  }
  {
    const double x = -1.0 * (obj_info.length / 2.0);
    for (double y = -1.0 * (obj_info.width / 2.0); y <= ((obj_info.width / 2.0) + epsilon);
         y += step) {
      horizontal_candidate_pointcloud.push_back(
        getPointWrtBaseLink(tf_base_link2moved_object, x, y, 0.0));
    }
  }
  {
    const double x = 1.0 * (obj_info.length / 2.0);
    for (double y = -1.0 * (obj_info.width / 2.0); y <= ((obj_info.width / 2.0) + epsilon);
         y += step) {
      horizontal_candidate_pointcloud.push_back(
        getPointWrtBaseLink(tf_base_link2moved_object, x, y, 0.0));
    }
  }
  // 2D ray tracing
  size_t ranges_size =
    std::ceil((horizontal_max_theta - horizontal_min_theta) / horizontal_theta_step);
  std::vector<double> horizontal_ray_traced_2d_pointcloud;
  horizontal_ray_traced_2d_pointcloud.assign(ranges_size, std::numeric_limits<double>::infinity());
  const int no_data = -1;
  std::vector<int> horizontal_ray_traced_pointcloud_indices;
  horizontal_ray_traced_pointcloud_indices.assign(ranges_size, no_data);
  for (size_t i = 0; i < horizontal_candidate_pointcloud.points.size(); ++i) {
    double angle =
      std::atan2(horizontal_candidate_pointcloud.at(i).y, horizontal_candidate_pointcloud.at(i).x);
    double range =
      std::hypot(horizontal_candidate_pointcloud.at(i).y, horizontal_candidate_pointcloud.at(i).x);
    if (angle < horizontal_min_theta || angle > horizontal_max_theta) {
      continue;
    }
    int index = (angle - horizontal_min_theta) / horizontal_theta_step;
    if (range < horizontal_ray_traced_2d_pointcloud[index]) {
      horizontal_ray_traced_2d_pointcloud[index] = range;
      horizontal_ray_traced_pointcloud_indices.at(index) = i;
    }
  }

  for (const auto & pointcloud_index : horizontal_ray_traced_pointcloud_indices) {
    if (pointcloud_index != no_data) {
      // generate vertical point
      horizontal_pointcloud.push_back(horizontal_candidate_pointcloud.at(pointcloud_index));
      const double distance = std::hypot(
        horizontal_candidate_pointcloud.at(pointcloud_index).x,
        horizontal_candidate_pointcloud.at(pointcloud_index).y);
      for (double vertical_theta = vertical_min_theta;
           vertical_theta <= vertical_max_theta + epsilon; vertical_theta += vertical_theta_step) {
        const double z = distance * std::tan(vertical_theta);
        if (min_z <= z && z <= max_z + epsilon) {
          pcl::PointXYZ point;
          point.x =
            horizontal_candidate_pointcloud.at(pointcloud_index).x + x_random(random_generator);
          point.y =
            horizontal_candidate_pointcloud.at(pointcloud_index).y + y_random(random_generator);
          point.z = z + z_random(random_generator);
          pointcloud->push_back(point);
        }
      }
    }
  }
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ObjectCentricPointCloudCreator::create_pointclouds(
  const std::vector<ObjectInfo> & obj_infos, const tf2::Transform & tf_base_link2map,
  std::mt19937 & random_generator, pcl::PointCloud<pcl::PointXYZ>::Ptr & merged_pointcloud) const
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointclouds_tmp;
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);

  for (const auto & obj_info : obj_infos) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_shared_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->create_object_pointcloud(
      obj_info, tf_base_link2map, random_generator, pointcloud_shared_ptr);
    pointclouds_tmp.push_back(pointcloud_shared_ptr);
  }

  for (const auto & cloud : pointclouds_tmp) {
    for (const auto & pt : *cloud) {
      merged_pointcloud_tmp->push_back(pt);
    }
  }

  if (!enable_ray_tracing_) {
    merged_pointcloud = merged_pointcloud_tmp;
    return pointclouds_tmp;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr ray_traced_merged_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGridOcclusionEstimation<pcl::PointXYZ> ray_tracing_filter;
  ray_tracing_filter.setInputCloud(merged_pointcloud_tmp);
  ray_tracing_filter.setLeafSize(0.25, 0.25, 0.25);
  ray_tracing_filter.initializeVoxelGrid();
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointclouds;
  for (size_t i = 0; i < pointclouds_tmp.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ray_traced_pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t j = 0; j < pointclouds_tmp.at(i)->size(); ++j) {
      Eigen::Vector3i grid_coordinates = ray_tracing_filter.getGridCoordinates(
        pointclouds_tmp.at(i)->at(j).x, pointclouds_tmp.at(i)->at(j).y,
        pointclouds_tmp.at(i)->at(j).z);
      int grid_state;
      if (ray_tracing_filter.occlusionEstimation(grid_state, grid_coordinates) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("dummy_perception_publisher"), "ray tracing failed");
      }
      if (grid_state == 1) {  // occluded
        continue;
      } else {  // not occluded
        ray_traced_pointcloud_ptr->push_back(pointclouds_tmp.at(i)->at(j));
        ray_traced_merged_pointcloud_ptr->push_back(pointclouds_tmp.at(i)->at(j));
      }
    }
    pointclouds.push_back(ray_traced_pointcloud_ptr);
  }
  merged_pointcloud = ray_traced_merged_pointcloud_ptr;
  return pointclouds;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> EgoCentricPointCloudCreator::create_pointclouds(
  const std::vector<ObjectInfo> & obj_infos, const tf2::Transform & tf_base_link2map,
  std::mt19937 & random_generator, pcl::PointCloud<pcl::PointXYZ>::Ptr & merged_pointcloud) const
{
  std::vector<std::shared_ptr<signed_distance_function::AbstractSignedDistanceFunction>> sdf_ptrs;
  for (const auto & obj_info : obj_infos) {
    const auto sdf_ptr = std::make_shared<signed_distance_function::BoxSDF>(
      obj_info.length, obj_info.width, tf_base_link2map * obj_info.tf_map2moved_object);
    sdf_ptrs.push_back(sdf_ptr);
  }
  const auto composite_sdf = signed_distance_function::CompositeSDF(sdf_ptrs);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointclouds(obj_infos.size());
  for (size_t i = 0; i < obj_infos.size(); ++i) {
    pointclouds.at(i) = (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
  }

  std::vector<double> min_zs(obj_infos.size());
  std::vector<double> max_zs(obj_infos.size());

  for (size_t idx = 0; idx < obj_infos.size(); ++idx) {
    const auto & obj_info = obj_infos.at(idx);
    const auto tf_base_link2moved_object = tf_base_link2map * obj_info.tf_map2moved_object;
    const double min_z = -1.0 * (obj_info.height / 2.0) + tf_base_link2moved_object.getOrigin().z();
    const double max_z = 1.0 * (obj_info.height / 2.0) + tf_base_link2moved_object.getOrigin().z();
    min_zs.at(idx) = min_z;
    max_zs.at(idx) = max_z;
  }

  double angle = 0.0;
  const auto n_scan = static_cast<size_t>(std::floor(2 * M_PI / horizontal_theta_step));
  for (size_t i = 0; i < n_scan; ++i) {
    angle += horizontal_theta_step;
    const auto dist = composite_sdf.getSphereTracingDist(0.0, 0.0, angle, visible_range_);

    if (std::isfinite(dist)) {
      const auto x_hit = dist * cos(angle);
      const auto y_hit = dist * sin(angle);
      const auto idx_hit = composite_sdf.nearest_sdf_index(x_hit, y_hit);
      const auto obj_info_here = obj_infos.at(idx_hit);
      const auto min_z_here = min_zs.at(idx_hit);
      const auto max_z_here = max_zs.at(idx_hit);
      std::normal_distribution<> x_random(0.0, obj_info_here.std_dev_x);
      std::normal_distribution<> y_random(0.0, obj_info_here.std_dev_y);
      std::normal_distribution<> z_random(0.0, obj_info_here.std_dev_z);

      for (double vertical_theta = vertical_min_theta;
           vertical_theta <= vertical_max_theta + epsilon; vertical_theta += vertical_theta_step) {
        const double z = dist * std::tan(vertical_theta);
        if (min_z_here <= z && z <= max_z_here + epsilon) {
          pointclouds.at(idx_hit)->push_back(pcl::PointXYZ(
            x_hit + x_random(random_generator), y_hit + y_random(random_generator),
            z + z_random(random_generator)));
        }
      }
    }
  }

  for (const auto & cloud : pointclouds) {
    for (const auto & pt : *cloud) {
      merged_pointcloud->push_back(pt);
    }
  }
  return pointclouds;
}
