// Copyright 2024 Autoware Foundation
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

#ifndef TEST_UTIL_HPP_
#define TEST_UTIL_HPP_

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

inline pcl::PointCloud<pcl::PointXYZ> make_sample_half_cubic_pcd()
{
  constexpr float length = 20;     // [m]
  constexpr float interval = 0.2;  // [m]
  constexpr int num_points_per_line = static_cast<int>(length / interval) + 1;
  constexpr int num_points_per_plane = num_points_per_line * num_points_per_line;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 3 * num_points_per_plane;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  for (int i = 0; i < num_points_per_line; ++i) {
    for (int j = 0; j < num_points_per_line; ++j) {
      const float u = interval * static_cast<float>(j);
      const float v = interval * static_cast<float>(i);

      // xy
      cloud.points[num_points_per_plane * 0 + i * num_points_per_line + j].x = u;  // NOLINT
      cloud.points[num_points_per_plane * 0 + i * num_points_per_line + j].y = v;  // NOLINT
      cloud.points[num_points_per_plane * 0 + i * num_points_per_line + j].z = 0;  // NOLINT

      // yz
      cloud.points[num_points_per_plane * 1 + i * num_points_per_line + j].x = 0;  // NOLINT
      cloud.points[num_points_per_plane * 1 + i * num_points_per_line + j].y = u;  // NOLINT
      cloud.points[num_points_per_plane * 1 + i * num_points_per_line + j].z = v;  // NOLINT

      // zx
      cloud.points[num_points_per_plane * 2 + i * num_points_per_line + j].x = u;  // NOLINT
      cloud.points[num_points_per_plane * 2 + i * num_points_per_line + j].y = 0;  // NOLINT
      cloud.points[num_points_per_plane * 2 + i * num_points_per_line + j].z = v;  // NOLINT
    }
  }
  return cloud;
}

inline geometry_msgs::msg::PoseWithCovarianceStamped make_pose(const float x, const float y)
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose{};
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;
  pose.pose.pose.position.z = 0.0;
  pose.pose.pose.orientation.x = 0.0;
  pose.pose.pose.orientation.y = 0.0;
  pose.pose.pose.orientation.z = 0.0;
  pose.pose.pose.orientation.w = 1.0;
  pose.header.frame_id = "map";
  pose.header.stamp.sec = 0;
  pose.header.stamp.nanosec = 0;
  pose.pose.covariance[0] = 0.25;
  pose.pose.covariance[7] = 0.25;
  pose.pose.covariance[14] = 0.0025;
  pose.pose.covariance[21] = 0.0006853891909122467;
  pose.pose.covariance[28] = 0.0006853891909122467;
  pose.pose.covariance[35] = 0.06853891909122467;
  return pose;
}

inline sensor_msgs::msg::PointCloud2 make_default_sensor_pcd()
{
  pcl::PointCloud<pcl::PointXYZ> cloud = make_sample_half_cubic_pcd();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud.makeShared());
  vg.setLeafSize(1.0, 1.0, 1.0);
  vg.filter(cloud);
  sensor_msgs::msg::PointCloud2 input_cloud;
  pcl::toROSMsg(cloud, input_cloud);
  input_cloud.header.frame_id = "sensor_frame";
  input_cloud.header.stamp.sec = 1;
  input_cloud.header.stamp.nanosec = 0;
  return input_cloud;
}

#endif  // TEST_UTIL_HPP_
