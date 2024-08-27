// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__UTILS__UTILS_HPP_
#define AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__UTILS__UTILS_HPP_

#include "autoware/probabilistic_occupancy_grid_map/cost_value/cost_value.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::occupancy_grid_map
{
namespace utils
{

/**
 * @brief 3D point struct for sorting and searching
 *
 */
struct MyPoint3d
{
  float x;
  float y;
  float z;

  // constructor
  MyPoint3d(float x, float y, float z) : x(x), y(y), z(z) {}

  // calc squared norm
  float norm2() const { return powf(x, 2) + powf(y, 2) + powf(z, 2); }

  // calc arctan2
  float arctan2() const { return atan2f(y, x); }

  // overload operator<
  bool operator<(const MyPoint3d & other) const
  {
    const auto a = norm2();
    const auto b = other.norm2();
    if (a == b) {  // escape when norm2 is same
      return arctan2() < other.arctan2();
    } else {
      return a < b;
    }
  }

  // overload operator==
  bool operator==(const MyPoint3d & other) const
  {
    return fabsf(x - other.x) < FLT_EPSILON && fabsf(y - other.y) < FLT_EPSILON &&
           fabsf(z - other.z) < FLT_EPSILON;
  }
};

bool transformPointcloud(
  const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, sensor_msgs::msg::PointCloud2 & output);

void transformPointcloud(
  const sensor_msgs::msg::PointCloud2 & input, const geometry_msgs::msg::Pose & pose,
  sensor_msgs::msg::PointCloud2 & output);

Eigen::Matrix4f getTransformMatrix(const geometry_msgs::msg::Pose & pose);

bool cropPointcloudByHeight(
  const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, const float min_height, const float max_height,
  sensor_msgs::msg::PointCloud2 & output);

// get pose from tf2
geometry_msgs::msg::Pose getPose(
  const std_msgs::msg::Header & source_header, const tf2_ros::Buffer & tf2,
  const std::string & target_frame);

geometry_msgs::msg::Pose getPose(
  const builtin_interfaces::msg::Time & stamp, const tf2_ros::Buffer & tf2,
  const std::string & source_frame, const std::string & target_frame);

// get inverted pose
geometry_msgs::msg::Pose getInversePose(const geometry_msgs::msg::Pose & pose);

bool extractCommonPointCloud(
  const sensor_msgs::msg::PointCloud2 & obstacle_pc, const sensor_msgs::msg::PointCloud2 & raw_pc,
  sensor_msgs::msg::PointCloud2 & output_obstacle_pc);

}  // namespace utils
}  // namespace autoware::occupancy_grid_map

#endif  // AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__UTILS__UTILS_HPP_
