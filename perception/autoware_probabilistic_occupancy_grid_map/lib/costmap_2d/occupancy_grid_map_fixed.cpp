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

#include "autoware/probabilistic_occupancy_grid_map/costmap_2d/occupancy_grid_map_fixed.hpp"

#include "autoware/probabilistic_occupancy_grid_map/cost_value/cost_value.hpp"
#include "autoware/probabilistic_occupancy_grid_map/utils/utils.hpp"

#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <grid_map_costmap_2d/grid_map_costmap_2d.hpp>
#include <pcl_ros/transforms.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <algorithm>

namespace autoware::occupancy_grid_map
{
namespace costmap_2d
{
using sensor_msgs::PointCloud2ConstIterator;

OccupancyGridMapFixedBlindSpot::OccupancyGridMapFixedBlindSpot(
  const unsigned int cells_size_x, const unsigned int cells_size_y, const float resolution)
: OccupancyGridMapInterface(cells_size_x, cells_size_y, resolution)
{
}

/**
 * @brief update Gridmap with PointCloud
 *
 * @param raw_pointcloud raw point cloud on a certain frame (usually base_link)
 * @param obstacle_pointcloud raw point cloud on a certain frame (usually base_link)
 * @param robot_pose frame of the input point cloud (usually base_link)
 * @param scan_origin manually chosen grid map origin frame
 */
void OccupancyGridMapFixedBlindSpot::updateWithPointCloud(
  const PointCloud2 & raw_pointcloud, const PointCloud2 & obstacle_pointcloud,
  const Pose & robot_pose, const Pose & scan_origin)
{
  const size_t angle_bin_size =
    ((max_angle_ - min_angle_) * angle_increment_inv_) + size_t(1 /*margin*/);

  // Transform Matrix from base_link to map frame
  mat_map_ = utils::getTransformMatrix(robot_pose);

  const auto scan2map_pose = utils::getInversePose(scan_origin);  // scan -> map transform pose

  // Transform Matrix from map frame to scan frame
  mat_scan_ = utils::getTransformMatrix(scan2map_pose);

  if (!offset_initialized_) {
    setFieldOffsets(raw_pointcloud, obstacle_pointcloud);
  }

  // Create angle bins and sort by distance
  struct BinInfo
  {
    BinInfo() = default;
    BinInfo(const double _range, const double _wx, const double _wy)
    : range(_range), wx(_wx), wy(_wy)
    {
    }
    double range{};
    double wx{};
    double wy{};
  };

  std::vector</*angle bin*/ std::vector<BinInfo>> obstacle_pointcloud_angle_bins(angle_bin_size);
  std::vector</*angle bin*/ std::vector<BinInfo>> raw_pointcloud_angle_bins(angle_bin_size);

  const size_t raw_pointcloud_size = raw_pointcloud.width * raw_pointcloud.height;
  const size_t obstacle_pointcloud_size = obstacle_pointcloud.width * obstacle_pointcloud.height;

  // Reserve a certain amount of memory in advance for performance reasons
  const size_t raw_reserve_size = raw_pointcloud_size / angle_bin_size;
  const size_t obstacle_reserve_size = obstacle_pointcloud_size / angle_bin_size;
  for (size_t i = 0; i < angle_bin_size; i++) {
    raw_pointcloud_angle_bins[i].reserve(raw_reserve_size);
    obstacle_pointcloud_angle_bins[i].reserve(obstacle_reserve_size);
  }

  // Updated every loop inside transformPointAndCalculate()
  Eigen::Vector4f pt_map;
  int angle_bin_index;
  double range;

  size_t global_offset = 0;
  for (size_t i = 0; i < raw_pointcloud_size; ++i) {
    Eigen::Vector4f pt(
      *reinterpret_cast<const float *>(&raw_pointcloud.data[global_offset + x_offset_raw_]),
      *reinterpret_cast<const float *>(&raw_pointcloud.data[global_offset + y_offset_raw_]),
      *reinterpret_cast<const float *>(&raw_pointcloud.data[global_offset + z_offset_raw_]), 1);
    global_offset += raw_pointcloud.point_step;
    if (!isPointValid(pt)) {
      continue;
    }
    transformPointAndCalculate(pt, pt_map, angle_bin_index, range);

    raw_pointcloud_angle_bins.at(angle_bin_index).emplace_back(range, pt_map[0], pt_map[1]);
  }

  for (auto & raw_pointcloud_angle_bin : raw_pointcloud_angle_bins) {
    std::sort(raw_pointcloud_angle_bin.begin(), raw_pointcloud_angle_bin.end(), [](auto a, auto b) {
      return a.range < b.range;
    });
  }

  // Create obstacle angle bins and sort points by range
  global_offset = 0;
  for (size_t i = 0; i < obstacle_pointcloud_size; ++i) {
    Eigen::Vector4f pt(
      *reinterpret_cast<const float *>(
        &obstacle_pointcloud.data[global_offset + x_offset_obstacle_]),
      *reinterpret_cast<const float *>(
        &obstacle_pointcloud.data[global_offset + y_offset_obstacle_]),
      *reinterpret_cast<const float *>(
        &obstacle_pointcloud.data[global_offset + z_offset_obstacle_]),
      1);
    global_offset += obstacle_pointcloud.point_step;
    if (!isPointValid(pt)) {
      continue;
    }
    transformPointAndCalculate(pt, pt_map, angle_bin_index, range);

    // Ignore obstacle points exceed the range of the raw points
    if (raw_pointcloud_angle_bins.at(angle_bin_index).empty()) {
      continue;  // No raw point in this angle bin
    } else if (range > raw_pointcloud_angle_bins.at(angle_bin_index).back().range) {
      continue;  // Obstacle point exceeds the range of the raw points
    }
    obstacle_pointcloud_angle_bins.at(angle_bin_index).emplace_back(range, pt_map[0], pt_map[1]);
  }

  for (auto & obstacle_pointcloud_angle_bin : obstacle_pointcloud_angle_bins) {
    std::sort(
      obstacle_pointcloud_angle_bin.begin(), obstacle_pointcloud_angle_bin.end(),
      [](auto a, auto b) { return a.range < b.range; });
  }

  // First step: Initialize cells to the final point with freespace
  for (size_t bin_index = 0; bin_index < obstacle_pointcloud_angle_bins.size(); ++bin_index) {
    const auto & raw_pointcloud_angle_bin = raw_pointcloud_angle_bins.at(bin_index);

    BinInfo end_distance;
    if (raw_pointcloud_angle_bin.empty()) {
      continue;
    } else {
      end_distance = raw_pointcloud_angle_bin.back();
    }
    raytrace(
      scan_origin.position.x, scan_origin.position.y, end_distance.wx, end_distance.wy,
      cost_value::FREE_SPACE);
  }

  // Second step: Add unknown cell
  for (size_t bin_index = 0; bin_index < obstacle_pointcloud_angle_bins.size(); ++bin_index) {
    const auto & obstacle_pointcloud_angle_bin = obstacle_pointcloud_angle_bins.at(bin_index);
    const auto & raw_pointcloud_angle_bin = raw_pointcloud_angle_bins.at(bin_index);
    auto raw_distance_iter = raw_pointcloud_angle_bin.begin();
    for (size_t dist_index = 0; dist_index < obstacle_pointcloud_angle_bin.size(); ++dist_index) {
      // Calculate next raw point from obstacle point
      while (raw_distance_iter != raw_pointcloud_angle_bin.end()) {
        if (
          raw_distance_iter->range <
          obstacle_pointcloud_angle_bin.at(dist_index).range + distance_margin_)
          raw_distance_iter++;
        else
          break;
      }

      // There is no point far than the obstacle point.
      const bool no_freespace_point = (raw_distance_iter == raw_pointcloud_angle_bin.end());

      if (dist_index + 1 == obstacle_pointcloud_angle_bin.size()) {
        const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
        if (!no_freespace_point) {
          const auto & target = *raw_distance_iter;
          raytrace(source.wx, source.wy, target.wx, target.wy, cost_value::NO_INFORMATION);
          setCellValue(target.wx, target.wy, cost_value::FREE_SPACE);
        }
        continue;
      }

      auto next_obstacle_point_distance = std::abs(
        obstacle_pointcloud_angle_bin.at(dist_index + 1).range -
        obstacle_pointcloud_angle_bin.at(dist_index).range);
      if (next_obstacle_point_distance <= distance_margin_) {
        continue;
      } else if (no_freespace_point) {
        const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
        const auto & target = obstacle_pointcloud_angle_bin.at(dist_index + 1);
        raytrace(source.wx, source.wy, target.wx, target.wy, cost_value::NO_INFORMATION);
        continue;
      }

      auto next_raw_distance =
        std::abs(obstacle_pointcloud_angle_bin.at(dist_index).range - raw_distance_iter->range);
      if (next_raw_distance < next_obstacle_point_distance) {
        const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
        const auto & target = *raw_distance_iter;
        raytrace(source.wx, source.wy, target.wx, target.wy, cost_value::NO_INFORMATION);
        setCellValue(target.wx, target.wy, cost_value::FREE_SPACE);
        continue;
      } else {
        const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
        const auto & target = obstacle_pointcloud_angle_bin.at(dist_index + 1);
        raytrace(source.wx, source.wy, target.wx, target.wy, cost_value::NO_INFORMATION);
        continue;
      }
    }
  }

  // Third step: Overwrite occupied cell
  for (const auto & obstacle_pointcloud_angle_bin : obstacle_pointcloud_angle_bins) {
    for (size_t dist_index = 0; dist_index < obstacle_pointcloud_angle_bin.size(); ++dist_index) {
      const auto & obstacle_point = obstacle_pointcloud_angle_bin.at(dist_index);
      setCellValue(obstacle_point.wx, obstacle_point.wy, cost_value::LETHAL_OBSTACLE);

      if (dist_index + 1 == obstacle_pointcloud_angle_bin.size()) {
        continue;
      }

      auto next_obstacle_point_distance = std::abs(
        obstacle_pointcloud_angle_bin.at(dist_index + 1).range -
        obstacle_pointcloud_angle_bin.at(dist_index).range);
      if (next_obstacle_point_distance <= distance_margin_) {
        const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
        const auto & target = obstacle_pointcloud_angle_bin.at(dist_index + 1);
        raytrace(source.wx, source.wy, target.wx, target.wy, cost_value::LETHAL_OBSTACLE);
        continue;
      }
    }
  }
}

void OccupancyGridMapFixedBlindSpot::initRosParam(rclcpp::Node & node)
{
  distance_margin_ =
    node.declare_parameter<double>("OccupancyGridMapFixedBlindSpot.distance_margin");
}

}  // namespace costmap_2d
}  // namespace autoware::occupancy_grid_map
