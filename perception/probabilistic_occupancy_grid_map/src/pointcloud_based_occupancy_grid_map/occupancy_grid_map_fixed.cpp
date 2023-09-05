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

#include "pointcloud_based_occupancy_grid_map/occupancy_grid_map_fixed.hpp"

#include "cost_value.hpp"
#include "utils/utils.hpp"

#include <grid_map_costmap_2d/grid_map_costmap_2d.hpp>
#include <pcl_ros/transforms.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <algorithm>
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
  constexpr double min_angle = tier4_autoware_utils::deg2rad(-180.0);
  constexpr double max_angle = tier4_autoware_utils::deg2rad(180.0);
  constexpr double angle_increment = tier4_autoware_utils::deg2rad(0.1);
  const size_t angle_bin_size = ((max_angle - min_angle) / angle_increment) + size_t(1 /*margin*/);

  // Transform from base_link to map frame
  PointCloud2 map_raw_pointcloud, map_obstacle_pointcloud;  // point cloud in map frame
  utils::transformPointcloud(raw_pointcloud, robot_pose, map_raw_pointcloud);
  utils::transformPointcloud(obstacle_pointcloud, robot_pose, map_obstacle_pointcloud);

  // Transform from map frame to scan frame
  PointCloud2 scan_raw_pointcloud, scan_obstacle_pointcloud;      // point cloud in scan frame
  const auto scan2map_pose = utils::getInversePose(scan_origin);  // scan -> map transform pose
  utils::transformPointcloud(map_raw_pointcloud, scan2map_pose, scan_raw_pointcloud);
  utils::transformPointcloud(map_obstacle_pointcloud, scan2map_pose, scan_obstacle_pointcloud);

  // Create angle bins
  struct BinInfo
  {
    BinInfo() = default;
    BinInfo(const double _range, const double _wx, const double _wy)
    : range(_range), wx(_wx), wy(_wy)
    {
    }
    double range;
    double wx;
    double wy;
  };
  std::vector</*angle bin*/ std::vector<BinInfo>> obstacle_pointcloud_angle_bins;
  std::vector</*angle bin*/ std::vector<BinInfo>> raw_pointcloud_angle_bins;
  obstacle_pointcloud_angle_bins.resize(angle_bin_size);
  raw_pointcloud_angle_bins.resize(angle_bin_size);
  for (PointCloud2ConstIterator<float> iter_x(scan_raw_pointcloud, "x"),
       iter_y(scan_raw_pointcloud, "y"), iter_wx(map_raw_pointcloud, "x"),
       iter_wy(map_raw_pointcloud, "y");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_wx, ++iter_wy) {
    const double angle = atan2(*iter_y, *iter_x);
    const int angle_bin_index = (angle - min_angle) / angle_increment;
    raw_pointcloud_angle_bins.at(angle_bin_index)
      .push_back(BinInfo(std::hypot(*iter_y, *iter_x), *iter_wx, *iter_wy));
  }
  for (PointCloud2ConstIterator<float> iter_x(scan_obstacle_pointcloud, "x"),
       iter_y(scan_obstacle_pointcloud, "y"), iter_wx(map_obstacle_pointcloud, "x"),
       iter_wy(map_obstacle_pointcloud, "y");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_wx, ++iter_wy) {
    const double angle = atan2(*iter_y, *iter_x);
    int angle_bin_index = (angle - min_angle) / angle_increment;
    obstacle_pointcloud_angle_bins.at(angle_bin_index)
      .push_back(BinInfo(std::hypot(*iter_y, *iter_x), *iter_wx, *iter_wy));
  }

  // Sort by distance
  for (auto & obstacle_pointcloud_angle_bin : obstacle_pointcloud_angle_bins) {
    std::sort(
      obstacle_pointcloud_angle_bin.begin(), obstacle_pointcloud_angle_bin.end(),
      [](auto a, auto b) { return a.range < b.range; });
  }
  for (auto & raw_pointcloud_angle_bin : raw_pointcloud_angle_bins) {
    std::sort(raw_pointcloud_angle_bin.begin(), raw_pointcloud_angle_bin.end(), [](auto a, auto b) {
      return a.range < b.range;
    });
  }

  // First step: Initialize cells to the final point with freespace
  for (size_t bin_index = 0; bin_index < obstacle_pointcloud_angle_bins.size(); ++bin_index) {
    auto & obstacle_pointcloud_angle_bin = obstacle_pointcloud_angle_bins.at(bin_index);
    auto & raw_pointcloud_angle_bin = raw_pointcloud_angle_bins.at(bin_index);

    BinInfo end_distance;
    if (raw_pointcloud_angle_bin.empty() && obstacle_pointcloud_angle_bin.empty()) {
      continue;
    } else if (raw_pointcloud_angle_bin.empty()) {
      end_distance = obstacle_pointcloud_angle_bin.back();
    } else if (obstacle_pointcloud_angle_bin.empty()) {
      end_distance = raw_pointcloud_angle_bin.back();
    } else {
      end_distance = obstacle_pointcloud_angle_bin.back().range + distance_margin_ <
                         raw_pointcloud_angle_bin.back().range
                       ? raw_pointcloud_angle_bin.back()
                       : obstacle_pointcloud_angle_bin.back();
    }
    raytrace(
      scan_origin.position.x, scan_origin.position.y, end_distance.wx, end_distance.wy,
      occupancy_cost_value::FREE_SPACE);
  }

  // Second step: Add unknown cell
  for (size_t bin_index = 0; bin_index < obstacle_pointcloud_angle_bins.size(); ++bin_index) {
    auto & obstacle_pointcloud_angle_bin = obstacle_pointcloud_angle_bins.at(bin_index);
    auto & raw_pointcloud_angle_bin = raw_pointcloud_angle_bins.at(bin_index);
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
          raytrace(
            source.wx, source.wy, target.wx, target.wy, occupancy_cost_value::NO_INFORMATION);
          setCellValue(target.wx, target.wy, occupancy_cost_value::FREE_SPACE);
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
        raytrace(source.wx, source.wy, target.wx, target.wy, occupancy_cost_value::NO_INFORMATION);
        continue;
      }

      auto next_raw_distance =
        std::abs(obstacle_pointcloud_angle_bin.at(dist_index).range - raw_distance_iter->range);
      if (next_raw_distance < next_obstacle_point_distance) {
        const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
        const auto & target = *raw_distance_iter;
        raytrace(source.wx, source.wy, target.wx, target.wy, occupancy_cost_value::NO_INFORMATION);
        setCellValue(target.wx, target.wy, occupancy_cost_value::FREE_SPACE);
        continue;
      } else {
        const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
        const auto & target = obstacle_pointcloud_angle_bin.at(dist_index + 1);
        raytrace(source.wx, source.wy, target.wx, target.wy, occupancy_cost_value::NO_INFORMATION);
        continue;
      }
    }
  }

  // Third step: Overwrite occupied cell
  for (size_t bin_index = 0; bin_index < obstacle_pointcloud_angle_bins.size(); ++bin_index) {
    auto & obstacle_pointcloud_angle_bin = obstacle_pointcloud_angle_bins.at(bin_index);
    for (size_t dist_index = 0; dist_index < obstacle_pointcloud_angle_bin.size(); ++dist_index) {
      const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
      setCellValue(source.wx, source.wy, occupancy_cost_value::LETHAL_OBSTACLE);

      if (dist_index + 1 == obstacle_pointcloud_angle_bin.size()) {
        continue;
      }

      auto next_obstacle_point_distance = std::abs(
        obstacle_pointcloud_angle_bin.at(dist_index + 1).range -
        obstacle_pointcloud_angle_bin.at(dist_index).range);
      if (next_obstacle_point_distance <= distance_margin_) {
        const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
        const auto & target = obstacle_pointcloud_angle_bin.at(dist_index + 1);
        raytrace(source.wx, source.wy, target.wx, target.wy, occupancy_cost_value::LETHAL_OBSTACLE);
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
