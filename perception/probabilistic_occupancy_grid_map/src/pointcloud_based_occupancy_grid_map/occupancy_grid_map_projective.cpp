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

#include "pointcloud_based_occupancy_grid_map/occupancy_grid_map_projective.hpp"

#include "cost_value.hpp"
#include "utils/utils.hpp"

#include <grid_map_costmap_2d/grid_map_costmap_2d.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
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

OccupancyGridMapProjectiveBlindSpot::OccupancyGridMapProjectiveBlindSpot(
  const unsigned int cells_size_x, const unsigned int cells_size_y, const float resolution)
: OccupancyGridMapInterface(cells_size_x, cells_size_y, resolution)
{
}

/**
 * @brief update Gridmap with PointCloud in 3D manner
 *
 * @param raw_pointcloud raw point cloud on a certain frame (usually base_link)
 * @param obstacle_pointcloud raw point cloud on a certain frame (usually base_link)
 * @param robot_pose frame of the input point cloud (usually base_link)
 * @param scan_origin manually chosen grid map origin frame
 */
void OccupancyGridMapProjectiveBlindSpot::updateWithPointCloud(
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
  struct BinInfo3D
  {
    BinInfo3D(
      const double _range = 0.0, const double _wx = 0.0, const double _wy = 0.0,
      const double _wz = 0.0, const double _projection_length = 0.0,
      const double _projected_wx = 0.0, const double _projected_wy = 0.0)
    : range(_range),
      wx(_wx),
      wy(_wy),
      wz(_wz),
      projection_length(_projection_length),
      projected_wx(_projected_wx),
      projected_wy(_projected_wy)
    {
    }
    double range;
    double wx;
    double wy;
    double wz;
    double projection_length;
    double projected_wx;
    double projected_wy;
  };

  std::vector</*angle bin*/ std::vector<BinInfo3D>> obstacle_pointcloud_angle_bins;
  std::vector</*angle bin*/ std::vector<BinInfo3D>> raw_pointcloud_angle_bins;
  obstacle_pointcloud_angle_bins.resize(angle_bin_size);
  raw_pointcloud_angle_bins.resize(angle_bin_size);
  for (PointCloud2ConstIterator<float> iter_x(scan_raw_pointcloud, "x"),
       iter_y(scan_raw_pointcloud, "y"), iter_wx(map_raw_pointcloud, "x"),
       iter_wy(map_raw_pointcloud, "y"), iter_wz(map_raw_pointcloud, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_wx, ++iter_wy, ++iter_wz) {
    const double angle = atan2(*iter_y, *iter_x);
    const int angle_bin_index = (angle - min_angle) / angle_increment;
    raw_pointcloud_angle_bins.at(angle_bin_index)
      .emplace_back(std::hypot(*iter_y, *iter_x), *iter_wx, *iter_wy, *iter_wz);
  }
  for (PointCloud2ConstIterator<float> iter_x(scan_obstacle_pointcloud, "x"),
       iter_y(scan_obstacle_pointcloud, "y"), iter_z(scan_obstacle_pointcloud, "z"),
       iter_wx(map_obstacle_pointcloud, "x"), iter_wy(map_obstacle_pointcloud, "y"),
       iter_wz(map_obstacle_pointcloud, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_wx, ++iter_wy, ++iter_wz) {
    const double scan_z = scan_origin.position.z - robot_pose.position.z;
    const double obstacle_z = (*iter_wz) - robot_pose.position.z;
    const double dz = scan_z - obstacle_z;
    const double angle = atan2(*iter_y, *iter_x);
    const int angle_bin_index = (angle - min_angle) / angle_increment;
    const double range = std::hypot(*iter_x, *iter_y);
    if (dz > projection_dz_threshold_) {
      const double ratio = obstacle_z / dz;
      const double projection_length = range * ratio;
      const double projected_wx = (*iter_wx) + ((*iter_wx) - scan_origin.position.x) * ratio;
      const double projected_wy = (*iter_wy) + ((*iter_wy) - scan_origin.position.y) * ratio;
      obstacle_pointcloud_angle_bins.at(angle_bin_index)
        .emplace_back(
          range, *iter_wx, *iter_wy, *iter_wz, projection_length, projected_wx, projected_wy);
    } else {
      obstacle_pointcloud_angle_bins.at(angle_bin_index)
        .emplace_back(
          range, *iter_wx, *iter_wy, *iter_wz, std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    }
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

  grid_map::Costmap2DConverter<grid_map::GridMap> converter;
  if (pub_debug_grid_) {
    debug_grid_.clearAll();
    debug_grid_.setFrameId("map");
    debug_grid_.setGeometry(
      grid_map::Length(size_x_ * resolution_, size_y_ * resolution_), resolution_,
      grid_map::Position(
        origin_x_ + size_x_ * resolution_ / 2.0, origin_y_ + size_y_ * resolution_ / 2.0));
  }

  auto is_visible_beyond_obstacle = [&](const BinInfo3D & obstacle, const BinInfo3D & raw) -> bool {
    if (raw.range < obstacle.range) {
      return false;
    }

    if (std::isinf(obstacle.projection_length)) {
      return false;
    }

    // y = ax + b
    const double a = -(scan_origin.position.z - robot_pose.position.z) /
                     (obstacle.range + obstacle.projection_length);
    const double b = scan_origin.position.z;
    return raw.wz > (a * raw.range + b);
  };

  // First step: Initialize cells to the final point with freespace
  for (size_t bin_index = 0; bin_index < obstacle_pointcloud_angle_bins.size(); ++bin_index) {
    const auto & obstacle_pointcloud_angle_bin = obstacle_pointcloud_angle_bins.at(bin_index);
    const auto & raw_pointcloud_angle_bin = raw_pointcloud_angle_bins.at(bin_index);

    BinInfo3D ray_end;
    if (raw_pointcloud_angle_bin.empty() && obstacle_pointcloud_angle_bin.empty()) {
      continue;
    } else if (raw_pointcloud_angle_bin.empty()) {
      ray_end = obstacle_pointcloud_angle_bin.back();
    } else if (obstacle_pointcloud_angle_bin.empty()) {
      ray_end = raw_pointcloud_angle_bin.back();
    } else {
      const auto & farthest_obstacle_this_bin = obstacle_pointcloud_angle_bin.back();
      const auto & farthest_raw_this_bin = raw_pointcloud_angle_bin.back();
      ray_end = is_visible_beyond_obstacle(farthest_obstacle_this_bin, farthest_raw_this_bin)
                  ? farthest_raw_this_bin
                  : farthest_obstacle_this_bin;
    }
    raytrace(
      scan_origin.position.x, scan_origin.position.y, ray_end.wx, ray_end.wy,
      occupancy_cost_value::FREE_SPACE);
  }

  if (pub_debug_grid_)
    converter.addLayerFromCostmap2D(*this, "filled_free_to_farthest", debug_grid_);

  // Second step: Add unknown cell
  for (size_t bin_index = 0; bin_index < obstacle_pointcloud_angle_bins.size(); ++bin_index) {
    const auto & obstacle_pointcloud_angle_bin = obstacle_pointcloud_angle_bins.at(bin_index);
    const auto & raw_pointcloud_angle_bin = raw_pointcloud_angle_bins.at(bin_index);
    auto raw_distance_iter = raw_pointcloud_angle_bin.begin();
    for (size_t dist_index = 0; dist_index < obstacle_pointcloud_angle_bin.size(); ++dist_index) {
      // Calculate next raw point from obstacle point
      const auto & obstacle_bin = obstacle_pointcloud_angle_bin.at(dist_index);
      while (raw_distance_iter != raw_pointcloud_angle_bin.end()) {
        if (!is_visible_beyond_obstacle(obstacle_bin, *raw_distance_iter))
          raw_distance_iter++;
        else
          break;
      }

      // There is no point farther than the obstacle point.
      const bool no_visible_point_beyond = (raw_distance_iter == raw_pointcloud_angle_bin.end());
      if (no_visible_point_beyond) {
        const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
        raytrace(
          source.wx, source.wy, source.projected_wx, source.projected_wy,
          occupancy_cost_value::NO_INFORMATION);
        break;
      }

      if (dist_index + 1 == obstacle_pointcloud_angle_bin.size()) {
        const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
        if (!no_visible_point_beyond) {
          raytrace(
            source.wx, source.wy, source.projected_wx, source.projected_wy,
            occupancy_cost_value::NO_INFORMATION);
        }
        continue;
      }

      auto next_obstacle_point_distance = std::abs(
        obstacle_pointcloud_angle_bin.at(dist_index + 1).range -
        obstacle_pointcloud_angle_bin.at(dist_index).range);
      if (next_obstacle_point_distance <= obstacle_separation_threshold_) {
        continue;
      } else if (no_visible_point_beyond) {
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

  if (pub_debug_grid_) converter.addLayerFromCostmap2D(*this, "added_unknown", debug_grid_);

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
      if (next_obstacle_point_distance <= obstacle_separation_threshold_) {
        const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
        const auto & target = obstacle_pointcloud_angle_bin.at(dist_index + 1);
        raytrace(source.wx, source.wy, target.wx, target.wy, occupancy_cost_value::LETHAL_OBSTACLE);
        continue;
      }
    }
  }

  if (pub_debug_grid_) converter.addLayerFromCostmap2D(*this, "added_obstacle", debug_grid_);
  if (pub_debug_grid_) {
    debug_grid_map_publisher_ptr_->publish(grid_map::GridMapRosConverter::toMessage(debug_grid_));
  }
}

void OccupancyGridMapProjectiveBlindSpot::initRosParam(rclcpp::Node & node)
{
  projection_dz_threshold_ =
    node.declare_parameter<double>("OccupancyGridMapProjectiveBlindSpot.projection_dz_threshold");
  obstacle_separation_threshold_ = node.declare_parameter<double>(
    "OccupancyGridMapProjectiveBlindSpot.obstacle_separation_threshold");
  pub_debug_grid_ =
    node.declare_parameter<bool>("OccupancyGridMapProjectiveBlindSpot.pub_debug_grid");
  debug_grid_map_publisher_ptr_ = node.create_publisher<grid_map_msgs::msg::GridMap>(
    "~/debug/grid_map", rclcpp::QoS(1).durability_volatile());
}

}  // namespace costmap_2d
