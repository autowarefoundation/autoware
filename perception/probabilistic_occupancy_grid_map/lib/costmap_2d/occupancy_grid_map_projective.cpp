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

#include "autoware/probabilistic_occupancy_grid_map/costmap_2d/occupancy_grid_map_projective.hpp"

#include "autoware/probabilistic_occupancy_grid_map/cost_value/cost_value.hpp"
#include "autoware/probabilistic_occupancy_grid_map/utils/utils.hpp"

#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <grid_map_costmap_2d/grid_map_costmap_2d.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
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
  const size_t angle_bin_size =
    ((max_angle_ - min_angle_) * angle_increment_inv_) + size_t(1 /*margin*/);

  // Transform from base_link to map frame
  mat_map_ = utils::getTransformMatrix(robot_pose);

  const auto scan2map_pose = utils::getInversePose(scan_origin);  // scan -> map transform pose

  // Transform from map frame to scan frame
  mat_scan_ = utils::getTransformMatrix(scan2map_pose);

  if (!offset_initialized_) {
    setFieldOffsets(raw_pointcloud, obstacle_pointcloud);
  }

  // Create angle bins and sort points by range
  struct BinInfo3D
  {
    explicit BinInfo3D(
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

  std::vector</*angle bin*/ std::vector<BinInfo3D>> obstacle_pointcloud_angle_bins(angle_bin_size);
  std::vector</*angle bin*/ std::vector<BinInfo3D>> raw_pointcloud_angle_bins(angle_bin_size);

  const size_t raw_pointcloud_size = raw_pointcloud.width * raw_pointcloud.height;
  const size_t obstacle_pointcloud_size = obstacle_pointcloud.width * obstacle_pointcloud.height;

  // Updated every loop inside transformPointAndCalculate()
  Eigen::Vector4f pt_map;
  int angle_bin_index;
  double range;

  size_t global_offset = 0;
  for (size_t i = 0; i < raw_pointcloud_size; i++) {
    Eigen::Vector4f pt(
      *reinterpret_cast<const float *>(&raw_pointcloud.data[global_offset + x_offset_raw_]),
      *reinterpret_cast<const float *>(&raw_pointcloud.data[global_offset + y_offset_raw_]),
      *reinterpret_cast<const float *>(&raw_pointcloud.data[global_offset + z_offset_raw_]), 1);
    global_offset += raw_pointcloud.point_step;
    if (!isPointValid(pt)) {
      continue;
    }
    transformPointAndCalculate(pt, pt_map, angle_bin_index, range);

    raw_pointcloud_angle_bins.at(angle_bin_index)
      .emplace_back(range, pt_map[0], pt_map[1], pt_map[2]);
  }

  for (auto & raw_pointcloud_angle_bin : raw_pointcloud_angle_bins) {
    std::sort(raw_pointcloud_angle_bin.begin(), raw_pointcloud_angle_bin.end(), [](auto a, auto b) {
      return a.range < b.range;
    });
  }

  // Create obstacle angle bins and sort points by range
  global_offset = 0;
  for (size_t i = 0; i < obstacle_pointcloud_size; i++) {
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
    const double scan_z = scan_origin.position.z - robot_pose.position.z;
    const double obstacle_z = (pt_map[2]) - robot_pose.position.z;
    const double dz = scan_z - obstacle_z;

    // Ignore obstacle points exceed the range of the raw points
    if (raw_pointcloud_angle_bins.at(angle_bin_index).empty()) {
      continue;  // No raw point in this angle bin
    } else if (range > raw_pointcloud_angle_bins.at(angle_bin_index).back().range) {
      continue;  // Obstacle point exceeds the range of the raw points
    }

    if (dz > projection_dz_threshold_) {
      const double ratio = obstacle_z / dz;
      const double projection_length = range * ratio;
      const double projected_wx = (pt_map[0]) + ((pt_map[0]) - scan_origin.position.x) * ratio;
      const double projected_wy = (pt_map[1]) + ((pt_map[1]) - scan_origin.position.y) * ratio;
      obstacle_pointcloud_angle_bins.at(angle_bin_index)
        .emplace_back(
          range, pt_map[0], pt_map[1], pt_map[2], projection_length, projected_wx, projected_wy);
    } else {
      obstacle_pointcloud_angle_bins.at(angle_bin_index)
        .emplace_back(
          range, pt_map[0], pt_map[1], pt_map[2], std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    }
  }

  for (auto & obstacle_pointcloud_angle_bin : obstacle_pointcloud_angle_bins) {
    std::sort(
      obstacle_pointcloud_angle_bin.begin(), obstacle_pointcloud_angle_bin.end(),
      [](auto a, auto b) { return a.range < b.range; });
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
    const auto & raw_pointcloud_angle_bin = raw_pointcloud_angle_bins.at(bin_index);

    BinInfo3D ray_end;
    if (raw_pointcloud_angle_bin.empty()) {
      continue;
    } else {
      ray_end = raw_pointcloud_angle_bin.back();
    }
    raytrace(
      scan_origin.position.x, scan_origin.position.y, ray_end.wx, ray_end.wy,
      cost_value::FREE_SPACE);
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
          cost_value::NO_INFORMATION);
        break;
      }

      if (dist_index + 1 == obstacle_pointcloud_angle_bin.size()) {
        const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
        raytrace(
          source.wx, source.wy, source.projected_wx, source.projected_wy,
          cost_value::NO_INFORMATION);
        continue;
      }

      auto next_obstacle_point_distance = std::abs(
        obstacle_pointcloud_angle_bin.at(dist_index + 1).range -
        obstacle_pointcloud_angle_bin.at(dist_index).range);
      if (next_obstacle_point_distance <= obstacle_separation_threshold_) {
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

  if (pub_debug_grid_) converter.addLayerFromCostmap2D(*this, "added_unknown", debug_grid_);

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
      if (next_obstacle_point_distance <= obstacle_separation_threshold_) {
        const auto & source = obstacle_pointcloud_angle_bin.at(dist_index);
        const auto & target = obstacle_pointcloud_angle_bin.at(dist_index + 1);
        raytrace(source.wx, source.wy, target.wx, target.wy, cost_value::LETHAL_OBSTACLE);
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
}  // namespace autoware::occupancy_grid_map
