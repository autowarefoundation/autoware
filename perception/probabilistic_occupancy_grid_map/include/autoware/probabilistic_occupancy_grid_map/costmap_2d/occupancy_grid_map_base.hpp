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
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#ifndef AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_BASE_HPP_
#define AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_BASE_HPP_

#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace autoware::occupancy_grid_map
{
namespace costmap_2d
{
using geometry_msgs::msg::Pose;
using sensor_msgs::msg::PointCloud2;

class OccupancyGridMapInterface : public nav2_costmap_2d::Costmap2D
{
public:
  OccupancyGridMapInterface(
    const unsigned int cells_size_x, const unsigned int cells_size_y, const float resolution);

  virtual void updateWithPointCloud(
    const PointCloud2 & raw_pointcloud, const PointCloud2 & obstacle_pointcloud,
    const Pose & robot_pose, const Pose & scan_origin) = 0;

  void updateOrigin(double new_origin_x, double new_origin_y) override;
  void raytrace(
    const double source_x, const double source_y, const double target_x, const double target_y,
    const unsigned char cost);
  void setCellValue(const double wx, const double wy, const unsigned char cost);
  using nav2_costmap_2d::Costmap2D::resetMaps;

  virtual void initRosParam(rclcpp::Node & node) = 0;

  void setHeightLimit(const double min_height, const double max_height);

  double min_height_;
  double max_height_;

  void setFieldOffsets(const PointCloud2 & input_raw, const PointCloud2 & input_obstacle);

  int x_offset_raw_;
  int y_offset_raw_;
  int z_offset_raw_;
  int x_offset_obstacle_;
  int y_offset_obstacle_;
  int z_offset_obstacle_;
  bool offset_initialized_;

  const double min_angle_ = autoware::universe_utils::deg2rad(-180.0);
  const double max_angle_ = autoware::universe_utils::deg2rad(180.0);
  const double angle_increment_inv_ = 1.0 / autoware::universe_utils::deg2rad(0.1);

  Eigen::Matrix4f mat_map_, mat_scan_;

  bool isPointValid(const Eigen::Vector4f & pt) const
  {
    // Apply height filter and exclude invalid points
    return min_height_ < pt[2] && pt[2] < max_height_ && std::isfinite(pt[0]) &&
           std::isfinite(pt[1]) && std::isfinite(pt[2]);
  }
  // Transform pt to (pt_map, pt_scan), then calculate angle_bin_index and range
  void transformPointAndCalculate(
    const Eigen::Vector4f & pt, Eigen::Vector4f & pt_map, int & angle_bin_index,
    double & range) const
  {
    pt_map = mat_map_ * pt;
    Eigen::Vector4f pt_scan(mat_scan_ * pt_map);
    const double angle = atan2(pt_scan[1], pt_scan[0]);
    angle_bin_index = (angle - min_angle_) * angle_increment_inv_;
    range = std::sqrt(pt_scan[1] * pt_scan[1] + pt_scan[0] * pt_scan[0]);
  }

private:
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;

  rclcpp::Logger logger_{rclcpp::get_logger("pointcloud_based_occupancy_grid_map")};
  rclcpp::Clock clock_{RCL_ROS_TIME};

  double resolution_inv_;
};

}  // namespace costmap_2d
}  // namespace autoware::occupancy_grid_map

#endif  // AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__COSTMAP_2D__OCCUPANCY_GRID_MAP_BASE_HPP_
