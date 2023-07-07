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

#ifndef POINTCLOUD_BASED_OCCUPANCY_GRID_MAP__OCCUPANCY_GRID_MAP_BASE_HPP_
#define POINTCLOUD_BASED_OCCUPANCY_GRID_MAP__OCCUPANCY_GRID_MAP_BASE_HPP_

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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

private:
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;

  rclcpp::Logger logger_{rclcpp::get_logger("pointcloud_based_occupancy_grid_map")};
  rclcpp::Clock clock_{RCL_ROS_TIME};
};

}  // namespace costmap_2d

#endif  // POINTCLOUD_BASED_OCCUPANCY_GRID_MAP__OCCUPANCY_GRID_MAP_BASE_HPP_
