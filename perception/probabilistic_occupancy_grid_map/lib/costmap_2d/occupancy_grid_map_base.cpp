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

#include "autoware/probabilistic_occupancy_grid_map/costmap_2d/occupancy_grid_map_base.hpp"

#include "autoware/probabilistic_occupancy_grid_map/cost_value/cost_value.hpp"
#include "autoware/probabilistic_occupancy_grid_map/utils/utils.hpp"

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

OccupancyGridMapInterface::OccupancyGridMapInterface(
  const unsigned int cells_size_x, const unsigned int cells_size_y, const float resolution)
: Costmap2D(cells_size_x, cells_size_y, resolution, 0.f, 0.f, cost_value::NO_INFORMATION)
{
  min_height_ = -std::numeric_limits<double>::infinity();
  max_height_ = std::numeric_limits<double>::infinity();
  resolution_inv_ = 1.0 / resolution_;
  offset_initialized_ = false;
}

inline bool OccupancyGridMapInterface::worldToMap(
  double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }

  mx = static_cast<int>(std::floor((wx - origin_x_) * resolution_inv_));
  my = static_cast<int>(std::floor((wy - origin_y_) * resolution_inv_));

  if (mx < size_x_ && my < size_y_) {
    return true;
  }

  return false;
}

void OccupancyGridMapInterface::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  int cell_ox{static_cast<int>(std::floor((new_origin_x - origin_x_) / resolution_))};
  int cell_oy{static_cast<int>(std::floor((new_origin_y - origin_y_) / resolution_))};

  // compute the associated world coordinates for the origin cell
  // because we want to keep things grid-aligned
  double new_grid_ox{origin_x_ + cell_ox * resolution_};
  double new_grid_oy{origin_y_ + cell_oy * resolution_};

  // To save casting from unsigned int to int a bunch of times
  int size_x{static_cast<int>(size_x_)};
  int size_y{static_cast<int>(size_y_)};

  // we need to compute the overlap of the new and existing windows
  int lower_left_x{std::min(std::max(cell_ox, 0), size_x)};
  int lower_left_y{std::min(std::max(cell_oy, 0), size_y)};
  int upper_right_x{std::min(std::max(cell_ox + size_x, 0), size_x)};
  int upper_right_y{std::min(std::max(cell_oy + size_y, 0), size_y)};

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  // we need a map to store the obstacles in the window temporarily
  unsigned char * local_map = new unsigned char[cell_size_x * cell_size_y];

  // copy the local window in the costmap to the local map
  copyMapRegion(
    costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x,
    cell_size_y);

  // now we'll set the costmap to be completely unknown if we track unknown space
  resetMaps();

  // update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  // compute the starting cell location for copying data back in
  int start_x{lower_left_x - cell_ox};
  int start_y{lower_left_y - cell_oy};

  // now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(
    local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);

  // make sure to clean up
  delete[] local_map;
}

void OccupancyGridMapInterface::setCellValue(
  const double wx, const double wy, const unsigned char cost)
{
  MarkCell marker(costmap_, cost);
  unsigned int mx{};
  unsigned int my{};
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_DEBUG(logger_, "Computing map coords failed");
    return;
  }
  const unsigned int index = getIndex(mx, my);
  marker(index);
}

void OccupancyGridMapInterface::raytrace(
  const double source_x, const double source_y, const double target_x, const double target_y,
  const unsigned char cost)
{
  unsigned int x0{};
  unsigned int y0{};
  const double ox{source_x};
  const double oy{source_y};
  if (!worldToMap(ox, oy, x0, y0)) {
    RCLCPP_DEBUG(
      logger_,
      "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot "
      "raytrace for it.",
      ox, oy);
    return;
  }

  // we can pre-compute the endpoints of the map outside of the inner loop... we'll need these later
  const double origin_x = origin_x_, origin_y = origin_y_;
  const double map_end_x = origin_x + size_x_ * resolution_;
  const double map_end_y = origin_y + size_y_ * resolution_;

  double wx = target_x;
  double wy = target_y;

  // now we also need to make sure that the endpoint we're ray-tracing
  // to isn't off the costmap and scale if necessary
  const double a = wx - ox;
  const double b = wy - oy;

  // the minimum value to raytrace from is the origin
  if (wx < origin_x) {
    const double t = (origin_x - ox) / a;
    wx = origin_x;
    wy = oy + b * t;
  }
  if (wy < origin_y) {
    const double t = (origin_y - oy) / b;
    wx = ox + a * t;
    wy = origin_y;
  }

  // the maximum value to raytrace to is the end of the map
  if (wx > map_end_x) {
    const double t = (map_end_x - ox) / a;
    wx = map_end_x - .001;
    wy = oy + b * t;
  }
  if (wy > map_end_y) {
    const double t = (map_end_y - oy) / b;
    wx = ox + a * t;
    wy = map_end_y - .001;
  }

  // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
  unsigned int x1{};
  unsigned int y1{};

  // check for legality just in case
  if (!worldToMap(wx, wy, x1, y1)) {
    return;
  }

  constexpr unsigned int cell_raytrace_range = 10000;  // large number to ignore range threshold
  MarkCell marker(costmap_, cost);
  raytraceLine(marker, x0, y0, x1, y1, cell_raytrace_range);
}

void OccupancyGridMapInterface::setHeightLimit(const double min_height, const double max_height)
{
  min_height_ = min_height;
  max_height_ = max_height;
}

void OccupancyGridMapInterface::setFieldOffsets(
  const PointCloud2 & input_raw, const PointCloud2 & input_obstacle)
{
  x_offset_raw_ = input_raw.fields[pcl::getFieldIndex(input_raw, "x")].offset;
  y_offset_raw_ = input_raw.fields[pcl::getFieldIndex(input_raw, "y")].offset;
  z_offset_raw_ = input_raw.fields[pcl::getFieldIndex(input_raw, "z")].offset;
  x_offset_obstacle_ = input_obstacle.fields[pcl::getFieldIndex(input_obstacle, "x")].offset;
  y_offset_obstacle_ = input_obstacle.fields[pcl::getFieldIndex(input_obstacle, "y")].offset;
  z_offset_obstacle_ = input_obstacle.fields[pcl::getFieldIndex(input_obstacle, "z")].offset;
  offset_initialized_ = true;
}

}  // namespace costmap_2d
}  // namespace autoware::occupancy_grid_map
