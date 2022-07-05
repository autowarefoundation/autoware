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

/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#ifndef COSTMAP_GENERATOR__POINTS_TO_COSTMAP_HPP_
#define COSTMAP_GENERATOR__POINTS_TO_COSTMAP_HPP_

#include <grid_map_ros/grid_map_ros.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <vector>

class PointsToCostmap
{
public:
  /// \brief calculate cost from sensor points
  /// \param[in] maximum_height_thres: Maximum height threshold for pointcloud data
  /// \param[in] minimum_height_thres: Minimum height threshold for pointcloud data
  /// \param[in] grid_min_value: Minimum cost for costmap
  /// \param[in] grid_max_value: Maximum cost fot costmap
  /// \param[in] gridmap: costmap based on gridmap
  /// \param[in] gridmap_layer_name: gridmap layer name for gridmap
  /// \param[in] in_sensor_points: subscribed pointcloud
  /// \param[out] calculated cost in grid_map::Matrix format
  grid_map::Matrix makeCostmapFromPoints(
    const double maximum_height_thres, const double minimum_height_thres,
    const double grid_min_value, const double grid_max_value, const grid_map::GridMap & gridmap,
    const std::string & gridmap_layer_name,
    const pcl::PointCloud<pcl::PointXYZ> & in_sensor_points);

private:
  double grid_length_x_;
  double grid_length_y_;
  double grid_resolution_;
  double grid_position_x_;
  double grid_position_y_;
  double y_cell_size_;
  double x_cell_size_;

  /// \brief initialize gridmap parameters
  /// \param[in] gridmap: gridmap object to be initialized
  void initGridmapParam(const grid_map::GridMap & gridmap);

  /// \brief check if index is valid in the gridmap
  /// \param[in] grid_ind: grid index corresponding with one of pointcloud
  /// \param[out] bool: true if index is valid
  bool isValidInd(const grid_map::Index & grid_ind);

  /// \brief Get index from one of pointcloud
  /// \param[in] point: one of subscribed pointcloud
  /// \param[out] index in gridmap
  grid_map::Index fetchGridIndexFromPoint(const pcl::PointXYZ & point);

  /// \brief Assign pointcloud to appropriate cell in gridmap
  /// \param[in] in_sensor_points: subscribed pointcloud
  /// \param[out] grid-x-length x grid-y-length size grid stuffed with point's height in
  /// corresponding grid cell
  std::vector<std::vector<std::vector<double>>> assignPoints2GridCell(
    const pcl::PointCloud<pcl::PointXYZ> & in_sensor_points);

  /// \brief calculate costmap from subscribed pointcloud
  /// \param[in] maximum_height_thres: Maximum height threshold for pointcloud data
  /// \param[in] minimum_height_thres: Minimum height threshold for pointcloud data
  /// \param[in] grid_min_value: Minimum cost for costmap
  /// \param[in] grid_max_value: Maximum cost fot costmap
  /// \param[in] gridmap: costmap based on gridmap
  /// \param[in] gridmap_layer_name: gridmap layer name for gridmap
  /// \param[in] grid_vec: grid-x-length x grid-y-length size grid stuffed with point's height in
  /// corresponding grid cell \param[out] calculated costmap in grid_map::Matrix format
  grid_map::Matrix calculateCostmap(
    const double maximum_height_thres, const double minimum_lidar_height_thres,
    const double grid_min_value, const double grid_max_value, const grid_map::GridMap & gridmap,
    const std::string & gridmap_layer_name,
    const std::vector<std::vector<std::vector<double>>> grid_vec);
};

#endif  // COSTMAP_GENERATOR__POINTS_TO_COSTMAP_HPP_
