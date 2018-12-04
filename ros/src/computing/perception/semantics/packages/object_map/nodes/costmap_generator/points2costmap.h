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
#ifndef POINTS2COSTMAP_H
#define POINTS2COSTMAP_H

// headers in ROS
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>

// headers in PCL
#include <pcl_ros/point_cloud.h>


class Points2Costmap
{
  public:
    Points2Costmap();
    ~Points2Costmap();

    grid_map::GridMap makeSensorPointsCostmap(const double maximum_height_thres,
                                              const grid_map::GridMap& gridmap,
                                              const std::string& gridmap_layer_name,
                                              const sensor_msgs::PointCloud2::ConstPtr& in_sensor_points);

  private:
    double grid_length_x_;
    double grid_length_y_;
    double grid_resolution_;
    double grid_position_x_;
    double grid_position_y_;
    double y_cell_size_;
    double x_cell_size_;

    bool isValidInd(const grid_map::Index& grid_ind);
    grid_map::Index fetchGridIndexFromPoint(const pcl::PointXYZ& point);
    std::vector<std::vector<std::vector<double>>> assignPoints2GridCell(const grid_map::GridMap& gridmap,
                                                                        const sensor_msgs::PointCloud2::ConstPtr& in_sensor_points);
    grid_map::Matrix calculateCostmap(const double maximum_height_thres,
                                      const grid_map::GridMap& gridmap, const std::string& gridmap_layer_name,
                                      const std::vector<std::vector<std::vector<double>>> grid_vec);
};

#endif  // POINTS2COSTMAP_H
