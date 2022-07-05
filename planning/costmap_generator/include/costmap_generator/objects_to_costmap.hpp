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

#ifndef COSTMAP_GENERATOR__OBJECTS_TO_COSTMAP_HPP_
#define COSTMAP_GENERATOR__OBJECTS_TO_COSTMAP_HPP_

#include <grid_map_ros/grid_map_ros.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <string>

class ObjectsToCostmap
{
public:
  ObjectsToCostmap();

  /// \brief calculate cost from PredictedObjects
  /// \param[in] costmap: initialized gridmap
  /// \param[in] expand_polygon_size: expand object's costmap polygon
  /// \param[in] size_of_expansion_kernel: kernel size for blurring cost
  /// \param[in] in_objects: subscribed PredictedObjects
  /// \param[out] calculated cost in grid_map::Matrix format
  grid_map::Matrix makeCostmapFromObjects(
    const grid_map::GridMap & costmap, const double expand_polygon_size,
    const double size_of_expansion_kernel,
    const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr in_objects);

private:
  const int NUMBER_OF_POINTS;
  const int NUMBER_OF_DIMENSIONS;
  const std::string OBJECTS_COSTMAP_LAYER_;
  const std::string BLURRED_OBJECTS_COSTMAP_LAYER_;

  /// \brief make 4 rectangle points from centroid position and orientation
  /// \param[in] in_object: subscribed one of PredictedObjects
  /// \param[in] expand_rectangle_size: expanding 4 points
  /// \param[out] 4 rectangle points
  Eigen::MatrixXd makeRectanglePoints(
    const autoware_auto_perception_msgs::msg::PredictedObject & in_object,
    const double expand_rectangle_size);

  /// \brief make polygon(grid_map::Polygon) from 4 rectangle's points
  /// \param[in] in_object: subscribed one of PredictedObjects
  /// \param[in] expand_rectangle_size: expanding 4 points
  /// \param[out] polygon with 4 rectangle points
  grid_map::Polygon makePolygonFromObjectBox(
    const std_msgs::msg::Header & header,
    const autoware_auto_perception_msgs::msg::PredictedObject & in_object,
    const double expand_rectangle_size);

  /// \brief make expanded point from convex hull's point
  /// \param[in] in_centroid: object's centroid
  /// \param[in] in_corner_point one of convex hull points
  /// \param[in] expand_polygon_size  the param for expanding convex_hull points
  /// \param[out] expanded point
  geometry_msgs::msg::Point makeExpandedPoint(
    const geometry_msgs::msg::Point & in_centroid,
    const geometry_msgs::msg::Point32 & in_corner_point, const double expand_polygon_size);

  /// \brief make polygon(grid_map::Polygon) from convex hull points
  /// \param[in] in_centroid: object's centroid
  /// \param[in] expand_polygon_size: expanding convex_hull points
  /// \param[out] polygon object with convex hull points
  grid_map::Polygon makePolygonFromObjectConvexHull(
    const std_msgs::msg::Header & header,
    const autoware_auto_perception_msgs::msg::PredictedObject & in_object,
    const double expand_polygon_size);

  /// \brief set cost in polygon by using DynamicObject's score
  /// \param[in] polygon: 4 rectangle points in polygon format
  /// \param[in] gridmap_layer_name: target gridmap layer name for calculated cost
  /// \param[in] score: set score as a cost for costmap
  /// \param[in] objects_costmap: update cost in this objects_costmap[gridmap_layer_name]
  void setCostInPolygon(
    const grid_map::Polygon & polygon, const std::string & gridmap_layer_name, const float score,
    grid_map::GridMap & objects_costmap);
};

#endif  // COSTMAP_GENERATOR__OBJECTS_TO_COSTMAP_HPP_
