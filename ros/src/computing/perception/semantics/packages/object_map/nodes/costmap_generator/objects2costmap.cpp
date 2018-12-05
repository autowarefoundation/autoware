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

// headers in ROS
#include <tf/transform_datatypes.h>

// headers in local directory
#include "objects2costmap.h"


// Constructor
Objects2Costmap::Objects2Costmap():
  NUMBER_OF_POINTS(4),
  NUMBER_OF_DIMENSIONS(2)
{
}

Objects2Costmap::~Objects2Costmap() {}

Eigen::MatrixXd Objects2Costmap::makeRectanglePoints(const autoware_msgs::DetectedObject& in_object)
{
  double length = in_object.dimensions.x;
  double width = in_object.dimensions.y;
  Eigen::MatrixXd origin_points(NUMBER_OF_DIMENSIONS, NUMBER_OF_POINTS);
  origin_points << length/2, length/2, -length/2, -length/2,
                   width/2,  -width/2,  -width/2, width/2;

  double yaw = tf::getYaw(in_object.pose.orientation);
  Eigen::MatrixXd rotation_matrix(NUMBER_OF_DIMENSIONS, NUMBER_OF_DIMENSIONS);
  rotation_matrix << std::cos(yaw), -std::sin(yaw),
                     std::sin(yaw),  std::cos(yaw);
  Eigen::MatrixXd rotated_points = rotation_matrix * origin_points;

  double dx = in_object.pose.position.x;
  double dy = in_object.pose.position.y;
  Eigen::MatrixXd transformed_points(NUMBER_OF_DIMENSIONS, NUMBER_OF_POINTS);
  Eigen::MatrixXd ones = Eigen::MatrixXd::Ones(1, NUMBER_OF_POINTS);
  transformed_points.row(0) = rotated_points.row(0) + dx*ones;
  transformed_points.row(1) = rotated_points.row(1) + dy*ones;
  return transformed_points;
}

grid_map::Polygon Objects2Costmap::makePolygonFromObject(const autoware_msgs::DetectedObject& in_object)
{
  //forward_right, forward_left, backward_right, backward_left
  grid_map::Position forward_right;
  grid_map::Polygon polygon;
  polygon.setFrameId(in_object.header.frame_id);
  Eigen::MatrixXd rectangle_points = makeRectanglePoints(in_object);
  for(int col = 0; col < rectangle_points.cols(); col++)
  {
    polygon.addVertex(grid_map::Position(rectangle_points(0, col), rectangle_points(1, col)));
  }
  return polygon;
}

void Objects2Costmap::setCostForPolygon(const grid_map::Polygon& polygon,const std::string& gridmap_layer_name,
                       grid_map::GridMap& objects_costmap)
{
  for (grid_map::PolygonIterator iterator(objects_costmap, polygon);
    !iterator.isPastEnd(); ++iterator)
  {
    // change cost depending on object.score?
    objects_costmap.at(gridmap_layer_name, *iterator) = 2.0;
  }
}

grid_map::Matrix Objects2Costmap::makeCostmapFromObjects(const grid_map::GridMap& costmap,
                                                            const std::string& gridmap_layer_name,
                                                            const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects)
{
  grid_map::GridMap objects_costmap = costmap;
  objects_costmap[gridmap_layer_name].setConstant(0.0);
  for (const auto& object: in_objects->objects)
  {
    grid_map::Polygon polygon = makePolygonFromObject(object);
    setCostForPolygon(polygon, gridmap_layer_name, objects_costmap);
  }
  return objects_costmap[gridmap_layer_name];
}
