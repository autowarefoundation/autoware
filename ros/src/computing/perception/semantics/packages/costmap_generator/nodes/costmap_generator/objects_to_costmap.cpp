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

// headers in standard library
#include <cmath>

// headers in ROS
#include <tf/transform_datatypes.h>

// headers in local directory
#include "costmap_generator/objects_to_costmap.h"

// Constructor
ObjectsToCostmap::ObjectsToCostmap() :
NUMBER_OF_POINTS(4),
NUMBER_OF_DIMENSIONS(2),
OBJECTS_COSTMAP_LAYER_("objects_costmap"),
BLURRED_OBJECTS_COSTMAP_LAYER_("blurred_objects_costmap")
{
}

ObjectsToCostmap::~ObjectsToCostmap()
{
}

Eigen::MatrixXd ObjectsToCostmap::makeRectanglePoints(const autoware_msgs::DetectedObject& in_object,
                                                     const double expand_rectangle_size)
{
  double length = in_object.dimensions.x + expand_rectangle_size;
  double width = in_object.dimensions.y + expand_rectangle_size;
  Eigen::MatrixXd origin_points(NUMBER_OF_DIMENSIONS, NUMBER_OF_POINTS);
  origin_points << length / 2, length / 2, -length / 2, -length / 2, width / 2, -width / 2, -width / 2, width / 2;

  double yaw = tf::getYaw(in_object.pose.orientation);
  Eigen::MatrixXd rotation_matrix(NUMBER_OF_DIMENSIONS, NUMBER_OF_DIMENSIONS);
  rotation_matrix << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  Eigen::MatrixXd rotated_points = rotation_matrix * origin_points;

  double dx = in_object.pose.position.x;
  double dy = in_object.pose.position.y;
  Eigen::MatrixXd transformed_points(NUMBER_OF_DIMENSIONS, NUMBER_OF_POINTS);
  Eigen::MatrixXd ones = Eigen::MatrixXd::Ones(1, NUMBER_OF_POINTS);
  transformed_points.row(0) = rotated_points.row(0) + dx * ones;
  transformed_points.row(1) = rotated_points.row(1) + dy * ones;
  return transformed_points;
}

grid_map::Polygon ObjectsToCostmap::makePolygonFromObjectBox(const autoware_msgs::DetectedObject& in_object,
                                                         const double expand_rectangle_size)
{
  grid_map::Polygon polygon;
  polygon.setFrameId(in_object.header.frame_id);
  Eigen::MatrixXd rectangle_points = makeRectanglePoints(in_object, expand_rectangle_size);
  for (int col = 0; col < rectangle_points.cols(); col++)
  {
    polygon.addVertex(grid_map::Position(rectangle_points(0, col), rectangle_points(1, col)));
  }
  return polygon;
}

geometry_msgs::Point ObjectsToCostmap::makeExpandedPoint(const geometry_msgs::Point& in_centroid,
                                                        const geometry_msgs::Point32& in_corner_point,
                                                        const double expand_polygon_size)
{
  geometry_msgs::Point expanded_point;
  if(expand_polygon_size == 0)
  {
    expanded_point.x = in_corner_point.x;
    expanded_point.y = in_corner_point.y;
    return expanded_point;
  }
  double theta = std::atan2(in_corner_point.y - in_centroid.y, in_corner_point.x - in_centroid.x);
  double delta_x = expand_polygon_size * std::cos(theta);
  double delta_y = expand_polygon_size * std::sin(theta);
  expanded_point.x = in_corner_point.x + delta_x;
  expanded_point.y = in_corner_point.y + delta_y;
  return expanded_point;
}


grid_map::Polygon ObjectsToCostmap::makePolygonFromObjectConvexHull(const autoware_msgs::DetectedObject& in_object,
                                                                   const double expand_polygon_size)
{
  grid_map::Polygon polygon;
  polygon.setFrameId(in_object.header.frame_id);

  double initial_z = in_object.convex_hull.polygon.points[0].z;
  for (size_t index = 0; index < in_object.convex_hull.polygon.points.size(); index++)
  {
    if(in_object.convex_hull.polygon.points[index].z == initial_z)
    {
      geometry_msgs::Point centroid = in_object.pose.position;
      geometry_msgs::Point expanded_point = makeExpandedPoint(centroid,
        in_object.convex_hull.polygon.points[index], expand_polygon_size);
      polygon.addVertex(grid_map::Position(expanded_point.x, expanded_point.y));
    }
  }
  return polygon;
}

void ObjectsToCostmap::setCostInPolygon(const grid_map::Polygon& polygon, const std::string& gridmap_layer_name,
                                       const float score, grid_map::GridMap& objects_costmap)
{
  grid_map::PolygonIterator iterators(objects_costmap, polygon);
  for (grid_map::PolygonIterator iterator(objects_costmap, polygon); !iterator.isPastEnd(); ++iterator)
  {
    const float current_score = objects_costmap.at(gridmap_layer_name, *iterator);
    if (score > current_score)
    {
      objects_costmap.at(gridmap_layer_name, *iterator) = score;
    }
  }
}

grid_map::Matrix ObjectsToCostmap::makeCostmapFromObjects(const grid_map::GridMap& costmap,
                                                         const double expand_polygon_size,
                                                         const double size_of_expansion_kernel,
                                                         const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects,
                                                         const bool use_objects_convex_hull)
{
  grid_map::GridMap objects_costmap = costmap;
  objects_costmap.add(OBJECTS_COSTMAP_LAYER_, 0);
  objects_costmap.add(BLURRED_OBJECTS_COSTMAP_LAYER_, 0);

  for (const auto& object : in_objects->objects)
  {
    grid_map::Polygon polygon, expanded_polygon;
    if(use_objects_convex_hull)
    {
      expanded_polygon = makePolygonFromObjectConvexHull(object, expand_polygon_size);
    }
    else
    {
      expanded_polygon = makePolygonFromObjectBox(object, expand_polygon_size);
    }
    setCostInPolygon(expanded_polygon, OBJECTS_COSTMAP_LAYER_, object.score, objects_costmap);
    setCostInPolygon(expanded_polygon, BLURRED_OBJECTS_COSTMAP_LAYER_, object.score, objects_costmap);
  }
  // Applying mean filter to expanded gridmap
  const grid_map::SlidingWindowIterator::EdgeHandling edge_handling =
      grid_map::SlidingWindowIterator::EdgeHandling::CROP;
  for (grid_map::SlidingWindowIterator iterator(objects_costmap, BLURRED_OBJECTS_COSTMAP_LAYER_, edge_handling,
                                                size_of_expansion_kernel);
       !iterator.isPastEnd(); ++iterator)
  {
    objects_costmap.at(BLURRED_OBJECTS_COSTMAP_LAYER_, *iterator) =
        iterator.getData().meanOfFinites();  // Blurring.
  }

  objects_costmap[OBJECTS_COSTMAP_LAYER_] =
      objects_costmap[OBJECTS_COSTMAP_LAYER_].cwiseMax(objects_costmap[BLURRED_OBJECTS_COSTMAP_LAYER_]);

  return objects_costmap[OBJECTS_COSTMAP_LAYER_];
}
