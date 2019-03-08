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


#include "costmap_generator/costmap_generator.h"

class TestClass
{
private:
  const double dummy_grid_length_x_;
  const double dummy_grid_length_y_;
  const double dummy_grid_resolution_;
  const double dummy_grid_position_x_;
  const double dummy_grid_position_y_;
  const std::string dummy_layer_name_;
  const double dummy_initialize_cost_;
public:
  TestClass();

  Points2Costmap *points2costmap_;

  grid_map::GridMap makeDummyCostmap();

  std::vector<std::vector<std::vector<double>>> makeDummy3DVec();

  autoware_msgs::DetectedObjectArray::Ptr makeDummyConvexHullObjectArrayForCalculatingCost();

  std::vector<std::vector<std::vector<double>>>
  assignPoints2GridCell(const grid_map::GridMap& gridmap, const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_sensor_points);

  grid_map::Index fetchGridIndexFromPoint(const grid_map::GridMap& gridmap, const pcl::PointXYZ& point);

  bool isValidInd(const grid_map::GridMap& gridmap, const grid_map::Index& grid_ind);

  grid_map::Matrix calculateCostmap(const double maximum_height_thres,
                                    const double minimum_lidar_height_thres, const double grid_min_value,
                                    const double grid_max_value, const grid_map::GridMap& gridmap,
                                    const std::string& gridmap_layer_name,
                                    const std::vector<std::vector<std::vector<double>>> grid_vec);


  Objects2Costmap *objects2costmap_;
  Eigen::MatrixXd makeRectanglePoints(const autoware_msgs::DetectedObject& in_object,
    const double expand_rectangle_size);
  geometry_msgs::Point makeExpandedPoint(const geometry_msgs::Point& in_centroid,
                                         const geometry_msgs::Point32& in_corner_point,
                                         const double expand_polygon_size);

  grid_map::Polygon makePolygonFromObjectConvexHull(const autoware_msgs::DetectedObject& in_object,
                                                    const double expand_polygon_size);

  void setCostInPolygon(const grid_map::Polygon& polygon, const std::string& gridmap_layer_name,
                                       const float score, grid_map::GridMap& objects_costmap);

  grid_map::Matrix makeCostmapFromObjects(const grid_map::GridMap& costmap,
                                          const double expand_polygon_size,
                                          const double size_of_expansion_kernel,
                                          const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects,
                                          const bool use_objects_convex_hull);
};

TestClass::TestClass():
dummy_grid_length_x_(10),
dummy_grid_length_y_(10),
dummy_grid_resolution_(1),
dummy_grid_position_x_(0),
dummy_grid_position_y_(0),
dummy_layer_name_("test"),
dummy_initialize_cost_(0)
{

}

grid_map::GridMap TestClass::makeDummyCostmap()
{
  grid_map::GridMap costmap;
  costmap.setGeometry(grid_map::Length(dummy_grid_length_x_, dummy_grid_length_y_), dummy_grid_resolution_,
                      grid_map::Position(dummy_grid_position_x_, dummy_grid_position_y_));

  costmap.add(dummy_layer_name_, dummy_initialize_cost_);
  return costmap;
}

std::vector<std::vector<std::vector<double>>> TestClass::makeDummy3DVec()
{
  double y_cell_size = std::ceil(dummy_grid_length_y_ * (1 / dummy_grid_resolution_));
  double x_cell_size = std::ceil(dummy_grid_length_x_ * (1 / dummy_grid_resolution_));
  std::vector<double> z_vec;
  std::vector<std::vector<double>> vec_y_z(y_cell_size, z_vec);
  std::vector<std::vector<std::vector<double>>> vec_x_y_z(x_cell_size, vec_y_z);
  return vec_x_y_z;
}
autoware_msgs::DetectedObjectArray::Ptr TestClass::makeDummyConvexHullObjectArrayForCalculatingCost()
{
  autoware_msgs::DetectedObject object;
  object.header.frame_id = "test";
  geometry_msgs::Point32 point;
  point.x = -1;
  point.y = -1;
  point.z = 0;
  object.convex_hull.polygon.points.push_back(point);
  point.x =  1;
  point.y = -1;
  point.z = 0;
  object.convex_hull.polygon.points.push_back(point);
  point.x = 1;
  point.y = 1;
  point.z = 0;
  object.convex_hull.polygon.points.push_back(point);
  point.x = 0;
  point.y = 2;
  point.z = 0;
  object.convex_hull.polygon.points.push_back(point);
  object.pose.position.x = 0;
  object.pose.position.y = 0;
  object.pose.position.z = 0;
  object.score = 1;
  autoware_msgs::DetectedObjectArray::Ptr objects(new autoware_msgs::DetectedObjectArray);
  objects->objects.push_back(object);
  return objects;
}

Eigen::MatrixXd TestClass::makeRectanglePoints(const autoware_msgs::DetectedObject& in_object,
                                               const double expanded_rectangle_size)
{
  return objects2costmap_->makeRectanglePoints(in_object, expanded_rectangle_size);
}

std::vector<std::vector<std::vector<double>>> TestClass::assignPoints2GridCell(
    const grid_map::GridMap& gridmap, const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_sensor_points)
{
  points2costmap_->grid_length_x_ = gridmap.getLength()[0];
  points2costmap_->grid_length_y_ = gridmap.getLength()[1];
  points2costmap_->grid_resolution_ = gridmap.getResolution();
  points2costmap_->grid_position_x_ = gridmap.getPosition()[0];
  points2costmap_->grid_position_y_ = gridmap.getPosition()[1];
  return points2costmap_->assignPoints2GridCell(gridmap, in_sensor_points);
}

grid_map::Index TestClass::fetchGridIndexFromPoint(const grid_map::GridMap& gridmap, const pcl::PointXYZ& point)
{
  points2costmap_->grid_length_x_ = gridmap.getLength()[0];
  points2costmap_->grid_length_y_ = gridmap.getLength()[1];
  points2costmap_->grid_resolution_ = gridmap.getResolution();
  points2costmap_->grid_position_x_ = gridmap.getPosition()[0];
  points2costmap_->grid_position_y_ = gridmap.getPosition()[1];
  return points2costmap_->fetchGridIndexFromPoint(point);
}

bool TestClass::isValidInd(const grid_map::GridMap& gridmap, const grid_map::Index& grid_ind)
{
  points2costmap_->grid_length_x_ = gridmap.getLength()[0];
  points2costmap_->grid_length_y_ = gridmap.getLength()[1];
  points2costmap_->grid_resolution_ = gridmap.getResolution();
  points2costmap_->grid_position_x_ = gridmap.getPosition()[0];
  points2costmap_->grid_position_y_ = gridmap.getPosition()[1];
  return points2costmap_->isValidInd(grid_ind);
}

grid_map::Matrix TestClass::calculateCostmap(const double maximum_height_thres,
                                                  const double minimum_lidar_height_thres, const double grid_min_value,
                                                  const double grid_max_value, const grid_map::GridMap& gridmap,
                                                  const std::string& gridmap_layer_name,
                                                  const std::vector<std::vector<std::vector<double>>> grid_vec)
{
  return points2costmap_->calculateCostmap(maximum_height_thres, minimum_lidar_height_thres,
                                          grid_min_value, grid_max_value,
                                          gridmap, gridmap_layer_name, grid_vec);
}

geometry_msgs::Point TestClass::makeExpandedPoint(const geometry_msgs::Point& in_centroid,
                                       const geometry_msgs::Point32& in_corner_point,
                                       const double expand_polygon_size)
{
  return objects2costmap_->makeExpandedPoint(in_centroid, in_corner_point, expand_polygon_size);
}

grid_map::Polygon TestClass::makePolygonFromObjectConvexHull(const autoware_msgs::DetectedObject& in_object,
                                                  const double expand_polygon_size)
{
  return objects2costmap_->makePolygonFromObjectConvexHull(in_object, expand_polygon_size);
}

void TestClass::setCostInPolygon(const grid_map::Polygon& polygon, const std::string& gridmap_layer_name,
                                     const float score, grid_map::GridMap& objects_costmap)
{
  objects2costmap_->setCostInPolygon(polygon, gridmap_layer_name, score, objects_costmap);
}

grid_map::Matrix TestClass::makeCostmapFromObjects(const grid_map::GridMap& costmap,
                                       const double expand_polygon_size,
                                       const double size_of_expansion_kernel,
                                       const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects,
                                       const bool use_objects_convex_hull)
{
  return objects2costmap_->makeCostmapFromObjects(costmap,
                                                  expand_polygon_size,
                                                  size_of_expansion_kernel,
                                                  in_objects,
                                                  use_objects_convex_hull);
}
