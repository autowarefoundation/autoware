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

#include <ros/ros.h>
#include <gtest/gtest.h>

#include "costmap_generator/costmap_generator.h"

class TestSuite : public ::testing::Test
{
public:
  TestSuite()
  {
  }
  ~TestSuite()
  {
  }
};

class TestClass
{
public:
  TestClass(){};

  Points2Costmap points2costmap_;

  std::vector<std::vector<std::vector<double>>>
  assignPoints2GridCell(const grid_map::GridMap& gridmap, const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_sensor_points);

  grid_map::Index fetchGridIndexFromPoint(const grid_map::GridMap& gridmap, const pcl::PointXYZ& point);

  bool isValidInd(const grid_map::GridMap& gridmap, const grid_map::Index& grid_ind);

  grid_map::Matrix calculateCostmap(const double maximum_height_thres,
                                    const double minimum_lidar_height_thres, const double grid_min_value,
                                    const double grid_max_value, const grid_map::GridMap& gridmap,
                                    const std::string& gridmap_layer_name,
                                    const std::vector<std::vector<std::vector<double>>> grid_vec);


  Objects2Costmap objects2costmap_;
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

Eigen::MatrixXd TestClass::makeRectanglePoints(const autoware_msgs::DetectedObject& in_object,
                                               const double expanded_rectangle_size)
{
  return objects2costmap_.makeRectanglePoints(in_object, expanded_rectangle_size);
}

std::vector<std::vector<std::vector<double>>> TestClass::assignPoints2GridCell(
    const grid_map::GridMap& gridmap, const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_sensor_points)
{
  points2costmap_.grid_length_x_ = gridmap.getLength()[0];
  points2costmap_.grid_length_y_ = gridmap.getLength()[1];
  points2costmap_.grid_resolution_ = gridmap.getResolution();
  points2costmap_.grid_position_x_ = gridmap.getPosition()[0];
  points2costmap_.grid_position_y_ = gridmap.getPosition()[1];
  return points2costmap_.assignPoints2GridCell(gridmap, in_sensor_points);
}

grid_map::Index TestClass::fetchGridIndexFromPoint(const grid_map::GridMap& gridmap, const pcl::PointXYZ& point)
{
  points2costmap_.grid_length_x_ = gridmap.getLength()[0];
  points2costmap_.grid_length_y_ = gridmap.getLength()[1];
  points2costmap_.grid_resolution_ = gridmap.getResolution();
  points2costmap_.grid_position_x_ = gridmap.getPosition()[0];
  points2costmap_.grid_position_y_ = gridmap.getPosition()[1];
  return points2costmap_.fetchGridIndexFromPoint(point);
}

bool TestClass::isValidInd(const grid_map::GridMap& gridmap, const grid_map::Index& grid_ind)
{
  points2costmap_.grid_length_x_ = gridmap.getLength()[0];
  points2costmap_.grid_length_y_ = gridmap.getLength()[1];
  points2costmap_.grid_resolution_ = gridmap.getResolution();
  points2costmap_.grid_position_x_ = gridmap.getPosition()[0];
  points2costmap_.grid_position_y_ = gridmap.getPosition()[1];
  return points2costmap_.isValidInd(grid_ind);
}

grid_map::Matrix TestClass::calculateCostmap(const double maximum_height_thres,
                                                  const double minimum_lidar_height_thres, const double grid_min_value,
                                                  const double grid_max_value, const grid_map::GridMap& gridmap,
                                                  const std::string& gridmap_layer_name,
                                                  const std::vector<std::vector<std::vector<double>>> grid_vec)
{
  return points2costmap_.calculateCostmap(maximum_height_thres, minimum_lidar_height_thres,
                                          grid_min_value, grid_max_value,
                                          gridmap, gridmap_layer_name, grid_vec);
}

geometry_msgs::Point TestClass::makeExpandedPoint(const geometry_msgs::Point& in_centroid,
                                       const geometry_msgs::Point32& in_corner_point,
                                       const double expand_polygon_size)
{
  return objects2costmap_.makeExpandedPoint(in_centroid, in_corner_point, expand_polygon_size);
}

grid_map::Polygon TestClass::makePolygonFromObjectConvexHull(const autoware_msgs::DetectedObject& in_object,
                                                  const double expand_polygon_size)
{
  return objects2costmap_.makePolygonFromObjectConvexHull(in_object, expand_polygon_size);
}

void TestClass::setCostInPolygon(const grid_map::Polygon& polygon, const std::string& gridmap_layer_name,
                                     const float score, grid_map::GridMap& objects_costmap)
{
  objects2costmap_.setCostInPolygon(polygon, gridmap_layer_name, score, objects_costmap);
}

grid_map::Matrix TestClass::makeCostmapFromObjects(const grid_map::GridMap& costmap,
                                       const double expand_polygon_size,
                                       const double size_of_expansion_kernel,
                                       const autoware_msgs::DetectedObjectArray::ConstPtr& in_objects,
                                       const bool use_objects_convex_hull)
{
  return objects2costmap_.makeCostmapFromObjects(costmap,
                                                  expand_polygon_size,
                                                  size_of_expansion_kernel,
                                                  in_objects,
                                                  use_objects_convex_hull);
}

TEST(TestSuite, CheckMakeRectanglePoints)
{
  TestClass test_obj;

  geometry_msgs::Point position;
  position.x = 2;
  position.y = 2;
  position.z = 0;
  geometry_msgs::Vector3 dimensions;
  dimensions.x = 2;
  dimensions.y = 4;
  dimensions.z = 1;
  const double pi = std::atan(1) * 4;
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(pi / 2);

  autoware_msgs::DetectedObject object;
  object.pose.position = position;
  object.pose.orientation = quat;
  object.dimensions = dimensions;

  Eigen::MatrixXd rectangle_points = test_obj.makeRectanglePoints(object, 0);

  // following rep103 ros coordinate
  const double expected_x = 0;
  const double expected_y = 3;
  const double buffer = 0.01;
  EXPECT_NEAR(expected_x, rectangle_points(0, 0), buffer);
  EXPECT_DOUBLE_EQ(expected_y, rectangle_points(1, 0));
}

TEST(TestSuite, CheckAssignPoints2GridCell)
{
  TestClass test_obj;

  const double grid_length_x = 10;
  const double grid_length_y = 10;
  const double grid_resolution = 1;
  const double grid_position_x = 0;
  const double grid_position_y = 0;
  const std::string layer_name = "test";
  const double initialize_cost = 0;
  grid_map::GridMap costmap;
  costmap.setGeometry(grid_map::Length(grid_length_x, grid_length_y), grid_resolution,
                      grid_map::Position(grid_position_x, grid_position_y));

  costmap.add(layer_name, initialize_cost);

  pcl::PointCloud<pcl::PointXYZ>::Ptr in_sensor_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ point;
  point.x = 0.5;
  point.y = 0.5;
  point.z = 100;
  in_sensor_points->push_back(point);


  std::vector<std::vector<std::vector<double>>> grid_vec = test_obj.assignPoints2GridCell(costmap, in_sensor_points);

  const double expected_value = 100;
  EXPECT_EQ(expected_value, grid_vec[5][5][0]);
}

TEST(TestSuite, CheckFetchGridIndexFromPoint)
{
  TestClass test_obj;

  const double grid_length_x = 10;
  const double grid_length_y = 10;
  const double grid_resolution = 1;
  const double grid_position_x = 0;
  const double grid_position_y = 0;
  const std::string layer_name = "test";
  const double initialize_cost = 0;
  grid_map::GridMap costmap;
  costmap.setGeometry(grid_map::Length(grid_length_x, grid_length_y), grid_resolution,
                      grid_map::Position(grid_position_x, grid_position_y));

  costmap.add(layer_name, initialize_cost);

  pcl::PointXYZ point;
  point.x = 3.5;
  point.y = 0.5;
  point.z = 100;

  grid_map::Index index = test_obj.fetchGridIndexFromPoint(costmap, point);


  const double expected_x_ind = 2;
  const double expected_y_ind = 5;
  EXPECT_EQ(expected_x_ind, index.x());
  EXPECT_EQ(expected_y_ind, index.y());
}

TEST(TestSuite, CheckValidIndex)
{
  TestClass test_obj;

  const double grid_length_x = 10;
  const double grid_length_y = 10;
  const double grid_resolution = 1;
  const double grid_position_x = 0;
  const double grid_position_y = 0;
  const std::string layer_name = "test";
  const double initialize_cost = 0;
  grid_map::GridMap costmap;
  costmap.setGeometry(grid_map::Length(grid_length_x, grid_length_y), grid_resolution,
                      grid_map::Position(grid_position_x, grid_position_y));

  costmap.add(layer_name, initialize_cost);

  grid_map::Index index(2,5);
  bool is_valid = test_obj.isValidInd(costmap, index);

  bool expected_valid = true;
  EXPECT_EQ(expected_valid, is_valid);
}

TEST(TestSuite, CheckNonValidIndex)
{
  TestClass test_obj;

  const double grid_length_x = 10;
  const double grid_length_y = 10;
  const double grid_resolution = 1;
  const double grid_position_x = 0;
  const double grid_position_y = 0;
  const std::string layer_name = "test";
  const double initialize_cost = 0;
  grid_map::GridMap costmap;
  costmap.setGeometry(grid_map::Length(grid_length_x, grid_length_y), grid_resolution,
                      grid_map::Position(grid_position_x, grid_position_y));

  costmap.add(layer_name, initialize_cost);

  grid_map::Index index(2,10);
  bool is_valid = test_obj.isValidInd(costmap, index);

  bool expected_valid = false;
  EXPECT_EQ(expected_valid, is_valid);
}

TEST(TestSuite, CheckCalculationPointsCostmap)
{
  TestClass test_obj;

  const double grid_length_x = 10;
  const double grid_length_y = 10;
  const double grid_resolution = 1;
  const double grid_position_x = 0;
  const double grid_position_y = 0;
  const std::string layer_name = "test";
  const double initialize_cost = 0;
  grid_map::GridMap costmap;

  costmap.setGeometry(grid_map::Length(grid_length_x, grid_length_y), grid_resolution,
  grid_map::Position(grid_position_x, grid_position_y));
  costmap.add(layer_name, initialize_cost);

  double y_cell_size = std::ceil(grid_length_y * (1 / grid_resolution));
  double x_cell_size = std::ceil(grid_length_x * (1 / grid_resolution));
  std::vector<double> z_vec;
  std::vector<std::vector<double>> vec_y_z(y_cell_size, z_vec);
  std::vector<std::vector<std::vector<double>>> vec_x_y_z(x_cell_size, vec_y_z);
  vec_x_y_z[5][5].push_back(2.2);

  double maximum_height_thres = 3.0;
  double minimum_lidar_height_thres = -2.0;
  double grid_min_value = 0;
  double grid_max_value = 1;

  grid_map::Matrix costmap_mat = test_obj.calculateCostmap(maximum_height_thres, minimum_lidar_height_thres, grid_min_value,
                                              grid_max_value, costmap, layer_name, vec_x_y_z);
  double expected_cost = 1.0;
  EXPECT_DOUBLE_EQ(expected_cost, costmap_mat(5,5));
}

TEST(TestSuite, CheckHeightThresholdForCost)
{
  TestClass test_obj;

  const double grid_length_x = 10;
  const double grid_length_y = 10;
  const double grid_resolution = 1;
  const double grid_position_x = 0;
  const double grid_position_y = 0;
  const std::string layer_name = "test";
  const double initialize_cost = 0;
  grid_map::GridMap costmap;

  costmap.setGeometry(grid_map::Length(grid_length_x, grid_length_y), grid_resolution,
  grid_map::Position(grid_position_x, grid_position_y));
  costmap.add(layer_name, initialize_cost);

  double y_cell_size = std::ceil(grid_length_y * (1 / grid_resolution));
  double x_cell_size = std::ceil(grid_length_x * (1 / grid_resolution));
  std::vector<double> z_vec;
  std::vector<std::vector<double>> vec_y_z(y_cell_size, z_vec);
  std::vector<std::vector<std::vector<double>>> vec_x_y_z(x_cell_size, vec_y_z);
  vec_x_y_z[5][5].push_back(3.2);

  double maximum_height_thres = 3.0;
  double minimum_lidar_height_thres = -2.0;
  double grid_min_value = 0;
  double grid_max_value = 1;

  grid_map::Matrix costmap_mat = test_obj.calculateCostmap(maximum_height_thres, minimum_lidar_height_thres, grid_min_value,
                                              grid_max_value, costmap, layer_name, vec_x_y_z);
  double expected_cost = 0.0;
  EXPECT_DOUBLE_EQ(expected_cost, costmap_mat(5,5));
}

TEST(TestSuite, CheckMakeExpandedPoints)
{
  TestClass test_obj;
  geometry_msgs::Point in_centroid;
  geometry_msgs::Point32 in_corner_point;
  double expand_polygon_size = 1;

  in_centroid.x = 0;
  in_centroid.y = 0;
  in_centroid.z = 0;

  in_corner_point.x = 1;
  in_corner_point.y = 1;
  in_corner_point.z = 1;

  geometry_msgs::Point expanded_point = test_obj.makeExpandedPoint(in_centroid, in_corner_point, expand_polygon_size);
  double expected_x = 1.707109;
  double expected_y = 1.707109;
  const double buffer = 0.01;
  EXPECT_NEAR(expected_x, expanded_point.x, buffer);
  EXPECT_NEAR(expected_y, expanded_point.y, buffer);
}

TEST(TestSuite, ChecMakePolygonFromObjectConvexhull)
{
  TestClass test_obj;

  double expand_polygon_size = 0;
  autoware_msgs::DetectedObject in_object;
  in_object.header.frame_id = "test";
  geometry_msgs::Point32 point;
  point.x = -1;
  point.y = -1;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x = 1;
  point.y = 1;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x = 0;
  point.y = 2;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);

  in_object.pose.position.x = 0;
  in_object.pose.position.y = 0;
  in_object.pose.position.z = 0;
  grid_map::Polygon polygon = test_obj.makePolygonFromObjectConvexHull(in_object, expand_polygon_size);
  double expected_0_x = -1;
  double expected_1_y =  1;
  EXPECT_EQ(expected_0_x, polygon[0].x());
  EXPECT_EQ(expected_1_y, polygon[1].y());
}

TEST(TestSuite, ChecMakePolygonFromObjectConvexhullDofferentHeight)
{
  TestClass test_obj;

  double expand_polygon_size = 0;
  autoware_msgs::DetectedObject in_object;
  in_object.header.frame_id = "test";
  geometry_msgs::Point32 point;
  point.x = -1;
  point.y = -1;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x = -1;
  point.y = 3;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x = 1;
  point.y = 1;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x = 0;
  point.y = 2;
  point.z = 3;
  in_object.convex_hull.polygon.points.push_back(point);

  in_object.pose.position.x = 0;
  in_object.pose.position.y = 0;
  in_object.pose.position.z = 0;
  grid_map::Polygon polygon = test_obj.makePolygonFromObjectConvexHull(in_object, expand_polygon_size);
  int num_vertices = polygon.nVertices();
  double expected_num_vertices = 3;
  EXPECT_EQ(expected_num_vertices, num_vertices);
}

TEST(TestSuite, ChecSetCostInPolygon)
{
  TestClass test_obj;

  grid_map::Polygon polygon;
  polygon.setFrameId("test");
  polygon.addVertex(grid_map::Position(-1, -1));
  polygon.addVertex(grid_map::Position(1, -1));
  polygon.addVertex(grid_map::Position(1, 1));
  polygon.addVertex(grid_map::Position(-1, 1));

  const double grid_length_x = 10;
  const double grid_length_y = 10;
  const double grid_resolution = 1;
  const double grid_position_x = 0;
  const double grid_position_y = 0;
  const std::string layer_name = "test";
  const double initialize_cost = 0;
  grid_map::GridMap costmap;

  costmap.setGeometry(grid_map::Length(grid_length_x, grid_length_y), grid_resolution,
  grid_map::Position(grid_position_x, grid_position_y));
  costmap.add(layer_name, initialize_cost);

  float score = 1;
  test_obj.setCostInPolygon(polygon, layer_name, score, costmap);
  float expected_score = 1;
  EXPECT_EQ(expected_score, costmap.atPosition(layer_name, grid_map::Position(0, 0)));
}

TEST(TestSuite, CheckMakeCostmapFromObjects)
{
  TestClass test_obj;
  autoware_msgs::DetectedObject in_object;
  in_object.header.frame_id = "test";
  geometry_msgs::Point32 point;
  point.x = -1;
  point.y = -1;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x =  1;
  point.y = -1;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x = 1;
  point.y = 1;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x = 0;
  point.y = 2;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  in_object.pose.position.x = 0;
  in_object.pose.position.y = 0;
  in_object.pose.position.z = 0;
  in_object.score = 1;
  autoware_msgs::DetectedObjectArray::Ptr in_objects(new autoware_msgs::DetectedObjectArray);
  in_objects->objects.push_back(in_object);

  const double grid_length_x = 10;
  const double grid_length_y = 10;
  const double grid_resolution = 1;
  const double grid_position_x = 0;
  const double grid_position_y = 0;
  const std::string layer_name = "test";
  const double initialize_cost = 0;
  grid_map::GridMap costmap;

  costmap.setGeometry(grid_map::Length(grid_length_x, grid_length_y), grid_resolution,
  grid_map::Position(grid_position_x, grid_position_y));
  costmap.add(layer_name, initialize_cost);

  double expand_polygon_size = 0;
  double size_of_expansion_kernel = 1;
  bool use_objects_convex_hull = true;
  grid_map::Matrix gridmap_mat = test_obj.makeCostmapFromObjects(costmap,
                                          expand_polygon_size,
                                          size_of_expansion_kernel,
                                          in_objects,
                                          use_objects_convex_hull);
  /*
    0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
    0 0 0 0 1 1 0 0 0 0
    0 0 0 0 1 1 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
  */
  float expected_score = 1;
  EXPECT_EQ(expected_score,gridmap_mat(5,5));
}

TEST(TestSuite, CheckMakeCostmapFromObjectsExpandSize)
{
  TestClass test_obj;
  autoware_msgs::DetectedObject in_object;
  in_object.header.frame_id = "test";
  geometry_msgs::Point32 point;
  point.x = -1;
  point.y = -1;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x =  1;
  point.y = -1;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x = 1;
  point.y = 1;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x = 0;
  point.y = 2;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  in_object.pose.position.x = 0;
  in_object.pose.position.y = 0;
  in_object.pose.position.z = 0;
  in_object.score = 1;
  autoware_msgs::DetectedObjectArray::Ptr in_objects(new autoware_msgs::DetectedObjectArray);
  in_objects->objects.push_back(in_object);

  const double grid_length_x = 10;
  const double grid_length_y = 10;
  const double grid_resolution = 1;
  const double grid_position_x = 0;
  const double grid_position_y = 0;
  const std::string layer_name = "test";
  const double initialize_cost = 0;
  grid_map::GridMap costmap;

  costmap.setGeometry(grid_map::Length(grid_length_x, grid_length_y), grid_resolution,
  grid_map::Position(grid_position_x, grid_position_y));
  costmap.add(layer_name, initialize_cost);

  double expand_polygon_size = 1;
  double size_of_expansion_kernel = 1;
  bool use_objects_convex_hull = true;
  grid_map::Matrix gridmap_mat = test_obj.makeCostmapFromObjects(costmap,
                                          expand_polygon_size,
                                          size_of_expansion_kernel,
                                          in_objects,
                                          use_objects_convex_hull);
  /*
    0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
    0 0 0 1 1 1 1 0 0 0
    0 0 1 1 1 1 1 0 0 0
    0 0 0 1 1 1 1 0 0 0
    0 0 0 0 0 0 1 0 0 0
    0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
  */
  float expected_score = 1;
  EXPECT_EQ(expected_score,gridmap_mat(6,6));
}

TEST(TestSuite, CheckMakeCostmapFromObjectsBlur)
{
  TestClass test_obj;
  autoware_msgs::DetectedObject in_object;
  in_object.header.frame_id = "test";
  geometry_msgs::Point32 point;
  point.x = -1;
  point.y = -1;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x =  1;
  point.y = -1;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x = 1;
  point.y = 1;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  point.x = 0;
  point.y = 2;
  point.z = 0;
  in_object.convex_hull.polygon.points.push_back(point);
  in_object.pose.position.x = 0;
  in_object.pose.position.y = 0;
  in_object.pose.position.z = 0;
  in_object.score = 1;
  autoware_msgs::DetectedObjectArray::Ptr in_objects(new autoware_msgs::DetectedObjectArray);
  in_objects->objects.push_back(in_object);

  const double grid_length_x = 10;
  const double grid_length_y = 10;
  const double grid_resolution = 1;
  const double grid_position_x = 0;
  const double grid_position_y = 0;
  const std::string layer_name = "test";
  const double initialize_cost = 0;
  grid_map::GridMap costmap;

  costmap.setGeometry(grid_map::Length(grid_length_x, grid_length_y), grid_resolution,
  grid_map::Position(grid_position_x, grid_position_y));
  costmap.add(layer_name, initialize_cost);

  double expand_polygon_size = 0;
  double size_of_expansion_kernel = 3;
  bool use_objects_convex_hull = true;
  grid_map::Matrix gridmap_mat = test_obj.makeCostmapFromObjects(costmap,
                                          expand_polygon_size,
                                          size_of_expansion_kernel,
                                          in_objects,
                                          use_objects_convex_hull);
  /*
  0  0  0           0           0           0 0.000228624 0.000635066 0.000683101 0.000629966
  0  0  0           0           0  0.00137174  0.00358177  0.00346354  0.00183676  0.00128529
  0  0  0           0   0.0123457   0.0306356   0.0267264   0.0117492  0.00456194  0.00264567
  0  0  0    0.111111    0.262003    0.204948   0.0719709    0.024008  0.00819001   0.0043936
  0  0  0    0.234568           1           1    0.105625    0.033391    0.010964  0.00580402
  0  0  0    0.248285           1           1   0.0989154   0.0330867   0.0112765  0.00618194
  0  0  0    0.138698    0.207193    0.110906   0.0598489    0.024047  0.00904712  0.00537396
  0  0  0   0.0154109    0.040335   0.0405389    0.024572   0.0130139  0.00573816  0.00386448
  0  0  0  0.00171233  0.00641596  0.00985468  0.00865679  0.00553543  0.00302765  0.00242517
  0  0  0 0.000285388  0.00140228  0.00294549  0.00357616   0.0029614  0.00192074  0.00184339
  */
  double expected_score = 0.0598489;
  double buffer = 0.001;
  EXPECT_NEAR(expected_score,gridmap_mat(6,6), buffer);
}

TEST(TestSuite, CheckMakeCostmapFromObjectsBox)
{
  TestClass test_obj;
  autoware_msgs::DetectedObject in_object;
  in_object.header.frame_id = "test";
  in_object.pose.position.x = 0;
  in_object.pose.position.y = 0;
  in_object.pose.position.z = 0;
  in_object.pose.orientation.x = 0;
  in_object.pose.orientation.y = 0;
  in_object.pose.orientation.z = 0;
  in_object.pose.orientation.w = 1;
  in_object.dimensions.x = 2;
  in_object.dimensions.y = 1.5;
  in_object.dimensions.z = 2;
  in_object.score = 1;
  autoware_msgs::DetectedObjectArray::Ptr in_objects(new autoware_msgs::DetectedObjectArray);
  in_objects->objects.push_back(in_object);

  const double grid_length_x = 10;
  const double grid_length_y = 10;
  const double grid_resolution = 1;
  const double grid_position_x = 0;
  const double grid_position_y = 0;
  const std::string layer_name = "test";
  const double initialize_cost = 0;
  grid_map::GridMap costmap;

  costmap.setGeometry(grid_map::Length(grid_length_x, grid_length_y), grid_resolution,
  grid_map::Position(grid_position_x, grid_position_y));
  costmap.add(layer_name, initialize_cost);

  double expand_polygon_size = 1;
  double size_of_expansion_kernel = 1;
  bool use_objects_convex_hull = false;
  grid_map::Matrix gridmap_mat = test_obj.makeCostmapFromObjects(costmap,
                                          expand_polygon_size,
                                          size_of_expansion_kernel,
                                          in_objects,
                                          use_objects_convex_hull);
  /*
  0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0
  0 0 0 0 1 1 0 0 0 0
  0 0 0 0 1 1 0 0 0 0
  0 0 0 0 1 1 0 0 0 0
  0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0
  0 0 0 0 0 0 0 0 0 0
  */
  float expected_score = 1;
  EXPECT_EQ(expected_score,gridmap_mat(5,5));
}

TEST(TestSuite, CheckMakeCostmapFromObjectsBoxBlur)
{
  TestClass test_obj;
  autoware_msgs::DetectedObject in_object;
  in_object.header.frame_id = "test";
  in_object.pose.position.x = -4;
  in_object.pose.position.y = -3;
  in_object.pose.position.z = 0;
  in_object.pose.orientation.x = 0;
  in_object.pose.orientation.y = 0;
  in_object.pose.orientation.z = 0;
  in_object.pose.orientation.w = 1;
  in_object.dimensions.x = 2;
  in_object.dimensions.y = 1.5;
  in_object.dimensions.z = 2;
  in_object.score = 1;
  autoware_msgs::DetectedObjectArray::Ptr in_objects(new autoware_msgs::DetectedObjectArray);
  in_objects->objects.push_back(in_object);

  const double grid_length_x = 10;
  const double grid_length_y = 10;
  const double grid_resolution = 1;
  const double grid_position_x = 0;
  const double grid_position_y = 0;
  const std::string layer_name = "test";
  const double initialize_cost = 0;
  grid_map::GridMap costmap;

  costmap.setGeometry(grid_map::Length(grid_length_x, grid_length_y), grid_resolution,
  grid_map::Position(grid_position_x, grid_position_y));
  costmap.add(layer_name, initialize_cost);

  double expand_polygon_size = 1;
  double size_of_expansion_kernel = 3;
  bool use_objects_convex_hull = false;
  grid_map::Matrix gridmap_mat = test_obj.makeCostmapFromObjects(costmap,
                                          expand_polygon_size,
                                          size_of_expansion_kernel,
                                          in_objects,
                                          use_objects_convex_hull);
  /*
  0 0 0 0 0 0           0           0           0           0
  0 0 0 0 0 0           0           0           0           0
  0 0 0 0 0 0           0           0           0           0
  0 0 0 0 0 0           0           0           0           0
  0 0 0 0 0 0           0           0           0 0.000228624
  0 0 0 0 0 0           0           0  0.00137174  0.00537266
  0 0 0 0 0 0           0   0.0123457   0.0306356   0.0406435
  0 0 0 0 0 0    0.111111    0.262003    0.206481    0.115094
  0 0 0 0 0 0    0.234568           1           1    0.196272
  0 0 0 0 0 0    0.372428           1           1    0.263083
  */
  float expected_score = 0.11111;
  float buffer = 0.00001;
  EXPECT_NEAR(expected_score, gridmap_mat(7,6), buffer);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
