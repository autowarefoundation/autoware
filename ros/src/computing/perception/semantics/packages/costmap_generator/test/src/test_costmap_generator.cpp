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
#include "test_costmap_generator.hpp"

class TestSuite : public ::testing::Test
{
public:
  TestSuite()
  {
  }
  ~TestSuite()
  {
  }

  TestClass test_obj_;

protected:
  void SetUp()
  {
    test_obj_.objects2costmap_ = new ObjectsToCostmap();
    test_obj_.points2costmap_ = new PointsToCostmap();
    test_obj_.dummy_point_ = new geometry_msgs::Point;
    test_obj_.dummy_pcl_point_ = new pcl::PointXYZ;
    test_obj_.dummy_object_ = new autoware_msgs::DetectedObject;
    test_obj_.dummy_costmap_ = new grid_map::GridMap;
    test_obj_.dummy_objects_array_.reset(new autoware_msgs::DetectedObjectArray);

    test_obj_.fillDummyObjectParam(test_obj_.dummy_object_);
    test_obj_.fillDummyCostmapParam(test_obj_.dummy_costmap_);
    test_obj_.fillDummyObjectsArrayParam(test_obj_.dummy_objects_array_);
    test_obj_.initDummy3DVecParam();

  };
  void TearDown()
  {
    delete test_obj_.objects2costmap_;
    delete test_obj_.points2costmap_;
    delete test_obj_.dummy_point_;
    delete test_obj_.dummy_pcl_point_;
    delete test_obj_.dummy_object_;
    delete test_obj_.dummy_costmap_;
  };
};

TEST_F(TestSuite, CheckMakeRectanglePoints)
{
  test_obj_.dummy_point_->x = 2;
  test_obj_.dummy_point_->y = 2;
  test_obj_.dummy_point_->z = 0;
  geometry_msgs::Vector3 dimensions;
  dimensions.x = 2;
  dimensions.y = 4;
  dimensions.z = 1;
  const double pi = std::atan(1) * 4;
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(pi / 2);

  autoware_msgs::DetectedObject object;
  object.pose.position = *test_obj_.dummy_point_;
  object.pose.orientation = quat;
  object.dimensions = dimensions;

  Eigen::MatrixXd rectangle_points = test_obj_.makeRectanglePoints(object, 0);

  // following rep103 ros coordinate
  const double expected_x = 0;
  const double expected_y = 3;
  const double buffer = 0.01;
  EXPECT_NEAR(expected_x, rectangle_points(0, 0), buffer);
  EXPECT_DOUBLE_EQ(expected_y, rectangle_points(1, 0));
}

TEST_F(TestSuite, CheckAssignPoints2GridCell)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr in_sensor_points(new pcl::PointCloud<pcl::PointXYZ>);
  test_obj_.dummy_pcl_point_->x = 0.5;
  test_obj_.dummy_pcl_point_->y = 0.5;
  test_obj_.dummy_pcl_point_->z = 100;
  in_sensor_points->push_back(*test_obj_.dummy_pcl_point_);


  std::vector<std::vector<std::vector<double>>> grid_vec =
    test_obj_.assignPoints2GridCell(*test_obj_.dummy_costmap_,
                                    in_sensor_points);

  const double expected_value = 100;
  EXPECT_EQ(expected_value, grid_vec[5][5][0]);
}

TEST_F(TestSuite, CheckFetchGridIndexFromPoint)
{

  test_obj_.dummy_pcl_point_->x = 3.5;
  test_obj_.dummy_pcl_point_->y = 0.5;
  test_obj_.dummy_pcl_point_->z = 100;

  grid_map::Index index =
  test_obj_.fetchGridIndexFromPoint(*test_obj_.dummy_costmap_,
                                    *test_obj_.dummy_pcl_point_);


  const double expected_x_ind = 2;
  const double expected_y_ind = 5;
  EXPECT_EQ(expected_x_ind, index.x());
  EXPECT_EQ(expected_y_ind, index.y());
}

TEST_F(TestSuite, CheckValidIndex)
{
  grid_map::Index index(2,5);
  bool is_valid = test_obj_.isValidInd(*test_obj_.dummy_costmap_, index);

  bool expected_valid = true;
  EXPECT_EQ(expected_valid, is_valid);
}

TEST_F(TestSuite, CheckNonValidIndex)
{
  grid_map::Index index(2,10);
  bool is_valid = test_obj_.isValidInd(*test_obj_.dummy_costmap_, index);

  bool expected_valid = false;
  EXPECT_EQ(expected_valid, is_valid);
}

TEST_F(TestSuite, CheckCalculationPointsCostmap)
{
  test_obj_.dummy_3d_vec_->at(5)[5].push_back(2.2);

  grid_map::Matrix costmap_mat = test_obj_.calculateCostmap(
    test_obj_.dummy_maximum_lidar_height_thres_,
    test_obj_.dummy_minimum_lidar_height_thres_,
    test_obj_.dummy_grid_min_value_,
    test_obj_.dummy_grid_max_value_,
    *test_obj_.dummy_costmap_,
    test_obj_.dummy_layer_name_,
    *test_obj_.dummy_3d_vec_);
  double expected_cost = 1.0;
  EXPECT_DOUBLE_EQ(expected_cost, costmap_mat(5,5));
}

TEST_F(TestSuite, CheckHeightThresholdForCost)
{
  test_obj_.dummy_3d_vec_->at(5)[5].push_back(3.2);

  grid_map::Matrix costmap_mat = test_obj_.calculateCostmap(
    test_obj_.dummy_maximum_lidar_height_thres_,
    test_obj_.dummy_minimum_lidar_height_thres_,
    test_obj_.dummy_grid_min_value_,
    test_obj_.dummy_grid_max_value_,
    *test_obj_.dummy_costmap_,
    test_obj_.dummy_layer_name_,
    *test_obj_.dummy_3d_vec_);
  double expected_cost = 0.0;
  EXPECT_DOUBLE_EQ(expected_cost, costmap_mat(5,5));
}

TEST_F(TestSuite, CheckMakeExpandedPoints)
{
  double expand_polygon_size = 1;

  geometry_msgs::Point in_centroid;
  in_centroid.x = 0;
  in_centroid.y = 0;
  in_centroid.z = 0;

  geometry_msgs::Point32 in_corner_point;
  in_corner_point.x = 1;
  in_corner_point.y = 1;
  in_corner_point.z = 1;

  geometry_msgs::Point expanded_point = test_obj_.makeExpandedPoint(in_centroid, in_corner_point, expand_polygon_size);
  double expected_x = 1.707109;
  double expected_y = 1.707109;
  const double buffer = 0.01;
  EXPECT_NEAR(expected_x, expanded_point.x, buffer);
  EXPECT_NEAR(expected_y, expanded_point.y, buffer);
}

TEST_F(TestSuite, ChecMakePolygonFromObjectConvexhull)
{
  double expand_polygon_size = 0;
  geometry_msgs::Point32 point;
  point.x = -1;
  point.y = -1;
  point.z = 0;
  test_obj_.dummy_object_->convex_hull.polygon.points.push_back(point);
  point.x = 1;
  point.y = 1;
  point.z = 0;
  test_obj_.dummy_object_->convex_hull.polygon.points.push_back(point);
  point.x = 0;
  point.y = 2;
  point.z = 0;
  test_obj_.dummy_object_->convex_hull.polygon.points.push_back(point);

  grid_map::Polygon polygon =
  test_obj_.makePolygonFromObjectConvexHull(*test_obj_.dummy_object_,
                                                expand_polygon_size);
  double expected_0_x = -1;
  double expected_1_y =  1;
  EXPECT_EQ(expected_0_x, polygon[0].x());
  EXPECT_EQ(expected_1_y, polygon[1].y());
}

TEST_F(TestSuite, ChecMakePolygonFromObjectConvexhullDofferentHeight)
{
  double expand_polygon_size = 0;
  geometry_msgs::Point32 point;
  point.x = -1;
  point.y = -1;
  point.z = 0;
  test_obj_.dummy_object_->convex_hull.polygon.points.push_back(point);
  point.x = -1;
  point.y = 3;
  point.z = 0;
  test_obj_.dummy_object_->convex_hull.polygon.points.push_back(point);
  point.x = 1;
  point.y = 1;
  point.z = 0;
  test_obj_.dummy_object_->convex_hull.polygon.points.push_back(point);
  point.x = 0;
  point.y = 2;
  point.z = 3;
  test_obj_.dummy_object_->convex_hull.polygon.points.push_back(point);

  grid_map::Polygon polygon = test_obj_.makePolygonFromObjectConvexHull(
                                  *test_obj_.dummy_object_,
                                   expand_polygon_size);
  int num_vertices = polygon.nVertices();
  double expected_num_vertices = 3;
  EXPECT_EQ(expected_num_vertices, num_vertices);
}

TEST_F(TestSuite, CheckSetCostInPolygon)
{
  grid_map::Polygon polygon;
  polygon.setFrameId(test_obj_.dummy_layer_name_);
  polygon.addVertex(grid_map::Position(-1, -1));
  polygon.addVertex(grid_map::Position(1, -1));
  polygon.addVertex(grid_map::Position(1, 1));
  polygon.addVertex(grid_map::Position(-1, 1));


  float score = 1;
  test_obj_.setCostInPolygon(polygon, test_obj_.dummy_layer_name_,
                             score, *test_obj_.dummy_costmap_);
  float expected_score = 1;
  EXPECT_EQ(expected_score,
    test_obj_.dummy_costmap_->atPosition(test_obj_.dummy_layer_name_, grid_map::Position(0, 0)));
}

TEST_F(TestSuite, CheckMakeCostmapFromObjects)
{
  geometry_msgs::Point32 point;
  point.x = -1;
  point.y = -1;
  point.z = 0;
  test_obj_.dummy_object_->convex_hull.polygon.points.push_back(point);
  point.x =  1;
  point.y = -1;
  point.z = 0;
  test_obj_.dummy_object_->convex_hull.polygon.points.push_back(point);
  point.x = 1;
  point.y = 1;
  point.z = 0;
  test_obj_.dummy_object_->convex_hull.polygon.points.push_back(point);
  point.x = 0;
  point.y = 2;
  point.z = 0;
  test_obj_.dummy_object_->convex_hull.polygon.points.push_back(point);
  autoware_msgs::DetectedObjectArray::Ptr dummy_objects(new autoware_msgs::DetectedObjectArray);
  dummy_objects->objects.push_back(*test_obj_.dummy_object_);


  double expand_polygon_size = 0;
  double size_of_expansion_kernel = 1;
  bool use_objects_convex_hull = true;
  grid_map::Matrix gridmap_mat = test_obj_.makeCostmapFromObjects(
                                          *test_obj_.dummy_costmap_,
                                          expand_polygon_size,
                                          size_of_expansion_kernel,
                                          dummy_objects,
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

TEST_F(TestSuite, CheckMakeCostmapFromObjectsExpandSize)
{
  double expand_polygon_size = 1;
  double size_of_expansion_kernel = 1;
  bool use_objects_convex_hull = true;
  grid_map::Matrix gridmap_mat = test_obj_.makeCostmapFromObjects(
                                          *test_obj_.dummy_costmap_,
                                          expand_polygon_size,
                                          size_of_expansion_kernel,
                                          test_obj_.dummy_objects_array_,
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

TEST_F(TestSuite, CheckMakeCostmapFromObjectsBlur)
{
  double expand_polygon_size = 0;
  double size_of_expansion_kernel = 3;
  bool use_objects_convex_hull = true;
  grid_map::Matrix gridmap_mat = test_obj_.makeCostmapFromObjects(
                                          *test_obj_.dummy_costmap_,
                                          expand_polygon_size,
                                          size_of_expansion_kernel,
                                          test_obj_.dummy_objects_array_,
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

TEST_F(TestSuite, CheckMakeCostmapFromObjectsBox)
{
  autoware_msgs::DetectedObjectArray::Ptr dummy_objects(new autoware_msgs::DetectedObjectArray);
  dummy_objects->objects.push_back(*test_obj_.dummy_object_);

  double expand_polygon_size = 1;
  double size_of_expansion_kernel = 1;
  bool use_objects_convex_hull = false;
  grid_map::Matrix gridmap_mat = test_obj_.makeCostmapFromObjects(
                                          *test_obj_.dummy_costmap_,
                                          expand_polygon_size,
                                          size_of_expansion_kernel,
                                          dummy_objects,
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

TEST_F(TestSuite, CheckMakeCostmapFromObjectsBoxBlur)
{
  test_obj_.dummy_object_->pose.position.x = -4;
  test_obj_.dummy_object_->pose.position.y = -3;
  test_obj_.dummy_object_->pose.position.z = 0;
  autoware_msgs::DetectedObjectArray::Ptr dummy_objects(new autoware_msgs::DetectedObjectArray);
  dummy_objects->objects.push_back(*test_obj_.dummy_object_);

  double expand_polygon_size = 1;
  double size_of_expansion_kernel = 3;
  bool use_objects_convex_hull = false;
  grid_map::Matrix gridmap_mat = test_obj_.makeCostmapFromObjects(
                                          *test_obj_.dummy_costmap_,
                                          expand_polygon_size,
                                          size_of_expansion_kernel,
                                          dummy_objects,
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
