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

#include "costmap_generator.h"

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

  Objects2Costmap objects2costmap_;

  Eigen::MatrixXd makeRectanglePoints(const autoware_msgs::DetectedObject& in_object,
                                      const double expand_rectangle_size);

  Points2Costmap points2costmap_;

  std::vector<std::vector<std::vector<double>>>
  assignPoints2GridCell(const grid_map::GridMap& gridmap, const sensor_msgs::PointCloud2::ConstPtr& in_sensor_points);
};

Eigen::MatrixXd TestClass::makeRectanglePoints(const autoware_msgs::DetectedObject& in_object,
                                               const double expanded_rectangle_size)
{
  return objects2costmap_.makeRectanglePoints(in_object, expanded_rectangle_size);
}

std::vector<std::vector<std::vector<double>>> TestClass::assignPoints2GridCell(
    const grid_map::GridMap& gridmap, const sensor_msgs::PointCloud2::ConstPtr& in_sensor_points)
{
  points2costmap_.grid_length_x_ = gridmap.getLength()[0];
  points2costmap_.grid_length_y_ = gridmap.getLength()[1];
  points2costmap_.grid_resolution_ = gridmap.getResolution();
  points2costmap_.grid_position_x_ = gridmap.getPosition()[0];
  points2costmap_.grid_position_y_ = gridmap.getPosition()[1];
  return points2costmap_.assignPoints2GridCell(gridmap, in_sensor_points);
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

  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  pcl::PointXYZ point;
  point.x = 0.5;
  point.y = 0.5;
  point.z = 100;
  pointcloud.push_back(point);

  sensor_msgs::PointCloud2::Ptr pointcloud_msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(pointcloud, *pointcloud_msg);

  std::vector<std::vector<std::vector<double>>> grid_vec = test_obj.assignPoints2GridCell(costmap, pointcloud_msg);

  const double expected_value = 100;
  EXPECT_EQ(expected_value, grid_vec[5][5][0]);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
