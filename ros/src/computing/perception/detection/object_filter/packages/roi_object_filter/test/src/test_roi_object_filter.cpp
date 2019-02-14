/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <gtest/gtest.h>

#include "roi_object_filter/roi_object_filter.h"

class RoiFilterTestSuite : public ::testing::Test
{
public:
  RoiFilterTestSuite()
  {
  }

  ~RoiFilterTestSuite()
  {
  }
};

class RoiFilterTestClass
{
public:
  RoiFilterTestClass()
  {
  };

  RosRoiObjectFilterApp app;

  geometry_msgs::Point TransformPoint(const geometry_msgs::Point &in_point, const tf::Transform &in_tf);

  bool CheckPointInGrid(const grid_map::GridMap &in_grid_map, const cv::Mat &in_grid_image,
                        const geometry_msgs::Point &in_point);
};

geometry_msgs::Point RoiFilterTestClass::TransformPoint(const geometry_msgs::Point &in_point, const tf::Transform &in_tf)
{
  return app.TransformPoint(in_point, in_tf);
}

bool RoiFilterTestClass::CheckPointInGrid(const grid_map::GridMap &in_grid_map, const cv::Mat &in_grid_image,
                                 const geometry_msgs::Point &in_point)
{
  return app.CheckPointInGrid(in_grid_map, in_grid_image, in_point);
}

TEST(TestSuite, CheckTransformPoint)
{

  RoiFilterTestClass testObj;

  // Check translation of 1 along X axis
  tf::Quaternion q(0, 0, 0, 1);
  tf::Vector3 v(1, 0, 0);
  geometry_msgs::Point in_point, out_point, expected_point;

  in_point.x = 0;
  in_point.y = 0;
  in_point.z = 0;
  expected_point.x = 1;
  expected_point.y = 0;
  expected_point.z = 0;

  tf::Transform translation(q, v);

  out_point = testObj.TransformPoint(in_point, translation);

  ASSERT_EQ(out_point.x, expected_point.x) << "X Coordinate should be " << expected_point.x;
  ASSERT_EQ(out_point.y, expected_point.y) << "Y Coordinate should be " << expected_point.y;
  ASSERT_EQ(out_point.z, expected_point.z) << "Z Coordinate should be " << expected_point.z;

}

TEST(TestSuite, CheckPointInGrid)
{

  RoiFilterTestClass testObj;
  grid_map::GridMap test_gridmap;

  test_gridmap.setFrameId("test");
  test_gridmap.add("test");
  test_gridmap.setGeometry(grid_map::Length(1.0, 1.0),
                           0.1,
                           grid_map::Position(0, 0));
  test_gridmap["test"].setConstant(255);

  cv::Mat grid_image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(test_gridmap, "test", CV_8UC1,
                                                          0, 255, grid_image);

  geometry_msgs::Point in_point, out_point;
  in_point.x = 0.5;
  in_point.y = 0.5;
  out_point.x = 10.;
  out_point.y = 10.;

  bool in_result = testObj.CheckPointInGrid(test_gridmap, grid_image, in_point);
  bool out_result = testObj.CheckPointInGrid(test_gridmap, grid_image, out_point);

  ASSERT_EQ(in_result, true) << "Point " << in_point.x << "," << in_point.y
                             << " result should be TRUE (inside) got: " << in_result;
  ASSERT_EQ(out_result, false) << "Point " << out_point.x << "," << out_point.y
                             << " result should be FALSE (outside) got: " << out_result;

}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "RoiFilterTestNode");
  return RUN_ALL_TESTS();
}

