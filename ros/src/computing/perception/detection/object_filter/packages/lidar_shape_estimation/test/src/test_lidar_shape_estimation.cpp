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
 *
 *
 * v1.0 Yukihiro Saito
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "autoware_msgs/DetectedObject.h"
#include "lidar_shape_estimation/shape_estimator.hpp"

class ShapeEstimationTestSuite : public ::testing::Test
{
public:
  ShapeEstimationTestSuite()
  {
  }

  ~ShapeEstimationTestSuite()
  {
  }
};

class ShapeEstimationTestClass
{
public:
  ShapeEstimationTestClass(){};

  ShapeEstimator shape_estimator;
};

TEST(TestSuite, CheckOnePoint)
{
  ShapeEstimationTestClass test_shape_estimation;

  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  pcl::PointXYZ p0;
  p0.x = p0.y = p0.z = 0;
  pointcloud.push_back(p0);
  autoware_msgs::DetectedObject output;

  bool ret1 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("car"), pointcloud, output);
  bool ret2 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("person"), pointcloud, output);
  bool ret3 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("motorbike"), pointcloud, output);
  bool ret4 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("bicycle"), pointcloud, output);
  bool ret5 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("foo"), pointcloud, output);

  ASSERT_EQ(ret1, false) << "Only 1 point. It shoud be "
                         << "false";  // minimum point is 2
  ASSERT_EQ(ret2, true) << "Only 1 point. It shoud be "
                        << "true";  // minimum point is 1
  ASSERT_EQ(ret3, false) << "Only 1 point. It shoud be "
                         << "false";  // minimum point is 2
  ASSERT_EQ(ret4, false) << "Only 1 point. It shoud be "
                         << "false";  // minimum point is 2
  ASSERT_EQ(ret5, false) << "Only 1 point. It shoud be "
                         << "false";  // minimum point is 2
}

TEST(TestSuite, CheckTwoPoint)
{
  ShapeEstimationTestClass test_shape_estimation;

  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  pcl::PointXYZ p0, p1, p2, p3;
  p0.x = p0.y = p0.z = 0;
  pointcloud.push_back(p0);
  p1.x = p1.y = p1.z = 1;
  pointcloud.push_back(p1);
  autoware_msgs::DetectedObject output;

  bool ret1 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("car"), pointcloud, output);
  bool ret2 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("person"), pointcloud, output);
  bool ret3 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("motorbike"), pointcloud, output);
  bool ret4 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("bicycle"), pointcloud, output);
  bool ret5 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("foo"), pointcloud, output);

  ASSERT_EQ(ret1, true) << "difference 2 points. It shoud be "
                        << "true";  // minimum point is 2
  ASSERT_EQ(ret2, true) << "difference 2 points. It shoud be "
                        << "true";  // minimum point is 1
  ASSERT_EQ(ret3, true) << "difference 2 points. It shoud be "
                        << "true";  // minimum point is 2
  ASSERT_EQ(ret4, true) << "difference 2 points. It shoud be "
                        << "true";  // minimum point is 2
  ASSERT_EQ(ret5, true) << "difference 2 points. It shoud be "
                        << "true";  // minimum point is 2
}

TEST(TestSuite, CheckEmptyPoint)
{
  ShapeEstimationTestClass test_shape_estimation;

  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  autoware_msgs::DetectedObject output;

  bool ret1 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("car"), pointcloud, output);
  bool ret2 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("person"), pointcloud, output);
  bool ret3 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("motorbike"), pointcloud, output);
  bool ret4 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("bicycle"), pointcloud, output);
  bool ret5 = test_shape_estimation.shape_estimator.getShapeAndPose(std::string("foo"), pointcloud, output);

  ASSERT_EQ(ret1, false) << "empty pointcloud cannnot estimate. It shoud be "
                         << "false";
  ASSERT_EQ(ret2, false) << "empty pointcloud cannnot estimate. It shoud be "
                         << "false";
  ASSERT_EQ(ret3, false) << "empty pointcloud cannnot estimate. It shoud be "
                         << "false";
  ASSERT_EQ(ret4, false) << "empty pointcloud cannnot estimate. It shoud be "
                         << "false";
  ASSERT_EQ(ret5, false) << "empty pointcloud cannnot estimate. It shoud be "
                         << "false";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ShapeEstimationTestNode");
  return RUN_ALL_TESTS();
}
