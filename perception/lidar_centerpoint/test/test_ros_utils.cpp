// Copyright 2024 TIER IV, Inc.
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

#include "lidar_centerpoint/ros_utils.hpp"

#include <gtest/gtest.h>

TEST(TestSuite, box3DToDetectedObject)
{
  std::vector<std::string> class_names = {"CAR",     "TRUCK",     "BUS",       "TRAILER",
                                          "BICYCLE", "MOTORBIKE", "PEDESTRIAN"};

  // Test case 1: Test with valid label, has_twist=true, has_variance=true
  {
    centerpoint::Box3D box3d;
    box3d.score = 0.8f;
    box3d.label = 0;  // CAR
    box3d.x = 1.0;
    box3d.y = 2.0;
    box3d.z = 3.0;
    box3d.yaw = 0.5;
    box3d.length = 4.0;
    box3d.width = 2.0;
    box3d.height = 1.5;
    box3d.vel_x = 1.0;
    box3d.vel_y = 0.5;
    box3d.x_variance = 0.1;
    box3d.y_variance = 0.2;
    box3d.z_variance = 0.3;
    box3d.yaw_variance = 0.4;
    box3d.vel_x_variance = 0.5;
    box3d.vel_y_variance = 0.6;

    autoware_auto_perception_msgs::msg::DetectedObject obj;
    centerpoint::box3DToDetectedObject(box3d, class_names, true, true, obj);

    EXPECT_FLOAT_EQ(obj.existence_probability, 0.8f);
    EXPECT_EQ(
      obj.classification[0].label, autoware_auto_perception_msgs::msg::ObjectClassification::CAR);
    EXPECT_FLOAT_EQ(obj.kinematics.pose_with_covariance.pose.position.x, 1.0);
    EXPECT_FLOAT_EQ(obj.kinematics.pose_with_covariance.pose.position.y, 2.0);
    EXPECT_FLOAT_EQ(obj.kinematics.pose_with_covariance.pose.position.z, 3.0);
    EXPECT_FLOAT_EQ(obj.shape.dimensions.x, 4.0);
    EXPECT_FLOAT_EQ(obj.shape.dimensions.y, 2.0);
    EXPECT_FLOAT_EQ(obj.shape.dimensions.z, 1.5);
    EXPECT_TRUE(obj.kinematics.has_position_covariance);
    EXPECT_TRUE(obj.kinematics.has_twist);
    EXPECT_TRUE(obj.kinematics.has_twist_covariance);
  }

  // Test case 2: Test with invalid label, has_twist=false, has_variance=false
  {
    centerpoint::Box3D box3d;
    box3d.score = 0.5f;
    box3d.label = 10;  // Invalid

    autoware_auto_perception_msgs::msg::DetectedObject obj;
    centerpoint::box3DToDetectedObject(box3d, class_names, false, false, obj);

    EXPECT_FLOAT_EQ(obj.existence_probability, 0.5f);
    EXPECT_EQ(
      obj.classification[0].label,
      autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN);
    EXPECT_FALSE(obj.kinematics.has_position_covariance);
    EXPECT_FALSE(obj.kinematics.has_twist);
    EXPECT_FALSE(obj.kinematics.has_twist_covariance);
  }
}

TEST(TestSuite, getSemanticType)
{
  EXPECT_EQ(
    centerpoint::getSemanticType("CAR"),
    autoware_auto_perception_msgs::msg::ObjectClassification::CAR);
  EXPECT_EQ(
    centerpoint::getSemanticType("TRUCK"),
    autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK);
  EXPECT_EQ(
    centerpoint::getSemanticType("BUS"),
    autoware_auto_perception_msgs::msg::ObjectClassification::BUS);
  EXPECT_EQ(
    centerpoint::getSemanticType("TRAILER"),
    autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER);
  EXPECT_EQ(
    centerpoint::getSemanticType("BICYCLE"),
    autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE);
  EXPECT_EQ(
    centerpoint::getSemanticType("MOTORBIKE"),
    autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE);
  EXPECT_EQ(
    centerpoint::getSemanticType("PEDESTRIAN"),
    autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN);
  EXPECT_EQ(
    centerpoint::getSemanticType("UNKNOWN"),
    autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN);
}

TEST(TestSuite, convertPoseCovarianceMatrix)
{
  centerpoint::Box3D box3d;
  box3d.x_variance = 0.1;
  box3d.y_variance = 0.2;
  box3d.z_variance = 0.3;
  box3d.yaw_variance = 0.4;

  std::array<double, 36> pose_covariance = centerpoint::convertPoseCovarianceMatrix(box3d);

  EXPECT_FLOAT_EQ(pose_covariance[0], 0.1);
  EXPECT_FLOAT_EQ(pose_covariance[7], 0.2);
  EXPECT_FLOAT_EQ(pose_covariance[14], 0.3);
  EXPECT_FLOAT_EQ(pose_covariance[35], 0.4);
}

TEST(TestSuite, convertTwistCovarianceMatrix)
{
  centerpoint::Box3D box3d;
  box3d.vel_x_variance = 0.1;
  box3d.vel_y_variance = 0.2;

  std::array<double, 36> twist_covariance = centerpoint::convertTwistCovarianceMatrix(box3d);

  EXPECT_FLOAT_EQ(twist_covariance[0], 0.1);
  EXPECT_FLOAT_EQ(twist_covariance[7], 0.2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
