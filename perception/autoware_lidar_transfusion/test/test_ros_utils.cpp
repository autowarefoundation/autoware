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

#include "autoware/lidar_transfusion/ros_utils.hpp"

#include <gtest/gtest.h>

TEST(TestSuite, box3DToDetectedObject)
{
  std::vector<std::string> class_names = {"CAR",     "TRUCK",      "BUS",       "TRAILER",
                                          "BICYCLE", "MOTORCYCLE", "PEDESTRIAN"};

  // Test case 1: Test with valid label
  {
    autoware::lidar_transfusion::Box3D box3d;
    box3d.label = 0;  // CAR
    box3d.score = 0.8f;
    box3d.x = 1.0;
    box3d.y = 2.0;
    box3d.z = 3.0;
    box3d.width = 2.0;
    box3d.height = 1.5;
    box3d.length = 4.0;
    box3d.yaw = 0.5;

    autoware_perception_msgs::msg::DetectedObject obj;
    autoware::lidar_transfusion::box3DToDetectedObject(box3d, class_names, obj);

    EXPECT_FLOAT_EQ(obj.existence_probability, 0.8f);
    EXPECT_EQ(
      obj.classification[0].label, autoware_perception_msgs::msg::ObjectClassification::CAR);
    EXPECT_FLOAT_EQ(obj.kinematics.pose_with_covariance.pose.position.x, 1.0);
    EXPECT_FLOAT_EQ(obj.kinematics.pose_with_covariance.pose.position.y, 2.0);
    EXPECT_FLOAT_EQ(obj.kinematics.pose_with_covariance.pose.position.z, 3.0);
    EXPECT_FLOAT_EQ(obj.shape.dimensions.x, 4.0);
    EXPECT_FLOAT_EQ(obj.shape.dimensions.y, 2.0);
    EXPECT_FLOAT_EQ(obj.shape.dimensions.z, 1.5);
    EXPECT_FALSE(obj.kinematics.has_position_covariance);
    EXPECT_FALSE(obj.kinematics.has_twist);
    EXPECT_FALSE(obj.kinematics.has_twist_covariance);
  }

  // Test case 2: Test with invalid label
  {
    autoware::lidar_transfusion::Box3D box3d;
    box3d.score = 0.5f;
    box3d.label = 10;  // Invalid

    autoware_perception_msgs::msg::DetectedObject obj;
    autoware::lidar_transfusion::box3DToDetectedObject(box3d, class_names, obj);

    EXPECT_FLOAT_EQ(obj.existence_probability, 0.5f);
    EXPECT_EQ(
      obj.classification[0].label, autoware_perception_msgs::msg::ObjectClassification::UNKNOWN);
    EXPECT_FALSE(obj.kinematics.has_position_covariance);
    EXPECT_FALSE(obj.kinematics.has_twist);
    EXPECT_FALSE(obj.kinematics.has_twist_covariance);
  }
}

TEST(TestSuite, getSemanticType)
{
  EXPECT_EQ(
    autoware::lidar_transfusion::getSemanticType("CAR"),
    autoware_perception_msgs::msg::ObjectClassification::CAR);
  EXPECT_EQ(
    autoware::lidar_transfusion::getSemanticType("TRUCK"),
    autoware_perception_msgs::msg::ObjectClassification::TRUCK);
  EXPECT_EQ(
    autoware::lidar_transfusion::getSemanticType("BUS"),
    autoware_perception_msgs::msg::ObjectClassification::BUS);
  EXPECT_EQ(
    autoware::lidar_transfusion::getSemanticType("TRAILER"),
    autoware_perception_msgs::msg::ObjectClassification::TRAILER);
  EXPECT_EQ(
    autoware::lidar_transfusion::getSemanticType("MOTORCYCLE"),
    autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE);
  EXPECT_EQ(
    autoware::lidar_transfusion::getSemanticType("BICYCLE"),
    autoware_perception_msgs::msg::ObjectClassification::BICYCLE);
  EXPECT_EQ(
    autoware::lidar_transfusion::getSemanticType("PEDESTRIAN"),
    autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN);
  EXPECT_EQ(
    autoware::lidar_transfusion::getSemanticType("UNKNOWN"),
    autoware_perception_msgs::msg::ObjectClassification::UNKNOWN);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
