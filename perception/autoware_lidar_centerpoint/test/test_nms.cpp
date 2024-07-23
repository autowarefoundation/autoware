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

#include "autoware/lidar_centerpoint/postprocess/non_maximum_suppression.hpp"

#include <gtest/gtest.h>

TEST(NonMaximumSuppressionTest, Apply)
{
  autoware::lidar_centerpoint::NonMaximumSuppression nms;
  autoware::lidar_centerpoint::NMSParams params;
  params.search_distance_2d_ = 1.0;
  params.iou_threshold_ = 0.2;
  params.nms_type_ = autoware::lidar_centerpoint::NMS_TYPE::IoU_BEV;
  params.target_class_names_ = {"CAR"};
  nms.setParameters(params);

  std::vector<autoware::lidar_centerpoint::DetectedObject> input_objects(4);

  // Object 1
  autoware_perception_msgs::msg::ObjectClassification obj1_classification;
  obj1_classification.label = 0;  // Assuming "car" has label 0
  obj1_classification.probability = 1.0;
  input_objects[0].classification = {obj1_classification};  // Assuming "car" has label 0
  input_objects[0].kinematics.pose_with_covariance.pose.position.x = 0.0;
  input_objects[0].kinematics.pose_with_covariance.pose.position.y = 0.0;
  input_objects[0].kinematics.pose_with_covariance.pose.orientation.x = 0.0;
  input_objects[0].kinematics.pose_with_covariance.pose.orientation.y = 0.0;
  input_objects[0].kinematics.pose_with_covariance.pose.orientation.z = 0.0;
  input_objects[0].kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  input_objects[0].shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  input_objects[0].shape.dimensions.x = 4.0;
  input_objects[0].shape.dimensions.y = 2.0;

  // Object 2 (overlaps with Object 1)
  autoware_perception_msgs::msg::ObjectClassification obj2_classification;
  obj2_classification.label = 0;  // Assuming "car" has label 0
  obj2_classification.probability = 1.0;
  input_objects[1].classification = {obj2_classification};  // Assuming "car" has label 0
  input_objects[1].kinematics.pose_with_covariance.pose.position.x = 0.5;
  input_objects[1].kinematics.pose_with_covariance.pose.position.y = 0.5;
  input_objects[1].kinematics.pose_with_covariance.pose.orientation.x = 0.0;
  input_objects[1].kinematics.pose_with_covariance.pose.orientation.y = 0.0;
  input_objects[1].kinematics.pose_with_covariance.pose.orientation.z = 0.0;
  input_objects[1].kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  input_objects[1].shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  input_objects[1].shape.dimensions.x = 4.0;
  input_objects[1].shape.dimensions.y = 2.0;

  // Object 3
  autoware_perception_msgs::msg::ObjectClassification obj3_classification;
  obj3_classification.label = 0;  // Assuming "car" has label 0
  obj3_classification.probability = 1.0;
  input_objects[2].classification = {obj3_classification};  // Assuming "car" has label 0
  input_objects[2].kinematics.pose_with_covariance.pose.position.x = 5.0;
  input_objects[2].kinematics.pose_with_covariance.pose.position.y = 5.0;
  input_objects[2].kinematics.pose_with_covariance.pose.orientation.x = 0.0;
  input_objects[2].kinematics.pose_with_covariance.pose.orientation.y = 0.0;
  input_objects[2].kinematics.pose_with_covariance.pose.orientation.z = 0.0;
  input_objects[2].kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  input_objects[2].shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  input_objects[2].shape.dimensions.x = 4.0;
  input_objects[2].shape.dimensions.y = 2.0;

  // Object 4 (different class)
  autoware_perception_msgs::msg::ObjectClassification obj4_classification;
  obj4_classification.label = 1;  // Assuming "pedestrian" has label 1
  obj4_classification.probability = 1.0;
  input_objects[3].classification = {obj4_classification};  // Assuming "pedestrian" has label 1
  input_objects[3].kinematics.pose_with_covariance.pose.position.x = 0.0;
  input_objects[3].kinematics.pose_with_covariance.pose.position.y = 0.0;
  input_objects[3].kinematics.pose_with_covariance.pose.orientation.x = 0.0;
  input_objects[3].kinematics.pose_with_covariance.pose.orientation.y = 0.0;
  input_objects[3].kinematics.pose_with_covariance.pose.orientation.z = 0.0;
  input_objects[3].kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  input_objects[3].shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  input_objects[3].shape.dimensions.x = 0.5;
  input_objects[3].shape.dimensions.y = 0.5;

  std::vector<autoware::lidar_centerpoint::DetectedObject> output_objects =
    nms.apply(input_objects);

  // Assert the expected number of output objects
  EXPECT_EQ(output_objects.size(), 3);

  // Assert that input_objects[2] is included in the output_objects
  bool is_input_object_2_included = false;
  for (const auto & output_object : output_objects) {
    if (output_object == input_objects[2]) {
      is_input_object_2_included = true;
      break;
    }
  }
  EXPECT_TRUE(is_input_object_2_included);

  // Assert that input_objects[3] is included in the output_objects
  bool is_input_object_3_included = false;
  for (const auto & output_object : output_objects) {
    if (output_object == input_objects[3]) {
      is_input_object_3_included = true;
      break;
    }
  }
  EXPECT_TRUE(is_input_object_3_included);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
