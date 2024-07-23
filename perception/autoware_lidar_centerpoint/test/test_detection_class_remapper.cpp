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

#include <autoware/lidar_centerpoint/detection_class_remapper.hpp>

#include <gtest/gtest.h>

TEST(DetectionClassRemapperTest, MapClasses)
{
  autoware::lidar_centerpoint::DetectionClassRemapper remapper;

  // Set up the parameters for the remapper
  // Labels: CAR, TRUCK, TRAILER
  std::vector<int64_t> allow_remapping_by_area_matrix = {
    0, 0, 0,  // CAR cannot be remapped
    0, 0, 1,  // TRUCK can be remapped to TRAILER
    0, 1, 0   // TRAILER can be remapped to TRUCK
  };
  std::vector<double> min_area_matrix = {0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0};
  std::vector<double> max_area_matrix = {0.0, 0.0, 0.0, 0.0, 0.0, 999.0, 0.0, 10.0, 0.0};

  remapper.setParameters(allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  // Create a DetectedObjects message with some objects
  autoware_perception_msgs::msg::DetectedObjects msg;

  // CAR with area 4.0, which is out of the range for remapping
  autoware_perception_msgs::msg::DetectedObject obj1;
  autoware_perception_msgs::msg::ObjectClassification obj1_classification;
  obj1.shape.dimensions.x = 2.0;
  obj1.shape.dimensions.y = 2.0;
  obj1_classification.label = 0;
  obj1_classification.probability = 1.0;
  obj1.classification = {obj1_classification};
  msg.objects.push_back(obj1);

  // TRUCK with area 16.0, which is in the range for remapping to TRAILER
  autoware_perception_msgs::msg::DetectedObject obj2;
  autoware_perception_msgs::msg::ObjectClassification obj2_classification;
  obj2.shape.dimensions.x = 8.0;
  obj2.shape.dimensions.y = 2.0;
  obj2_classification.label = 1;
  obj2_classification.probability = 1.0;
  obj2.classification = {obj2_classification};
  msg.objects.push_back(obj2);

  // TRAILER with area 9.0, which is in the range for remapping to TRUCK
  autoware_perception_msgs::msg::DetectedObject obj3;
  autoware_perception_msgs::msg::ObjectClassification obj3_classification;
  obj3.shape.dimensions.x = 3.0;
  obj3.shape.dimensions.y = 3.0;
  obj3_classification.label = 2;
  obj3_classification.probability = 1.0;
  obj3.classification = {obj3_classification};
  msg.objects.push_back(obj3);

  // TRAILER with area 12.0, which is out of the range for remapping
  autoware_perception_msgs::msg::DetectedObject obj4;
  autoware_perception_msgs::msg::ObjectClassification obj4_classification;
  obj4.shape.dimensions.x = 4.0;
  obj4.shape.dimensions.y = 3.0;
  obj4_classification.label = 2;
  obj4_classification.probability = 1.0;
  obj4.classification = {obj4_classification};
  msg.objects.push_back(obj4);

  // Call the mapClasses function
  remapper.mapClasses(msg);

  // Check the remapped labels
  EXPECT_EQ(msg.objects[0].classification[0].label, 0);
  EXPECT_EQ(msg.objects[1].classification[0].label, 2);
  EXPECT_EQ(msg.objects[2].classification[0].label, 1);
  EXPECT_EQ(msg.objects[3].classification[0].label, 2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
