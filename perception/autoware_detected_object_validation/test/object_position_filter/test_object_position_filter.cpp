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

#include "../../src/position_filter/position_filter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

using autoware::detected_object_validation::position_filter::ObjectPositionFilterNode;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;

std::shared_ptr<autoware::test_utils::AutowareTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<autoware::test_utils::AutowareTestManager>();
  return test_manager;
}

std::shared_ptr<ObjectPositionFilterNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto detected_object_validation_dir =
    ament_index_cpp::get_package_share_directory("autoware_detected_object_validation");
  node_options.arguments(
    {"--ros-args", "--params-file",
     detected_object_validation_dir + "/config/object_position_filter.param.yaml"});
  return std::make_shared<ObjectPositionFilterNode>(node_options);
}

TEST(DetectedObjectValidationTest, testObjectPositionFilterEmptyObject)
{
  rclcpp::init(0, nullptr);
  const std::string input_topic = "/input/object";
  const std::string output_topic = "/output/object";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  int counter = 0;
  auto callback = [&counter](const DetectedObjects::ConstSharedPtr msg) {
    (void)msg;
    ++counter;
  };
  test_manager->set_subscriber<DetectedObjects>(output_topic, callback);

  DetectedObjects msg;
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, input_topic, msg);
  EXPECT_GE(counter, 1);
  rclcpp::shutdown();
}

TEST(DetectedObjectValidationTest, testObjectPositionFilterSeveralObjects)
{
  rclcpp::init(0, nullptr);
  const std::string input_topic = "/input/object";
  const std::string output_topic = "/output/object";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // Create a DetectedObjects message with several objects
  DetectedObjects msg;
  msg.header.frame_id = "base_link";

  // Object 1: Inside bounds
  {
    DetectedObject object;
    object.kinematics.pose_with_covariance.pose.position.x = 10.0;
    object.kinematics.pose_with_covariance.pose.position.y = 5.0;
    object.classification.resize(1);
    object.classification[0].label = ObjectClassification::UNKNOWN;
    msg.objects.push_back(object);
  }

  // Object 2: Outside bounds (x-axis)
  {
    DetectedObject object;
    object.kinematics.pose_with_covariance.pose.position.x = 110.0;
    object.kinematics.pose_with_covariance.pose.position.y = 5.0;
    object.classification.resize(1);
    object.classification[0].label = ObjectClassification::UNKNOWN;
    msg.objects.push_back(object);
  }

  // Object 3: Outside bounds (y-axis)
  {
    DetectedObject object;
    object.kinematics.pose_with_covariance.pose.position.x = 10.0;
    object.kinematics.pose_with_covariance.pose.position.y = 60.0;
    object.classification.resize(1);
    object.classification[0].label = ObjectClassification::UNKNOWN;
    msg.objects.push_back(object);
  }

  // Object 4: Inside bounds
  {
    DetectedObject object;
    object.kinematics.pose_with_covariance.pose.position.x = 20.0;
    object.kinematics.pose_with_covariance.pose.position.y = -5.0;
    object.classification.resize(1);
    object.classification[0].label = ObjectClassification::UNKNOWN;
    msg.objects.push_back(object);
  }

  DetectedObjects latest_msg;
  auto callback = [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; };
  test_manager->set_subscriber<DetectedObjects>(output_topic, callback);

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, input_topic, msg);
  EXPECT_EQ(latest_msg.objects.size(), 2);
  rclcpp::shutdown();
}
