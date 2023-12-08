// Copyright 2023 The Autoware Foundation
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

#include <autoware_auto_msgs_adapter_core.hpp>

#include <gtest/gtest.h>

#include <random>

autoware_perception_msgs::msg::PredictedObjects generate_perception_msg()
{
  // generate deterministic random int
  std::mt19937 gen(0);
  std::uniform_int_distribution<> dis_int(0, 1000000);
  auto rand_int = [&dis_int, &gen]() { return dis_int(gen); };

  autoware_perception_msgs::msg::PredictedObjects msg_perception;
  msg_perception.header.stamp = rclcpp::Time(rand_int());
  msg_perception.header.frame_id = "test_frame";

  autoware_perception_msgs::msg::PredictedObject obj;
  // // {
  unique_identifier_msgs::msg::UUID uuid_;
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(uuid_.uuid.begin(), uuid_.uuid.end(), bit_eng);
  obj.object_id = uuid_;
  obj.existence_probability = 0.5;
  autoware_perception_msgs::msg::ObjectClassification obj_class;
  obj_class.label = 0;
  obj_class.probability = 0.5;
  obj.classification.push_back(obj_class);

  // {
  autoware_perception_msgs::msg::PredictedObjectKinematics kin;
  kin.initial_pose_with_covariance.pose.position.x = 10;
  kin.initial_pose_with_covariance.pose.position.y = 10;
  kin.initial_pose_with_covariance.pose.position.z = 0;
  kin.initial_pose_with_covariance.pose.orientation.x = 0;
  kin.initial_pose_with_covariance.pose.orientation.y = 0;
  kin.initial_pose_with_covariance.pose.orientation.z = 0;
  kin.initial_pose_with_covariance.pose.orientation.w = 1;

  kin.initial_twist_with_covariance.twist.linear.x = 1;
  kin.initial_twist_with_covariance.twist.linear.y = 0;
  kin.initial_twist_with_covariance.twist.linear.z = 0;
  kin.initial_twist_with_covariance.twist.angular.x = 0;
  kin.initial_twist_with_covariance.twist.angular.y = 0;
  kin.initial_twist_with_covariance.twist.angular.z = 0;

  kin.initial_acceleration_with_covariance.accel.linear.x = 0;
  kin.initial_acceleration_with_covariance.accel.linear.y = 0;
  kin.initial_acceleration_with_covariance.accel.linear.z = 0;
  kin.initial_acceleration_with_covariance.accel.angular.x = 0;
  kin.initial_acceleration_with_covariance.accel.angular.y = 0;
  kin.initial_acceleration_with_covariance.accel.angular.z = 0;

  constexpr size_t path_size = 10;
  kin.predicted_paths.resize(1);
  kin.predicted_paths[0].path.resize(path_size);
  for (size_t i = 0; i < path_size; i++) {
    kin.predicted_paths[0].path[i].position.x = i;
    kin.predicted_paths[0].path[i].position.y = 0;
    kin.predicted_paths[0].path[i].position.z = 0;
  }
  obj.kinematics = kin;
  // }
  // {
  autoware_perception_msgs::msg::Shape s;
  s.type = 1;
  geometry_msgs::msg::Point32 p;
  p.x = 9.0f;
  p.y = 11.0f;
  p.z = 0.0f;
  s.footprint.points.push_back(p);
  p.x = 11.0f;
  p.y = 11.0f;
  p.z = 0.0f;
  s.footprint.points.push_back(p);
  p.x = 11.0f;
  p.y = 9.0f;
  p.z = 0.0f;
  s.footprint.points.push_back(p);
  p.x = 9.0f;
  p.y = 9.0f;
  p.z = 0.0f;
  s.footprint.points.push_back(p);

  s.dimensions.x = 2;
  s.dimensions.y = 2;
  s.dimensions.z = 2;

  obj.shape = s;
  // }
  // }

  msg_perception.objects.push_back(obj);
  return msg_perception;
}

TEST(AutowareAutoMsgsAdapter, TestPredictedObjects)  // NOLINT for gtest
{
  const std::string msg_type_target = "autoware_auto_perception_msgs/msg/PredictedObjects";
  const std::string topic_name_source = "topic_name_source";
  const std::string topic_name_target = "topic_name_target";

  std::cout << "Creating the adapter node..." << std::endl;

  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("msg_type_target", msg_type_target);
  node_options.append_parameter_override("topic_name_source", topic_name_source);
  node_options.append_parameter_override("topic_name_target", topic_name_target);

  using autoware_auto_msgs_adapter::AutowareAutoMsgsAdapterNode;
  AutowareAutoMsgsAdapterNode::SharedPtr node_adapter;
  node_adapter = std::make_shared<AutowareAutoMsgsAdapterNode>(node_options);

  std::cout << "Creating the subscriber node..." << std::endl;

  auto node_subscriber = std::make_shared<rclcpp::Node>("node_subscriber", rclcpp::NodeOptions{});

  bool test_completed = false;

  const auto msg_perception = generate_perception_msg();
  auto sub = node_subscriber->create_subscription<
    autoware_auto_perception_msgs::msg::PredictedObjects>(
    topic_name_target, rclcpp::QoS{1},
    [&msg_perception,
     &test_completed](const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
      EXPECT_EQ(msg->header.stamp, msg_perception.header.stamp);
      EXPECT_EQ(msg->header.frame_id, msg_perception.header.frame_id);
      EXPECT_EQ(msg->objects[0].object_id.uuid, msg_perception.objects[0].object_id.uuid);
      EXPECT_FLOAT_EQ(
        msg->objects[0].existence_probability, msg_perception.objects[0].existence_probability);
      EXPECT_EQ(
        msg->objects[0].classification[0].label, msg_perception.objects[0].classification[0].label);
      EXPECT_FLOAT_EQ(
        msg->objects[0].classification[0].probability,
        msg_perception.objects[0].classification[0].probability);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_pose_with_covariance.pose.position.x,
        msg_perception.objects[0].kinematics.initial_pose_with_covariance.pose.position.x);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_pose_with_covariance.pose.position.y,
        msg_perception.objects[0].kinematics.initial_pose_with_covariance.pose.position.y);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_pose_with_covariance.pose.position.z,
        msg_perception.objects[0].kinematics.initial_pose_with_covariance.pose.position.z);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_pose_with_covariance.pose.orientation.x,
        msg_perception.objects[0].kinematics.initial_pose_with_covariance.pose.orientation.x);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_pose_with_covariance.pose.orientation.y,
        msg_perception.objects[0].kinematics.initial_pose_with_covariance.pose.orientation.y);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_pose_with_covariance.pose.orientation.z,
        msg_perception.objects[0].kinematics.initial_pose_with_covariance.pose.orientation.z);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_pose_with_covariance.pose.orientation.w,
        msg_perception.objects[0].kinematics.initial_pose_with_covariance.pose.orientation.w);

      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_twist_with_covariance.twist.linear.x,
        msg_perception.objects[0].kinematics.initial_twist_with_covariance.twist.linear.x);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_twist_with_covariance.twist.linear.y,
        msg_perception.objects[0].kinematics.initial_twist_with_covariance.twist.linear.y);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_twist_with_covariance.twist.linear.z,
        msg_perception.objects[0].kinematics.initial_twist_with_covariance.twist.linear.z);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_twist_with_covariance.twist.angular.x,
        msg_perception.objects[0].kinematics.initial_twist_with_covariance.twist.angular.x);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_twist_with_covariance.twist.angular.y,
        msg_perception.objects[0].kinematics.initial_twist_with_covariance.twist.angular.y);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_twist_with_covariance.twist.angular.z,
        msg_perception.objects[0].kinematics.initial_twist_with_covariance.twist.angular.z);

      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_acceleration_with_covariance.accel.linear.x,
        msg_perception.objects[0].kinematics.initial_acceleration_with_covariance.accel.linear.x);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_acceleration_with_covariance.accel.linear.y,
        msg_perception.objects[0].kinematics.initial_acceleration_with_covariance.accel.linear.y);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_acceleration_with_covariance.accel.linear.z,
        msg_perception.objects[0].kinematics.initial_acceleration_with_covariance.accel.linear.z);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_acceleration_with_covariance.accel.angular.x,
        msg_perception.objects[0].kinematics.initial_acceleration_with_covariance.accel.angular.x);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_acceleration_with_covariance.accel.angular.y,
        msg_perception.objects[0].kinematics.initial_acceleration_with_covariance.accel.angular.y);
      EXPECT_FLOAT_EQ(
        msg->objects[0].kinematics.initial_acceleration_with_covariance.accel.angular.z,
        msg_perception.objects[0].kinematics.initial_acceleration_with_covariance.accel.angular.z);

      for (size_t i = 0; i < msg->objects[0].kinematics.predicted_paths[0].path.size(); i++) {
        EXPECT_FLOAT_EQ(
          msg->objects[0].kinematics.predicted_paths[0].path[i].position.x,
          msg_perception.objects[0].kinematics.predicted_paths[0].path[i].position.x);
        EXPECT_FLOAT_EQ(
          msg->objects[0].kinematics.predicted_paths[0].path[i].position.y,
          msg_perception.objects[0].kinematics.predicted_paths[0].path[i].position.y);
        EXPECT_FLOAT_EQ(
          msg->objects[0].kinematics.predicted_paths[0].path[i].position.z,
          msg_perception.objects[0].kinematics.predicted_paths[0].path[i].position.z);
      }

      EXPECT_EQ(msg->objects[0].shape.type, msg_perception.objects[0].shape.type);
      for (size_t i = 0; i < msg_perception.objects[0].shape.footprint.points.size(); i++) {
        EXPECT_FLOAT_EQ(
          msg->objects[0].shape.footprint.points[i].x,
          msg_perception.objects[0].shape.footprint.points[i].x);
        EXPECT_FLOAT_EQ(
          msg->objects[0].shape.footprint.points[i].y,
          msg_perception.objects[0].shape.footprint.points[i].y);
        EXPECT_FLOAT_EQ(
          msg->objects[0].shape.footprint.points[i].z,
          msg_perception.objects[0].shape.footprint.points[i].z);
      }
      EXPECT_FLOAT_EQ(
        msg->objects[0].shape.dimensions.x, msg_perception.objects[0].shape.dimensions.x);
      EXPECT_FLOAT_EQ(
        msg->objects[0].shape.dimensions.y, msg_perception.objects[0].shape.dimensions.y);
      EXPECT_FLOAT_EQ(
        msg->objects[0].shape.dimensions.z, msg_perception.objects[0].shape.dimensions.z);

      test_completed = true;
    });

  std::cout << "Creating the publisher node..." << std::endl;

  auto node_publisher = std::make_shared<rclcpp::Node>("node_publisher", rclcpp::NodeOptions{});
  auto pub = node_publisher->create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
    topic_name_source, rclcpp::QoS{1});
  pub->publish(msg_perception);

  auto start_time = std::chrono::system_clock::now();
  auto max_test_dur = std::chrono::seconds(5);
  auto timed_out = false;

  while (rclcpp::ok() && !test_completed) {
    rclcpp::spin_some(node_subscriber);
    rclcpp::spin_some(node_adapter);
    rclcpp::spin_some(node_publisher);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    if (std::chrono::system_clock::now() - start_time > max_test_dur) {
      timed_out = true;
      break;
    }
  }

  EXPECT_TRUE(test_completed);
  EXPECT_FALSE(timed_out);

  //   rclcpp::shutdown();
}
