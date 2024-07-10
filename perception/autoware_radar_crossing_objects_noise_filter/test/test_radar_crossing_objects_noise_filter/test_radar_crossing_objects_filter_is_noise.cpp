// Copyright 2024 Tier IV, Inc.
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

#include "../../src/radar_crossing_objects_noise_filter_node.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/math/unit_conversion.hpp"

#include <geometry_msgs/msg/point32.hpp>

#include <gtest/gtest.h>

std::shared_ptr<autoware::radar_crossing_objects_noise_filter::RadarCrossingObjectsNoiseFilterNode>
get_node(double angle_threshold, double velocity_threshold)
{
  rclcpp::NodeOptions node_options;

  node_options.parameter_overrides(
    {{"angle_threshold", angle_threshold}, {"velocity_threshold", velocity_threshold}});
  auto node = std::make_shared<
    autoware::radar_crossing_objects_noise_filter::RadarCrossingObjectsNoiseFilterNode>(
    node_options);
  return node;
}

autoware_perception_msgs::msg::DetectedObject get_object(
  geometry_msgs::msg::Vector3 velocity, geometry_msgs::msg::Point position,
  geometry_msgs::msg::Quaternion orientation)
{
  autoware_perception_msgs::msg::DetectedObject object;
  object.kinematics.twist_with_covariance.twist.linear = velocity;
  object.kinematics.pose_with_covariance.pose.position = position;
  object.kinematics.pose_with_covariance.pose.orientation = orientation;
  return object;
}

TEST(RadarCrossingObjectsFilter, IsNoise)
{
  rclcpp::init(0, nullptr);
  {
    auto velocity = autoware::universe_utils::createVector3(40.0, 30.0, 0.0);
    auto position = autoware::universe_utils::createPoint(1.0, 0.0, 0.0);
    auto orientation = autoware::universe_utils::createQuaternion(1.0, 1.0, 1.0, 0.0);
    auto object = get_object(velocity, position, orientation);
    {
      double velocity_threshold = 40.0;
      double angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_TRUE(node->isNoise(object));
    }
    {
      double velocity_threshold = 40.0;
      double angle_threshold = -1.0472;

      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_TRUE(node->isNoise(object));
    }
    {
      double velocity_threshold = -40.0;
      double angle_threshold = 1.0472;

      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_TRUE(node->isNoise(object));
    }
    {
      double velocity_threshold = -40.0;
      double angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_TRUE(node->isNoise(object));
    }
  }

  {
    auto velocity = autoware::universe_utils::createVector3(40.0, 30.0, 0.0);
    auto position = autoware::universe_utils::createPoint(1.0, 2.0, 0.0);
    auto orientation = autoware::universe_utils::createQuaternion(1.0, 1.0, 1.0, 0.0);
    auto object = get_object(velocity, position, orientation);
    {
      double velocity_threshold = 40.0;
      double angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(object));
    }
    {
      double velocity_threshold = 40.0;
      double angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(object));
    }
    {
      double velocity_threshold = -40.0;
      double angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(object));
    }
    {
      double velocity_threshold = -40.0;
      double angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(object));
    }
  }

  {
    auto velocity = autoware::universe_utils::createVector3(24.0, 18.0, 0.0);
    auto position = autoware::universe_utils::createPoint(1.0, 0.0, 0.0);
    auto orientation = autoware::universe_utils::createQuaternion(1.0, 1.0, 1.0, 0.0);
    auto object = get_object(velocity, position, orientation);
    {
      double velocity_threshold = 40.0;
      double angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(object));
    }
    {
      double velocity_threshold = 40.0;
      double angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(object));
    }
    {
      double velocity_threshold = -40.0;
      double angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_TRUE(node->isNoise(object));
    }
    {
      double velocity_threshold = -40.0;
      double angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_TRUE(node->isNoise(object));
    }
  }

  {
    auto velocity = autoware::universe_utils::createVector3(24.0, 18.0, 0.0);
    auto position = autoware::universe_utils::createPoint(1.0, 2.0, 0.0);
    auto orientation = autoware::universe_utils::createQuaternion(1.0, 1.0, 1.0, 0.0);
    auto object = get_object(velocity, position, orientation);
    {
      double velocity_threshold = 40.0;
      double angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(object));
    }
    {
      double velocity_threshold = 40.0;
      double angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(object));
    }
    {
      double velocity_threshold = -40.0;
      double angle_threshold = 1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(object));
    }
    {
      double velocity_threshold = -40.0;
      double angle_threshold = -1.0472;
      auto node = get_node(angle_threshold, velocity_threshold);
      EXPECT_FALSE(node->isNoise(object));
    }
  }
}
