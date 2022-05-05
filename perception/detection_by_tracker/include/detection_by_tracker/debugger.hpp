// Copyright 2021 Tier IV, Inc.
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

#ifndef DETECTION_BY_TRACKER__DEBUGGER_HPP_
#define DETECTION_BY_TRACKER__DEBUGGER_HPP_

#include <euclidean_cluster/euclidean_cluster.hpp>
#include <euclidean_cluster/utils.hpp>
#include <euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_estimation/shape_estimator.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <vector>

class Debugger
{
public:
  explicit Debugger(rclcpp::Node * node)
  {
    initial_objects_pub_ =
      node->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
        "debug/initial_objects", 1);
    tracked_objects_pub_ =
      node->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
        "debug/tracked_objects", 1);
    merged_objects_pub_ =
      node->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
        "debug/merged_objects", 1);
    divided_objects_pub_ =
      node->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
        "debug/divided_objects", 1);
  }

  ~Debugger() {}
  void publishInitialObjects(const tier4_perception_msgs::msg::DetectedObjectsWithFeature & input)
  {
    initial_objects_pub_->publish(removeFeature(input));
  }
  void publishTrackedObjects(const autoware_auto_perception_msgs::msg::DetectedObjects & input)
  {
    tracked_objects_pub_->publish(input);
  }
  void publishMergedObjects(const tier4_perception_msgs::msg::DetectedObjectsWithFeature & input)
  {
    merged_objects_pub_->publish(removeFeature(input));
  }
  void publishDividedObjects(const tier4_perception_msgs::msg::DetectedObjectsWithFeature & input)
  {
    divided_objects_pub_->publish(removeFeature(input));
  }

private:
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr
    initial_objects_pub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr
    tracked_objects_pub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr
    merged_objects_pub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr
    divided_objects_pub_;

  autoware_auto_perception_msgs::msg::DetectedObjects removeFeature(
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature & input)
  {
    autoware_auto_perception_msgs::msg::DetectedObjects objects;
    objects.header = input.header;
    for (const auto & feature_object : input.feature_objects) {
      objects.objects.push_back(feature_object.object);
    }
    return objects;
  }
};

#endif  // DETECTION_BY_TRACKER__DEBUGGER_HPP_
