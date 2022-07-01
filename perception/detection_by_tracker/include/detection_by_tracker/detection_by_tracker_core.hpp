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

#ifndef DETECTION_BY_TRACKER__DETECTION_BY_TRACKER_CORE_HPP_
#define DETECTION_BY_TRACKER__DETECTION_BY_TRACKER_CORE_HPP_

#include "detection_by_tracker/debugger.hpp"

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

class TrackerHandler
{
private:
  std::deque<autoware_auto_perception_msgs::msg::TrackedObjects> objects_buffer_;

public:
  TrackerHandler() = default;
  void onTrackedObjects(
    const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr input_objects_msg);
  bool estimateTrackedObjects(
    const rclcpp::Time & time, autoware_auto_perception_msgs::msg::TrackedObjects & output);
};

class DetectionByTracker : public rclcpp::Node
{
public:
  explicit DetectionByTracker(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrackedObjects>::SharedPtr trackers_sub_;
  rclcpp::Subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    initial_objects_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  TrackerHandler tracker_handler_;
  std::shared_ptr<ShapeEstimator> shape_estimator_;
  std::shared_ptr<euclidean_cluster::EuclideanClusterInterface> cluster_;
  std::shared_ptr<Debugger> debugger_;

  bool ignore_unknown_tracker_;

  void onObjects(
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_msg);

  void divideUnderSegmentedObjects(
    const autoware_auto_perception_msgs::msg::DetectedObjects & tracked_objects,
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature & in_objects,
    autoware_auto_perception_msgs::msg::DetectedObjects & out_no_found_tracked_objects,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature & out_objects);

  float optimizeUnderSegmentedObject(
    const autoware_auto_perception_msgs::msg::DetectedObject & target_object,
    const sensor_msgs::msg::PointCloud2 & under_segmented_cluster,
    tier4_perception_msgs::msg::DetectedObjectWithFeature & output);

  void mergeOverSegmentedObjects(
    const autoware_auto_perception_msgs::msg::DetectedObjects & tracked_objects,
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature & in_objects,
    autoware_auto_perception_msgs::msg::DetectedObjects & out_no_found_tracked_objects,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature & out_objects);
};

#endif  // DETECTION_BY_TRACKER__DETECTION_BY_TRACKER_CORE_HPP_
