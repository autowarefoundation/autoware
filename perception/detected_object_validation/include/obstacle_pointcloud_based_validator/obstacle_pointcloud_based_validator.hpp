// Copyright 2022 Tier IV, Inc.
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

#ifndef OBSTACLE_POINTCLOUD_BASED_VALIDATOR__OBSTACLE_POINTCLOUD_BASED_VALIDATOR_HPP_
#define OBSTACLE_POINTCLOUD_BASED_VALIDATOR__OBSTACLE_POINTCLOUD_BASED_VALIDATOR_HPP_

#include "obstacle_pointcloud_based_validator/debugger.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>

namespace obstacle_pointcloud_based_validator
{
class ObstaclePointCloudBasedValidator : public rclcpp::Node
{
public:
  explicit ObstaclePointCloudBasedValidator(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_;
  message_filters::Subscriber<autoware_auto_perception_msgs::msg::DetectedObjects> objects_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> obstacle_pointcloud_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  typedef message_filters::sync_policies::ApproximateTime<
    autoware_auto_perception_msgs::msg::DetectedObjects, sensor_msgs::msg::PointCloud2>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;
  size_t min_pointcloud_num_;

  std::shared_ptr<Debugger> debugger_;

private:
  void onObjectsAndObstaclePointCloud(
    const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_obstacle_pointcloud);
  std::optional<size_t> getPointCloudNumWithinPolygon(
    const autoware_auto_perception_msgs::msg::DetectedObject & object,
    const pcl::PointCloud<pcl::PointXY>::Ptr pointcloud);
  std::optional<float> getMaxRadius(
    const autoware_auto_perception_msgs::msg::DetectedObject & object);
};
}  // namespace obstacle_pointcloud_based_validator

#endif  // OBSTACLE_POINTCLOUD_BASED_VALIDATOR__OBSTACLE_POINTCLOUD_BASED_VALIDATOR_HPP_
