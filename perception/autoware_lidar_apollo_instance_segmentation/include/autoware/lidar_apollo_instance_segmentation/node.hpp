// Copyright 2020-2023 TIER IV, Inc.
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

#ifndef AUTOWARE__LIDAR_APOLLO_INSTANCE_SEGMENTATION__NODE_HPP_
#define AUTOWARE__LIDAR_APOLLO_INSTANCE_SEGMENTATION__NODE_HPP_

#include "autoware/lidar_apollo_instance_segmentation/debugger.hpp"

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <memory>

namespace autoware
{
namespace lidar_apollo_instance_segmentation
{
class LidarInstanceSegmentationInterface
{
public:
  LidarInstanceSegmentationInterface() {}
  virtual ~LidarInstanceSegmentationInterface() {}
  virtual bool detectDynamicObjects(
    const sensor_msgs::msg::PointCloud2 & input,
    tier4_perception_msgs::msg::DetectedObjectsWithFeature & output) = 0;
};

class LidarInstanceSegmentationNode : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    dynamic_objects_pub_;
  std::shared_ptr<LidarInstanceSegmentationInterface> detector_ptr_;
  std::shared_ptr<Debugger> debugger_ptr_;
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_;

public:
  explicit LidarInstanceSegmentationNode(const rclcpp::NodeOptions & node_options);
};
}  // namespace lidar_apollo_instance_segmentation
}  // namespace autoware
#endif  // AUTOWARE__LIDAR_APOLLO_INSTANCE_SEGMENTATION__NODE_HPP_
