// Copyright 2020 TierIV
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

#include "lidar_apollo_instance_segmentation/node.hpp"

#include "lidar_apollo_instance_segmentation/detector.hpp"

#include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

LidarInstanceSegmentationNode::LidarInstanceSegmentationNode(
  const rclcpp::NodeOptions & node_options)
: Node("lidar_apollo_instance_segmentation_node", node_options)
{
  using std::placeholders::_1;
  // initialize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "lidar_apollo_instance_segmentation");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }
  detector_ptr_ = std::make_shared<LidarApolloInstanceSegmentation>(this);
  debugger_ptr_ = std::make_shared<Debugger>(this);
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/pointcloud", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&LidarInstanceSegmentationNode::pointCloudCallback, this, _1));
  dynamic_objects_pub_ =
    this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
      "output/labeled_clusters", rclcpp::QoS{1});
}

void LidarInstanceSegmentationNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  stop_watch_ptr_->toc("processing_time", true);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output_msg;
  detector_ptr_->detectDynamicObjects(*msg, output_msg);
  dynamic_objects_pub_->publish(output_msg);
  debugger_ptr_->publishColoredPointCloud(output_msg);

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(LidarInstanceSegmentationNode)
