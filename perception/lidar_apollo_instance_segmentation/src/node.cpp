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

LidarInstanceSegmentationNode::LidarInstanceSegmentationNode(
  const rclcpp::NodeOptions & node_options)
: Node("lidar_apollo_instance_segmentation_node", node_options)
{
  using std::placeholders::_1;
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
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output_msg;
  detector_ptr_->detectDynamicObjects(*msg, output_msg);
  dynamic_objects_pub_->publish(output_msg);
  debugger_ptr_->publishColoredPointCloud(output_msg);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(LidarInstanceSegmentationNode)
