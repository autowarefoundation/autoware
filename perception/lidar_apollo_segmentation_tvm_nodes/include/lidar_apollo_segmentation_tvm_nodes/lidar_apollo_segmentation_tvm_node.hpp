// Copyright 2020-2022 Arm Ltd., TierIV
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

#ifndef LIDAR_APOLLO_SEGMENTATION_TVM_NODES__LIDAR_APOLLO_SEGMENTATION_TVM_NODE_HPP_
#define LIDAR_APOLLO_SEGMENTATION_TVM_NODES__LIDAR_APOLLO_SEGMENTATION_TVM_NODE_HPP_

#include <lidar_apollo_segmentation_tvm/lidar_apollo_segmentation_tvm.hpp>
#include <lidar_apollo_segmentation_tvm_nodes/visibility_control.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

namespace autoware
{
namespace perception
{
namespace lidar_apollo_segmentation_tvm_nodes
{
/// \brief Object detection node based on neural network inference.
class LIDAR_APOLLO_SEGMENTATION_TVM_NODES_PUBLIC ApolloLidarSegmentationNode : public rclcpp::Node
{
private:
  const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_sub_ptr;
  const rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    m_detected_pub_ptr;
  const std::shared_ptr<lidar_apollo_segmentation_tvm::ApolloLidarSegmentation> m_detector_ptr;
  /// \brief Main callback function.
  void LIDAR_APOLLO_SEGMENTATION_TVM_NODES_LOCAL
  pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr & msg);

public:
  /// \brief Constructor
  /// \param options Additional options to control creation of the node.
  explicit ApolloLidarSegmentationNode(const rclcpp::NodeOptions & options);
};
}  // namespace lidar_apollo_segmentation_tvm_nodes
}  // namespace perception
}  // namespace autoware
#endif  // LIDAR_APOLLO_SEGMENTATION_TVM_NODES__LIDAR_APOLLO_SEGMENTATION_TVM_NODE_HPP_
