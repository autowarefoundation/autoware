// Copyright 2020 Tier IV, Inc.
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

#include "voxel_grid_based_euclidean_cluster_node.hpp"

#include "autoware/euclidean_cluster/utils.hpp"

#include <vector>

namespace autoware::euclidean_cluster
{
VoxelGridBasedEuclideanClusterNode::VoxelGridBasedEuclideanClusterNode(
  const rclcpp::NodeOptions & options)
: Node("voxel_grid_based_euclidean_cluster_node", options)
{
  const bool use_height = this->declare_parameter("use_height", false);
  const int min_cluster_size = this->declare_parameter("min_cluster_size", 1);
  const int max_cluster_size = this->declare_parameter("max_cluster_size", 500);
  const float tolerance = this->declare_parameter("tolerance", 1.0);
  const float voxel_leaf_size = this->declare_parameter("voxel_leaf_size", 0.5);
  const int min_points_number_per_voxel = this->declare_parameter("min_points_number_per_voxel", 3);
  cluster_ = std::make_shared<VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel);

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&VoxelGridBasedEuclideanClusterNode::onPointCloud, this, _1));

  cluster_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "output", rclcpp::QoS{1});
  debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/clusters", 1);
  stop_watch_ptr_ =
    std::make_unique<autoware::universe_utils::StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ = std::make_unique<autoware::universe_utils::DebugPublisher>(
    this, "voxel_grid_based_euclidean_cluster");
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
}

void VoxelGridBasedEuclideanClusterNode::onPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  stop_watch_ptr_->toc("processing_time", true);

  // convert ros to pcl
  if (input_msg->data.empty()) {
    // NOTE: prevent pcl log spam
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Empty sensor points!");
  }
  // cluster and build output msg
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output;

  cluster_->cluster(input_msg, output);
  cluster_pub_->publish(output);

  // build debug msg
  if (debug_pub_->get_subscription_count() >= 1) {
    sensor_msgs::msg::PointCloud2 debug;
    convertObjectMsg2SensorMsg(output, debug);
    debug_pub_->publish(debug);
  }
  if (debug_publisher_) {
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - output.header.stamp).nanoseconds()))
        .count();
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }
}

}  // namespace autoware::euclidean_cluster

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::euclidean_cluster::VoxelGridBasedEuclideanClusterNode)
