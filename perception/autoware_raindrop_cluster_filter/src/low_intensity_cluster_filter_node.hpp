// Copyright 2024 TIER IV, Inc.
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

#ifndef LOW_INTENSITY_CLUSTER_FILTER_NODE_HPP_
#define LOW_INTENSITY_CLUSTER_FILTER_NODE_HPP_

#include "autoware/detected_object_validation/utils/utils.hpp"
#include "autoware/universe_utils/ros/debug_publisher.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace autoware::low_intensity_cluster_filter
{

class LowIntensityClusterFilter : public rclcpp::Node
{
public:
  explicit LowIntensityClusterFilter(const rclcpp::NodeOptions & node_options);

private:
  void objectCallback(
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_msg);
  bool isValidatedCluster(const sensor_msgs::msg::PointCloud2 & cluster);

  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr object_pub_;
  rclcpp::Subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    object_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  double intensity_threshold_;
  double existence_probability_threshold_;
  double max_x_;
  double min_x_;
  double max_y_;
  double min_y_;

  double max_x_transformed_;
  double min_x_transformed_;
  double max_y_transformed_;
  double min_y_transformed_;
  // Eigen::Vector4f min_boundary_transformed_;
  // Eigen::Vector4f max_boundary_transformed_;
  bool is_validation_range_transformed_ = false;
  const std::string base_link_frame_id_ = "base_link";
  autoware::detected_object_validation::utils::FilterTargetLabel filter_target_;

  // debugger
  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{
    nullptr};
  std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_ptr_{nullptr};
};

}  // namespace autoware::low_intensity_cluster_filter

#endif  // LOW_INTENSITY_CLUSTER_FILTER_NODE_HPP_
