// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__LIDAR_TRANSFUSION__LIDAR_TRANSFUSION_NODE_HPP_
#define AUTOWARE__LIDAR_TRANSFUSION__LIDAR_TRANSFUSION_NODE_HPP_

#include "autoware/lidar_transfusion/detection_class_remapper.hpp"
#include "autoware/lidar_transfusion/postprocess/non_maximum_suppression.hpp"
#include "autoware/lidar_transfusion/preprocess/pointcloud_densification.hpp"
#include "autoware/lidar_transfusion/transfusion_trt.hpp"
#include "autoware/lidar_transfusion/visibility_control.hpp"

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/ros/published_time_publisher.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::lidar_transfusion
{

class LIDAR_TRANSFUSION_PUBLIC LidarTransfusionNode : public rclcpp::Node
{
public:
  explicit LidarTransfusionNode(const rclcpp::NodeOptions & options);

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr cloud_sub_{nullptr};
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_{
    nullptr};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  DetectionClassRemapper detection_class_remapper_;
  float score_threshold_{0.0};
  std::vector<std::string> class_names_;

  NonMaximumSuppression iou_bev_nms_;

  std::unique_ptr<TransfusionTRT> detector_ptr_{nullptr};

  // debugger
  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{
    nullptr};
  std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_ptr_{nullptr};
  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_pub_{nullptr};
};
}  // namespace autoware::lidar_transfusion

#endif  // AUTOWARE__LIDAR_TRANSFUSION__LIDAR_TRANSFUSION_NODE_HPP_
