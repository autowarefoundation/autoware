// Copyright 2023 TIER IV, Inc.
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

#ifndef CLUSTER_MERGER__NODE_HPP_
#define CLUSTER_MERGER__NODE_HPP_

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/ros/transform_listener.hpp"

#include "tier4_perception_msgs/msg/detected_objects_with_feature.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace cluster_merger
{
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;

class ClusterMergerNode : public rclcpp::Node
{
public:
  explicit ClusterMergerNode(const rclcpp::NodeOptions & node_options);

private:
  // Subscriber

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr sub_objects_{};
  message_filters::Subscriber<DetectedObjectsWithFeature> objects0_sub_;
  message_filters::Subscriber<DetectedObjectsWithFeature> objects1_sub_;
  typedef message_filters::sync_policies::ApproximateTime<
    DetectedObjectsWithFeature, DetectedObjectsWithFeature>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;

  std::string output_frame_id_;

  std::vector<rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr> sub_objects_array{};
  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;

  void objectsCallback(
    const DetectedObjectsWithFeature::ConstSharedPtr & input_objects0_msg,
    const DetectedObjectsWithFeature::ConstSharedPtr & input_objects1_msg);
  // Publisher
  rclcpp::Publisher<DetectedObjectsWithFeature>::SharedPtr pub_objects_;
};

}  // namespace cluster_merger

#endif  // CLUSTER_MERGER__NODE_HPP_
