// Copyright 2022 TIER IV, Inc.
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

#ifndef POSITION_FILTER__POSITION_FILTER_HPP_
#define POSITION_FILTER__POSITION_FILTER_HPP_

#include "autoware/detected_object_validation/utils/utils.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/ros/published_time_publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_map_msgs/msg/lanelet_map_bin.hpp"
#include "autoware_perception_msgs/msg/detected_objects.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace autoware::detected_object_validation
{
namespace position_filter
{

class ObjectPositionFilterNode : public rclcpp::Node
{
public:
  explicit ObjectPositionFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void objectCallback(const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr);

  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr object_pub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr object_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  float upper_bound_x_;
  float upper_bound_y_;
  float lower_bound_x_;
  float lower_bound_y_;
  utils::FilterTargetLabel filter_target_;
  bool isObjectInBounds(const autoware_perception_msgs::msg::DetectedObject & object) const;

  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_publisher_;
};

}  // namespace position_filter
}  // namespace autoware::detected_object_validation

#endif  // POSITION_FILTER__POSITION_FILTER_HPP_
