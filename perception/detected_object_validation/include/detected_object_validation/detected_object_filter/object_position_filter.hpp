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

#ifndef DETECTED_OBJECT_VALIDATION__DETECTED_OBJECT_FILTER__OBJECT_POSITION_FILTER_HPP_
#define DETECTED_OBJECT_VALIDATION__DETECTED_OBJECT_FILTER__OBJECT_POSITION_FILTER_HPP_

#include "detected_object_validation/utils/utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace object_position_filter
{

class ObjectPositionFilterNode : public rclcpp::Node
{
public:
  explicit ObjectPositionFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void objectCallback(const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr);

  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr object_pub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr object_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  float upper_bound_x_;
  float upper_bound_y_;
  float lower_bound_x_;
  float lower_bound_y_;
  utils::FilterTargetLabel filter_target_;
  bool isObjectInBounds(const autoware_auto_perception_msgs::msg::DetectedObject & object) const;
};

}  // namespace object_position_filter

#endif  // DETECTED_OBJECT_VALIDATION__DETECTED_OBJECT_FILTER__OBJECT_POSITION_FILTER_HPP_
