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

#include "detected_object_filter/object_position_filter.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

namespace object_position_filter
{
ObjectPositionFilterNode::ObjectPositionFilterNode(const rclcpp::NodeOptions & node_options)
: Node("object_position_filter_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  // Set parameters
  upper_bound_x_ = declare_parameter<float>("upper_bound_x", 100.0);
  upper_bound_y_ = declare_parameter<float>("upper_bound_y", 50.0);
  lower_bound_x_ = declare_parameter<float>("lower_bound_x", 0.0);
  lower_bound_y_ = declare_parameter<float>("lower_bound_y", -50.0);
  filter_target_.UNKNOWN = declare_parameter<bool>("filter_target_label.UNKNOWN", false);
  filter_target_.CAR = declare_parameter<bool>("filter_target_label.CAR", false);
  filter_target_.TRUCK = declare_parameter<bool>("filter_target_label.TRUCK", false);
  filter_target_.BUS = declare_parameter<bool>("filter_target_label.BUS", false);
  filter_target_.TRAILER = declare_parameter<bool>("filter_target_label.TRAILER", false);
  filter_target_.MOTORCYCLE = declare_parameter<bool>("filter_target_label.MOTORCYCLE", false);
  filter_target_.BICYCLE = declare_parameter<bool>("filter_target_label.BICYCLE", false);
  filter_target_.PEDESTRIAN = declare_parameter<bool>("filter_target_label.PEDESTRIAN", false);

  // Set publisher/subscriber
  object_sub_ = this->create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "input/object", rclcpp::QoS{1}, std::bind(&ObjectPositionFilterNode::objectCallback, this, _1));
  object_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "output/object", rclcpp::QoS{1});
}

void ObjectPositionFilterNode::objectCallback(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_msg)
{
  // Guard
  if (object_pub_->get_subscription_count() < 1) return;

  autoware_auto_perception_msgs::msg::DetectedObjects output_object_msg;
  output_object_msg.header = input_msg->header;

  for (const auto & object : input_msg->objects) {
    const auto & label = object.classification.front().label;
    if (filter_target_.isTarget(label)) {
      if (isObjectInBounds(object)) {
        output_object_msg.objects.emplace_back(object);
      }
    } else {
      output_object_msg.objects.emplace_back(object);
    }
  }

  object_pub_->publish(output_object_msg);
}

bool ObjectPositionFilterNode::isObjectInBounds(
  const autoware_auto_perception_msgs::msg::DetectedObject & object) const
{
  const auto & position = object.kinematics.pose_with_covariance.pose.position;
  return position.x > lower_bound_x_ && position.x < upper_bound_x_ &&
         position.y > lower_bound_y_ && position.y < upper_bound_y_;
}

}  // namespace object_position_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(object_position_filter::ObjectPositionFilterNode)
