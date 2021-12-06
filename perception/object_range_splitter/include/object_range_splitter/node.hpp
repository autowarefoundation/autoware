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

#ifndef OBJECT_RANGE_SPLITTER__NODE_HPP_
#define OBJECT_RANGE_SPLITTER__NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

#include <memory>

namespace object_range_splitter
{
class ObjectRangeSplitterNode : public rclcpp::Node
{
public:
  explicit ObjectRangeSplitterNode(const rclcpp::NodeOptions & node_options);

private:
  void objectCallback(
    const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_msg);

  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr
    long_range_object_pub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr
    short_range_object_pub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr sub_;

  // ROS Parameters
  float spilt_range_;
};

}  // namespace object_range_splitter

#endif  // OBJECT_RANGE_SPLITTER__NODE_HPP_
