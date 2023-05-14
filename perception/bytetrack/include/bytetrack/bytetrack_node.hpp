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

#ifndef BYTETRACK__BYTETRACK_NODE_HPP_
#define BYTETRACK__BYTETRACK_NODE_HPP_

#include <bytetrack/bytetrack.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_perception_msgs/msg/dynamic_object_array.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace bytetrack
{
using LabelMap = std::map<int, std::string>;

class ByteTrackNode : public rclcpp::Node
{
public:
  explicit ByteTrackNode(const rclcpp::NodeOptions & node_options);

private:
  void on_connect();
  void on_rect(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr msg);

  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr objects_pub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DynamicObjectArray>::SharedPtr objects_uuid_pub_;

  rclcpp::Subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    detection_rect_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<bytetrack::ByteTrack> bytetrack_;
};

}  // namespace bytetrack

#endif  // BYTETRACK__BYTETRACK_NODE_HPP_
