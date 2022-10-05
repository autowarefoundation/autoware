// Copyright 2015-2019 Autoware Foundation
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

#ifndef NDT_SCAN_MATCHER__TF2_LISTENER_MODULE_HPP_
#define NDT_SCAN_MATCHER__TF2_LISTENER_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

class Tf2ListenerModule
{
  using TransformStamped = geometry_msgs::msg::TransformStamped;

public:
  explicit Tf2ListenerModule(rclcpp::Node * node);
  bool get_transform(
    const builtin_interfaces::msg::Time & timestamp, const std::string & target_frame,
    const std::string & source_frame,
    const geometry_msgs::msg::TransformStamped::SharedPtr & transform_stamped_ptr) const;

private:
  rclcpp::Logger logger_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
};

#endif  // NDT_SCAN_MATCHER__TF2_LISTENER_MODULE_HPP_
