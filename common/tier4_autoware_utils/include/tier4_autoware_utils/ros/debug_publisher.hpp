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

#ifndef TIER4_AUTOWARE_UTILS__ROS__DEBUG_PUBLISHER_HPP_
#define TIER4_AUTOWARE_UTILS__ROS__DEBUG_PUBLISHER_HPP_

#include "tier4_autoware_utils/ros/debug_traits.hpp"

#include <rclcpp/publisher_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace tier4_autoware_utils
{
namespace debug_publisher
{
template <
  class T_msg, class T,
  std::enable_if_t<
    tier4_autoware_utils::debug_traits::is_debug_message<T_msg>::value, std::nullptr_t> = nullptr>
T_msg toDebugMsg(const T & data, const rclcpp::Time & stamp)
{
  T_msg msg;
  msg.stamp = stamp;
  msg.data = data;
  return msg;
}
}  // namespace debug_publisher

class DebugPublisher
{
public:
  explicit DebugPublisher(rclcpp::Node * node, const char * ns) : node_(node), ns_(ns) {}

  template <
    class T,
    std::enable_if_t<rosidl_generator_traits::is_message<T>::value, std::nullptr_t> = nullptr>
  void publish(const std::string & name, const T & data, const rclcpp::QoS & qos = rclcpp::QoS(1))
  {
    if (pub_map_.count(name) == 0) {
      pub_map_[name] = node_->create_publisher<T>(std::string(ns_) + "/" + name, qos);
    }

    std::dynamic_pointer_cast<rclcpp::Publisher<T>>(pub_map_.at(name))->publish(data);
  }

  template <
    class T_msg, class T,
    std::enable_if_t<!rosidl_generator_traits::is_message<T>::value, std::nullptr_t> = nullptr>
  void publish(const std::string & name, const T & data, const rclcpp::QoS & qos = rclcpp::QoS(1))
  {
    publish(name, debug_publisher::toDebugMsg<T_msg>(data, node_->now()), qos);
  }

private:
  rclcpp::Node * node_;
  const char * ns_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::PublisherBase>> pub_map_;
};
}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__ROS__DEBUG_PUBLISHER_HPP_
