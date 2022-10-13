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

#ifndef COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_
#define COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_

#include <component_interface_utils/rclcpp/interface.hpp>
#include <component_interface_utils/rclcpp/service_client.hpp>
#include <component_interface_utils/rclcpp/service_server.hpp>
#include <component_interface_utils/rclcpp/topic_publisher.hpp>
#include <component_interface_utils/rclcpp/topic_subscription.hpp>
#include <component_interface_utils/specs.hpp>
#include <rclcpp/rclcpp.hpp>

#include <utility>

namespace component_interface_utils
{

/// Create a client wrapper for logging. This is a private implementation.
template <class SpecT>
typename Client<SpecT>::SharedPtr create_client_impl(
  NodeInterface::SharedPtr interface, rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  // This function is a wrapper for the following.
  // https://github.com/ros2/rclcpp/blob/48068130edbb43cdd61076dc1851672ff1a80408/rclcpp/include/rclcpp/node.hpp#L253-L265
  return Client<SpecT>::make_shared(interface, group);
}

/// Create a service wrapper for logging. This is a private implementation.
template <class SpecT, class CallbackT>
typename Service<SpecT>::SharedPtr create_service_impl(
  NodeInterface::SharedPtr interface, CallbackT && callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  // This function is a wrapper for the following.
  // https://github.com/ros2/rclcpp/blob/48068130edbb43cdd61076dc1851672ff1a80408/rclcpp/include/rclcpp/node.hpp#L267-L281
  return Service<SpecT>::make_shared(interface, std::forward<CallbackT>(callback), group);
}

/// Create a publisher using traits like services. This is a private implementation.
template <class SpecT, class NodeT>
typename Publisher<SpecT>::SharedPtr create_publisher_impl(NodeT * node)
{
  // This function is a wrapper for the following.
  // https://github.com/ros2/rclcpp/blob/48068130edbb43cdd61076dc1851672ff1a80408/rclcpp/include/rclcpp/node.hpp#L167-L205
  auto publisher =
    node->template create_publisher<typename SpecT::Message>(SpecT::name, get_qos<SpecT>());
  return Publisher<SpecT>::make_shared(publisher);
}

/// Create a subscription using traits like services. This is a private implementation.
template <class SpecT, class NodeT, class CallbackT>
typename Subscription<SpecT>::SharedPtr create_subscription_impl(
  NodeT * node, CallbackT && callback)
{
  // This function is a wrapper for the following.
  // https://github.com/ros2/rclcpp/blob/48068130edbb43cdd61076dc1851672ff1a80408/rclcpp/include/rclcpp/node.hpp#L207-L238
  auto subscription = node->template create_subscription<typename SpecT::Message>(
    SpecT::name, get_qos<SpecT>(), std::forward<CallbackT>(callback));
  return Subscription<SpecT>::make_shared(subscription);
}

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_
