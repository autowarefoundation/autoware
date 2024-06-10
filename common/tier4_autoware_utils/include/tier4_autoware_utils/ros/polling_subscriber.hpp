// Copyright 2024 TIER IV, Inc.
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

#ifndef TIER4_AUTOWARE_UTILS__ROS__POLLING_SUBSCRIBER_HPP_
#define TIER4_AUTOWARE_UTILS__ROS__POLLING_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stdexcept>
#include <string>

namespace tier4_autoware_utils
{

template <typename T>
class InterProcessPollingSubscriber
{
public:
  using SharedPtr = std::shared_ptr<InterProcessPollingSubscriber<T>>;
  static SharedPtr create_subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  {
    return std::make_shared<InterProcessPollingSubscriber<T>>(node, topic_name, qos);
  }

private:
  typename rclcpp::Subscription<T>::SharedPtr subscriber_;
  typename T::SharedPtr data_;

public:
  explicit InterProcessPollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  {
    auto noexec_callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto noexec_subscription_options = rclcpp::SubscriptionOptions();
    noexec_subscription_options.callback_group = noexec_callback_group;

    subscriber_ = node->create_subscription<T>(
      topic_name, qos,
      [node]([[maybe_unused]] const typename T::ConstSharedPtr msg) { assert(false); },
      noexec_subscription_options);
    if (qos.get_rmw_qos_profile().depth > 1) {
      throw std::invalid_argument(
        "InterProcessPollingSubscriber will be used with depth > 1, which may cause inefficient "
        "serialization while updateLatestData()");
    }
  };
  typename T::ConstSharedPtr takeData()
  {
    auto new_data = std::make_shared<T>();
    rclcpp::MessageInfo message_info;
    const bool success = subscriber_->take(*new_data, message_info);
    if (success) {
      data_ = new_data;
    }

    return data_;
  };
};

}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__ROS__POLLING_SUBSCRIBER_HPP_
