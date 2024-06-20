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

#ifndef AUTOWARE__UNIVERSE_UTILS__ROS__POLLING_SUBSCRIBER_HPP_
#define AUTOWARE__UNIVERSE_UTILS__ROS__POLLING_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

namespace autoware::universe_utils
{

inline rclcpp::SensorDataQoS SingleDepthSensorQoS()
{
  rclcpp::SensorDataQoS qos;
  qos.get_rmw_qos_profile().depth = 1;
  return qos;
}

template <typename T, int N = 1, typename Enable = void>
class InterProcessPollingSubscriber;

template <typename T, int N>
class InterProcessPollingSubscriber<T, N, typename std::enable_if<N == 1>::type>
{
public:
  using SharedPtr =
    std::shared_ptr<InterProcessPollingSubscriber<T, N, typename std::enable_if<N == 1>::type>>;
  static SharedPtr create_subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  {
    return std::make_shared<InterProcessPollingSubscriber<T, N>>(node, topic_name, qos);
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
  /*
   * @brief take and return the latest data from DDS queue if such data existed, otherwise return
   * previous taken data("stale" data)
   * @note if you want to ignore "stale" data, you should use takeNewData()
   * instead
   */
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

  /*
   * @brief take and return the latest data from DDS queue if such data existed, otherwise return
   * nullptr instead
   * @note this API allows you to avoid redundant computation on the taken data which is unchanged
   * since the previous cycle
   */
  typename T::ConstSharedPtr takeNewData()
  {
    auto new_data = std::make_shared<T>();
    rclcpp::MessageInfo message_info;
    const bool success = subscriber_->take(*new_data, message_info);
    if (success) {
      data_ = new_data;
      return data_;
    } else {
      return nullptr;
    }
  }
};

template <typename T, int N>
class InterProcessPollingSubscriber<T, N, typename std::enable_if<(N >= 2)>::type>
{
public:
  using SharedPtr =
    std::shared_ptr<InterProcessPollingSubscriber<T, N, typename std::enable_if<(N >= 2)>::type>>;
  static SharedPtr create_subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{N})
  {
    return std::make_shared<InterProcessPollingSubscriber<T, N>>(node, topic_name, qos);
  }

private:
  typename rclcpp::Subscription<T>::SharedPtr subscriber_;

public:
  explicit InterProcessPollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{N})
  {
    auto noexec_callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto noexec_subscription_options = rclcpp::SubscriptionOptions();
    noexec_subscription_options.callback_group = noexec_callback_group;

    subscriber_ = node->create_subscription<T>(
      topic_name, qos,
      [node]([[maybe_unused]] const typename T::ConstSharedPtr msg) { assert(false); },
      noexec_subscription_options);
    if (qos.get_rmw_qos_profile().depth < N) {
      throw std::invalid_argument(
        "InterProcessPollingSubscriber will be used with depth == " + std::to_string(N) +
        ", which may cause inefficient serialization while updateLatestData()");
    }
  };
  std::vector<typename T::ConstSharedPtr> takeData()
  {
    std::vector<typename T::ConstSharedPtr> data;
    rclcpp::MessageInfo message_info;
    for (int i = 0; i < N; ++i) {
      auto datum = std::make_shared<T>();
      if (subscriber_->take(*datum, message_info)) {
        data.push_back(datum);
      } else {
        break;
      }
    }
    return data;
  };
};

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__ROS__POLLING_SUBSCRIBER_HPP_
