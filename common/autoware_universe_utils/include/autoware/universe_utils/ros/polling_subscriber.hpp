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
#include <vector>

namespace autoware::universe_utils
{

/**
 * @brief Creates a SensorDataQoS profile with a single depth.
 * @return rclcpp::SensorDataQoS The QoS profile with depth set to 1.
 */
inline rclcpp::SensorDataQoS SingleDepthSensorQoS()
{
  rclcpp::SensorDataQoS qos;
  qos.get_rmw_qos_profile().depth = 1;
  return qos;
}

namespace polling_policy
{

/**
 * @brief Polling policy that keeps the latest received message.
 *
 * This policy retains the latest received message and provides it when requested. If a new message
 * is received, it overwrites the previously stored message.
 *
 * @tparam MessageT The message type.
 */
template <typename MessageT>
class Latest
{
private:
  typename MessageT::ConstSharedPtr data_{nullptr};  ///< Data pointer to store the latest data

protected:
  /**
   * @brief Check the QoS settings for the subscription.
   *
   * @param qos The QoS profile to check.
   * @throws std::invalid_argument If the QoS depth is not 1.
   */
  void checkQoS(const rclcpp::QoS & qos)
  {
    if (qos.get_rmw_qos_profile().depth > 1) {
      throw std::invalid_argument(
        "InterProcessPollingSubscriber will be used with depth > 1, which may cause inefficient "
        "serialization while updateLatestData()");
    }
  }

public:
  /**
   * @brief Retrieve the latest data. If no new data has been received, the previously received data
   *
   * @return typename MessageT::ConstSharedPtr The latest data.
   */
  typename MessageT::ConstSharedPtr takeData();
};

/**
 * @brief Polling policy that keeps the newest received message.
 *
 * @tparam MessageT The message type.
 */
template <typename MessageT>
class Newest
{
protected:
  /**
   * @brief Check the QoS settings for the subscription.
   *
   * @param qos The QoS profile to check.
   * @throws std::invalid_argument If the QoS depth is not 1.
   */
  void checkQoS(const rclcpp::QoS & qos)
  {
    if (qos.get_rmw_qos_profile().depth > 1) {
      throw std::invalid_argument(
        "InterProcessPollingSubscriber will be used with depth > 1, which may cause inefficient "
        "serialization while updateLatestData()");
    }
  }

public:
  /**
   * @brief Retrieve the newest data. If no new data has been received, nullptr is returned.
   *
   * @return typename MessageT::ConstSharedPtr The newest data.
   */
  typename MessageT::ConstSharedPtr takeData();
};

/**
 * @brief Polling policy that keeps all received messages.
 *
 * @tparam MessageT The message type.
 */
template <typename MessageT>
class All
{
protected:
  /**
   * @brief Check the QoS settings for the subscription.
   *
   * @param qos The QoS profile to check.
   */
  void checkQoS(const rclcpp::QoS &) {}

public:
  /**
   * @brief Retrieve all data.
   *
   * @return std::vector<typename MessageT::ConstSharedPtr> The list of all received data.
   */
  std::vector<typename MessageT::ConstSharedPtr> takeData();
};

}  // namespace polling_policy

/**
 * @brief Subscriber class that uses a specified polling policy.
 *
 * @tparam MessageT The message type.
 * @tparam PollingPolicy The polling policy to use.
 */
template <typename MessageT, template <typename> class PollingPolicy = polling_policy::Latest>
class InterProcessPollingSubscriber : public PollingPolicy<MessageT>
{
  friend PollingPolicy<MessageT>;

private:
  typename rclcpp::Subscription<MessageT>::SharedPtr subscriber_;  ///< Subscription object

public:
  using SharedPtr = std::shared_ptr<InterProcessPollingSubscriber<MessageT, PollingPolicy>>;

  /**
   * @brief Construct a new InterProcessPollingSubscriber object.
   *
   * @param node The node to attach the subscriber to.
   * @param topic_name The topic name to subscribe to.
   * @param qos The QoS profile to use for the subscription.
   */
  explicit InterProcessPollingSubscriber(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  {
    this->checkQoS(qos);

    auto noexec_callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

    auto noexec_subscription_options = rclcpp::SubscriptionOptions();
    noexec_subscription_options.callback_group = noexec_callback_group;

    subscriber_ = node->create_subscription<MessageT>(
      topic_name, qos,
      [node]([[maybe_unused]] const typename MessageT::ConstSharedPtr msg) { assert(false); },
      noexec_subscription_options);
  }

  /**
   * @brief Create a subscription.
   *
   * @param node The node to attach the subscriber to.
   * @param topic_name The topic name to subscribe to.
   * @param qos The QoS profile to use for the subscription.
   * @return SharedPtr The created subscription.
   */
  static SharedPtr create_subscription(
    rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos = rclcpp::QoS{1})
  {
    return std::make_shared<InterProcessPollingSubscriber<MessageT, PollingPolicy>>(
      node, topic_name, qos);
  }

  typename rclcpp::Subscription<MessageT>::SharedPtr subscriber() { return subscriber_; }
};

namespace polling_policy
{

template <typename MessageT>
typename MessageT::ConstSharedPtr Latest<MessageT>::takeData()
{
  auto & subscriber =
    static_cast<InterProcessPollingSubscriber<MessageT, Latest> *>(this)->subscriber_;
  auto new_data = std::make_shared<MessageT>();
  rclcpp::MessageInfo message_info;
  const bool success = subscriber->take(*new_data, message_info);
  if (success) {
    data_ = new_data;
  }

  return data_;
}

template <typename MessageT>
typename MessageT::ConstSharedPtr Newest<MessageT>::takeData()
{
  auto & subscriber =
    static_cast<InterProcessPollingSubscriber<MessageT, Newest> *>(this)->subscriber_;
  auto new_data = std::make_shared<MessageT>();
  rclcpp::MessageInfo message_info;
  const bool success = subscriber->take(*new_data, message_info);
  if (success) {
    return new_data;
  }
  return nullptr;
}

template <typename MessageT>
std::vector<typename MessageT::ConstSharedPtr> All<MessageT>::takeData()
{
  auto & subscriber =
    static_cast<InterProcessPollingSubscriber<MessageT, All> *>(this)->subscriber_;
  std::vector<typename MessageT::ConstSharedPtr> data;
  rclcpp::MessageInfo message_info;
  for (;;) {
    auto datum = std::make_shared<MessageT>();
    if (subscriber->take(*datum, message_info)) {
      data.push_back(datum);
    } else {
      break;
    }
  }
  return data;
}

}  // namespace polling_policy

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__ROS__POLLING_SUBSCRIBER_HPP_
