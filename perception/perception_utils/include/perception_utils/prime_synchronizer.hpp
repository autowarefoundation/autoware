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
#ifndef PERCEPTION_UTILS__PRIME_SYNCHRONIZER_HPP_
#define PERCEPTION_UTILS__PRIME_SYNCHRONIZER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace perception_utils
{

/**
 * @brief  This class implemented a multi-topic approximate time synchronizer.
 * Different from message_filters::sync::ApproximateTime, this class assumes there's a primary topic
 * and more than one secondary topics and the registered callback function will always been called
 * for every primary topic message, while for message_filters::sync::ApproximateTime, the callback
 * function will be called only when all topics are synchronized.
 *
 * For every primary topic message M1, when this class receives it, the class would check whether it
 * could gather one message of every secondary topic whose timestamp is similar to that of M1. If
 * yes, the registered callback function would be called at once with M1 and the gathered messages.
 * Otherwise, it would wait for some time. After that, the class would collect secondary messages
 * again. For secondary topics that can't find any message with similar timestamp to M1, nullptr
 * would be used instead and the registered function would be called.
 *
 * This class assumes the message types of the primary topic and secondary topics are defined during
 * compile-time. It should be noted that the registered callback function is too slow, it would
 * block the thread and the primary message couldn't be received.
 *
 * @tparam PrimeMsgT      Primary topic message type
 * @tparam SecondaryMsgT  Secondary topic message types
 */
template <typename PrimeMsgT, typename... SecondaryMsgT>
class PrimeSynchronizer
{
  typedef double StampT;

public:
  /**
   * @brief Construct a new Prime Synchronizer object
   *
   * @param node_ptr      node handler pointer
   * @param topics        topic vector. topics[0] corresponds to primary topic and the others
   * correspond to secondary topics
   * @param qos           qos vector. configured for every topic
   * @param callback      registered callback function, would be called when the messages are
   * synchronized
   * @param max_delay_t   maximum timestamp different (seconds) between primary topic message and
   * any other secondary topic message
   * @param max_wait_t    maximum wait time (seconds) before the messages are synchronized
   */
  PrimeSynchronizer(
    rclcpp::Node * node_ptr, const std::vector<std::string> & topics,
    const std::vector<rclcpp::QoS> & qos,
    std::function<void(
      const typename PrimeMsgT::ConstSharedPtr, const typename SecondaryMsgT::ConstSharedPtr...)>
      callback,
    StampT max_delay_t = 0.2, StampT max_wait_t = 0.1)
  : node_ptr_(node_ptr), callback_(callback), max_wait_t_(max_wait_t), max_delay_t_(max_delay_t)
  {
    assert((topics.size() == sizeof...(SecondaryMsgT) + 1) && "Incorrect topic number");
    assert(topics.size() == qos.size() && "topic size not equal to qos size!");
    prime_subscriber_ = node_ptr_->create_subscription<PrimeMsgT>(
      topics[0], qos[0], std::bind(&PrimeSynchronizer::primeCallback, this, std::placeholders::_1));
    initSecondaryListener(
      std::vector<std::string>(topics.begin() + 1, topics.end()),
      std::vector<rclcpp::QoS>(qos.begin() + 1, qos.end()));
    std::chrono::nanoseconds wait_duration(static_cast<int>(1e9 * max_wait_t));
    timer_ = rclcpp::create_timer(
      node_ptr_, node_ptr_->get_clock(), wait_duration,
      std::bind(&PrimeSynchronizer::timerCallback, this));
    timer_->cancel();
  }

private:
  /**
   * @brief initialize the secondary topic subscribers.
   * Have to use template recursion method to indexing the subscriber tuple
   *
   * @tparam Idx     secondary subscriber tuple index
   * @param topics   topics vector
   * @param qos      qos vector
   */
  template <std::size_t Idx = 0>
  void initSecondaryListener(
    const std::vector<std::string> & topics, const std::vector<rclcpp::QoS> & qos)
  {
    if constexpr (Idx < sizeof...(SecondaryMsgT)) {
      typedef std::tuple_element_t<Idx, std::tuple<SecondaryMsgT...>> type;
      std::get<Idx>(sec_subscriber_) = node_ptr_->create_subscription<type>(
        topics[Idx], qos[Idx],
        std::bind(&PrimeSynchronizer::secondaryCallback<type, Idx>, this, std::placeholders::_1));
      initSecondaryListener<Idx + 1>(topics, qos);
    }
  }

  /**
   * @brief Collect the Idx-th secondary messages with similar timestamp to prime message,
   * which is stored in argv[0] and write to argv[Idx + 1]
   *
   * @tparam Idx  secondary subscriber tuple index
   * @param argv  output tuple
   */
  template <std::size_t Idx = 0>
  void collectSecondaryMsg(
    std::tuple<typename PrimeMsgT::ConstSharedPtr, typename SecondaryMsgT::ConstSharedPtr...> &
      argv)
  {
    if constexpr (Idx < sizeof...(SecondaryMsgT)) {
      /*
      if can't find any message for this topic, write nullptr
      */
      if (std::get<Idx>(sec_messages_).empty()) {
        std::get<Idx + 1>(argv) = nullptr;
      } else {
        StampT prime_stamp = convertStampFormat(std::get<0>(argv)->header.stamp);
        StampT min_delay = std::numeric_limits<StampT>::max();
        auto best_sec_msg = std::get<Idx>(sec_messages_).begin()->second;
        /*
        find the message with closest timestamp to primary message
        */
        for (const auto & sec_msg_p : std::get<Idx>(sec_messages_)) {
          StampT delay = std::abs(prime_stamp - sec_msg_p.first);
          if (delay < min_delay) {
            min_delay = delay;
            best_sec_msg = sec_msg_p.second;
          }
        }
        std::get<Idx + 1>(argv) = min_delay < max_delay_t_ ? best_sec_msg : nullptr;
      }
      collectSecondaryMsg<Idx + 1>(argv);
    }
  }

  /**
   * @brief check if all messages in argv are valid (not equal to nullptr)
   *
   * @tparam Idx
   * @param argv
   * @return true  All messages are not nullptr
   * @return false At least one message in the topic is nullptr
   */
  template <std::size_t Idx = 0>
  bool isArgvValid(
    const std::tuple<
      typename PrimeMsgT::ConstSharedPtr, typename SecondaryMsgT::ConstSharedPtr...> & argv)
  {
    if constexpr (Idx < sizeof...(SecondaryMsgT) + 1) {
      return (std::get<Idx>(argv) != nullptr) && isArgvValid<Idx + 1>(argv);
    }
    return true;
  }

  inline StampT convertStampFormat(const std_msgs::msg::Header::_stamp_type & stamp)
  {
    return rclcpp::Time(stamp).seconds();
  }

  /**
   * @brief callback function when primary message is received
   *
   * @param msg
   */
  void primeCallback(const typename PrimeMsgT::ConstSharedPtr msg)
  {
    prime_cnt++;
    timer_->cancel();
    assert(prime_messages_.size() <= 1);
    /*
    if there are old prime messages waiting for synchronization with secondary messages,
    stop waiting and call the registered callback function with prime message and synchronized
    secondary messages. For secondary topics that are not synchronized, use nullptr.
    */
    for (auto & p : prime_messages_) {
      tryCallback(p.first, true);
    }
    prime_messages_.clear();
    /*
    update the prime messages
    */
    StampT stamp = convertStampFormat(msg->header.stamp);
    prime_messages_.insert(std::make_pair(stamp, msg));
    /*
    check if secondary messages are all ready to synchronize with prime message.
    If yes, process it immediately
    */
    if (tryCallback(stamp, false)) {
      prime_messages_.clear();
    } else {
      timer_->reset();
    }
    // RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "primary message count = " << prime_cnt);
  }

  /**
   * @brief callback function when secondary message is received.
   *
   * @tparam M    Type of the secondary message
   * @tparam Idx  Idx of the type
   * @param msg
   */
  template <typename M, std::size_t Idx>
  void secondaryCallback(const typename M::ConstSharedPtr msg)
  {
    /*
    insert the new secondary message
    */
    StampT stamp = convertStampFormat(msg->header.stamp);
    auto & msg_map = std::get<Idx>(sec_messages_);
    msg_map.insert(std::make_pair(stamp, msg));
    assert(prime_messages_.size() <= 1);
    /*
    check if any prime message can gather all secondary messages.
    */
    for (auto it = prime_messages_.begin(); it != prime_messages_.end();) {
      /*
      Try to synchronize. If succeeds, remove the handled primary message
      */
      if (tryCallback(it->first, false)) {
        timer_->cancel();
        it = prime_messages_.erase(it);
      } else {
        it++;
      }
    }

    /*
    this should not happen. Just in case.
    */
    if (prime_messages_.empty() && msg_map.empty()) {
      RCLCPP_ERROR_STREAM(node_ptr_->get_logger(), "Empty prime_messages and secondary_messages");
      return;
    }
    /*
    remove old secondary messages.
    */
    StampT stamp_thres =
      prime_messages_.empty() ? msg_map.rbegin()->first : prime_messages_.begin()->first;
    for (auto it = msg_map.begin(); it != msg_map.end();) {
      if (stamp_thres - it->first > max_delay_t_) {
        it = msg_map.erase(it);
      } else {
        it++;
      }
    }
  }

  /**
   * @brief Timer callback. The maximum wait time exceeds. Handle all stored primary messages.
   *
   */
  void timerCallback()
  {
    timer_->cancel();
    assert(prime_messages_.size() <= 1);
    for (auto & p : prime_messages_) {
      tryCallback(p.first, true);
    }
    prime_messages_.clear();
  }

  /**
   * @brief Try to synchronize. If ignoreInvalidSecMsg is set to true,
   * the registered function would be called with the primary message with given timestamp and
   * collected secondary messages even if not all secondary messages are collected. Otherwise, the
   * registered function would not be called except all secondary messages are collected.
   *
   * @param stamp
   * @param ignoreInvalidSecMsg
   * @return true
   * @return false
   */
  bool tryCallback(StampT stamp, bool ignoreInvalidSecMsg = true)
  {
    if (prime_messages_.count(stamp) == 0) {
      return true;
    }
    std::tuple<typename PrimeMsgT::ConstSharedPtr, typename SecondaryMsgT::ConstSharedPtr...> argv;
    std::get<0>(argv) = prime_messages_[stamp];
    collectSecondaryMsg(argv);
    if (ignoreInvalidSecMsg || isArgvValid(argv)) {
      std::apply(callback_, argv);
      return true;
    }
    return false;
  }
  /**
   * @brief node pointer
   *
   */
  rclcpp::Node * node_ptr_;
  /**
   * @brief The registered callback function that would be called when the prime message and sub
   * messages are synchronized or timeout
   *
   */
  std::function<void(
    const typename PrimeMsgT::ConstSharedPtr, const typename SecondaryMsgT::ConstSharedPtr...)>
    callback_;
  /**
   * @brief the prime message subscriber
   *
   */
  typename rclcpp::Subscription<PrimeMsgT>::SharedPtr prime_subscriber_;
  /**
   * @brief the secondary message subscriber tuple
   *
   */
  std::tuple<typename rclcpp::Subscription<SecondaryMsgT>::SharedPtr...> sec_subscriber_;
  /**
   * @brief map to store the prime messages using timestamp of the messages as key.
   *
   */
  std::map<StampT, typename PrimeMsgT::ConstSharedPtr> prime_messages_;
  rclcpp::TimerBase::SharedPtr timer_;
  /**
   * @brief tuple of maps to store the secondary messages using timestamp of the messages as key
   *
   */
  std::tuple<typename std::map<StampT, typename SecondaryMsgT::ConstSharedPtr>...> sec_messages_;
  /*
  maximum wait time (seconds) before the secondary messages are collected
  */
  double max_wait_t_;
  /*
  maximum delay time (seconds) between a secondary message and the primary message
  */
  double max_delay_t_;
  int prime_cnt = 0;
};

}  // namespace perception_utils

#endif  // PERCEPTION_UTILS__PRIME_SYNCHRONIZER_HPP_
