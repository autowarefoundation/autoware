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

#ifndef TOPIC_STATE_MONITOR__TOPIC_STATE_MONITOR_HPP_
#define TOPIC_STATE_MONITOR__TOPIC_STATE_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <string>

namespace topic_state_monitor
{
struct Param
{
  double warn_rate;
  double error_rate;
  double timeout;
  int window_size;
};

enum class TopicStatus : int8_t {
  Ok,
  NotReceived,
  WarnRate,
  ErrorRate,
  Timeout,
};

class TopicStateMonitor
{
public:
  explicit TopicStateMonitor(rclcpp::Node & node);

  void setParam(const Param & param) { param_ = param; }

  rclcpp::Time getLastMessageTime() const { return last_message_time_; }
  double getTopicRate() const { return topic_rate_; }

  void update();
  TopicStatus getTopicStatus() const;

private:
  Param param_;

  static constexpr double max_rate = 100000.0;

  std::deque<rclcpp::Time> time_buffer_;
  rclcpp::Time last_message_time_ = rclcpp::Time(0);
  double topic_rate_ = TopicStateMonitor::max_rate;

  rclcpp::Clock::SharedPtr clock_;

  double calcTopicRate() const;
  bool isNotReceived() const;
  bool isWarnRate() const;
  bool isErrorRate() const;
  bool isTimeout() const;
};
}  // namespace topic_state_monitor

#endif  // TOPIC_STATE_MONITOR__TOPIC_STATE_MONITOR_HPP_
