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

#include "topic_state_monitor/topic_state_monitor.hpp"

namespace topic_state_monitor
{
TopicStateMonitor::TopicStateMonitor(rclcpp::Node & node) : clock_(node.get_clock()) {}

void TopicStateMonitor::update()
{
  // Add data
  last_message_time_ = clock_->now();
  time_buffer_.push_back(last_message_time_);

  // Remove old data
  while (static_cast<int>(time_buffer_.size()) > param_.window_size) {
    time_buffer_.pop_front();
  }

  // Calc topic rate
  topic_rate_ = calcTopicRate();
}

TopicStatus TopicStateMonitor::getTopicStatus() const
{
  if (isNotReceived()) {
    return TopicStatus::NotReceived;
  }
  if (isTimeout()) {
    return TopicStatus::Timeout;
  }
  if (isErrorRate()) {
    return TopicStatus::ErrorRate;
  }
  if (isWarnRate()) {
    return TopicStatus::WarnRate;
  }
  return TopicStatus::Ok;
}

double TopicStateMonitor::calcTopicRate() const
{
  // Output max_rate when topic rate can't be calculated.
  // In this case, it's assumed timeout is used instead.
  if (time_buffer_.size() < 2) {
    return TopicStateMonitor::max_rate;
  }

  const auto time_diff = (time_buffer_.back() - time_buffer_.front()).seconds();
  const auto num_intervals = time_buffer_.size() - 1;

  return static_cast<double>(num_intervals) / time_diff;
}

bool TopicStateMonitor::isNotReceived() const { return time_buffer_.empty(); }

bool TopicStateMonitor::isWarnRate() const
{
  if (param_.warn_rate == 0.0) {
    return false;
  }

  return getTopicRate() < param_.warn_rate;
}

bool TopicStateMonitor::isErrorRate() const
{
  if (param_.error_rate == 0.0) {
    return false;
  }

  return getTopicRate() < param_.error_rate;
}

bool TopicStateMonitor::isTimeout() const
{
  if (param_.timeout == 0.0) {
    return false;
  }

  const auto time_diff = (clock_->now() - time_buffer_.back()).seconds();

  return time_diff > param_.timeout;
}
}  // namespace topic_state_monitor
