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

#ifndef TOPIC_STATE_MONITOR__TOPIC_STATE_MONITOR_CORE_HPP_
#define TOPIC_STATE_MONITOR__TOPIC_STATE_MONITOR_CORE_HPP_

#include "topic_state_monitor/topic_state_monitor.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_msgs/msg/tf_message.hpp>

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace topic_state_monitor
{
struct NodeParam
{
  double update_rate;
  std::string diag_name;
  std::string topic;
  std::string topic_type;
  std::string frame_id;
  std::string child_frame_id;
  bool transient_local;
  bool best_effort;
  bool is_transform;
};

class TopicStateMonitorNode : public rclcpp::Node
{
public:
  explicit TopicStateMonitorNode(const rclcpp::NodeOptions & node_options);

private:
  // Parameter
  NodeParam node_param_;
  Param param_;

  // Parameter Reconfigure
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  // Core
  std::unique_ptr<TopicStateMonitor> topic_state_monitor_;

  // Subscriber
  rclcpp::GenericSubscription::SharedPtr sub_topic_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_transform_;

  // Timer
  void onTimer();
  rclcpp::TimerBase::SharedPtr timer_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;

  void checkTopicStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
};
}  // namespace topic_state_monitor

#endif  // TOPIC_STATE_MONITOR__TOPIC_STATE_MONITOR_CORE_HPP_
