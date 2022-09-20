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

#include "topic_state_monitor/topic_state_monitor_core.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
template <typename T>
void update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  auto it = std::find_if(
    parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
  if (it != parameters.cend()) {
    value = it->template get_value<T>();
  }
}
}  // namespace

namespace topic_state_monitor
{
TopicStateMonitorNode::TopicStateMonitorNode(const rclcpp::NodeOptions & node_options)
: Node("topic_state_monitor", node_options), updater_(this)
{
  using std::placeholders::_1;

  // Parameter
  node_param_.update_rate = declare_parameter("update_rate", 10.0);
  node_param_.topic = declare_parameter<std::string>("topic");
  node_param_.transient_local = declare_parameter("transient_local", false);
  node_param_.best_effort = declare_parameter("best_effort", false);
  node_param_.diag_name = declare_parameter<std::string>("diag_name");
  node_param_.is_transform = (node_param_.topic == "/tf" || node_param_.topic == "/tf_static");

  if (node_param_.is_transform) {
    node_param_.frame_id = declare_parameter<std::string>("frame_id");
    node_param_.child_frame_id = declare_parameter<std::string>("child_frame_id");
  } else {
    node_param_.topic_type = declare_parameter<std::string>("topic_type");
  }

  param_.warn_rate = declare_parameter("warn_rate", 0.5);
  param_.error_rate = declare_parameter("error_rate", 0.1);
  param_.timeout = declare_parameter("timeout", 1.0);
  param_.window_size = declare_parameter("window_size", 10);

  // Parameter Reconfigure
  set_param_res_ =
    this->add_on_set_parameters_callback(std::bind(&TopicStateMonitorNode::onParameter, this, _1));

  // Core
  topic_state_monitor_ = std::make_unique<TopicStateMonitor>(*this);
  topic_state_monitor_->setParam(param_);

  // Subscriber
  rclcpp::QoS qos = rclcpp::QoS{1};
  if (node_param_.transient_local) {
    qos.transient_local();
  }
  if (node_param_.best_effort) {
    qos.best_effort();
  }

  if (node_param_.is_transform) {
    sub_transform_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      node_param_.topic, qos, [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
        for (const auto & transform : msg->transforms) {
          if (
            transform.header.frame_id == node_param_.frame_id &&
            transform.child_frame_id == node_param_.child_frame_id) {
            topic_state_monitor_->update();
          }
        }
      });
  } else {
    sub_topic_ = this->create_generic_subscription(
      node_param_.topic, node_param_.topic_type, qos,
      [this]([[maybe_unused]] std::shared_ptr<rclcpp::SerializedMessage> msg) {
        topic_state_monitor_->update();
      });
  }

  // Diagnostic Updater
  updater_.setHardwareID("topic_state_monitor");
  updater_.add(node_param_.diag_name, this, &TopicStateMonitorNode::checkTopicStatus);

  // Timer
  const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&TopicStateMonitorNode::onTimer, this));
}

rcl_interfaces::msg::SetParametersResult TopicStateMonitorNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    update_param(parameters, "warn_rate", param_.warn_rate);
    update_param(parameters, "error_rate", param_.error_rate);
    update_param(parameters, "timeout", param_.timeout);
    update_param(parameters, "window_size", param_.window_size);
    topic_state_monitor_->setParam(param_);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

void TopicStateMonitorNode::onTimer()
{
  // Publish diagnostics
  updater_.force_update();
}

void TopicStateMonitorNode::checkTopicStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  // Get information
  const auto topic_status = topic_state_monitor_->getTopicStatus();
  const auto last_message_time = topic_state_monitor_->getLastMessageTime();
  const auto topic_rate = topic_state_monitor_->getTopicRate();

  // Add topic name
  if (node_param_.is_transform) {
    const auto frame = "(" + node_param_.frame_id + " to " + node_param_.child_frame_id + ")";
    stat.addf("topic", "%s %s", node_param_.topic.c_str(), frame.c_str());
  } else {
    stat.addf("topic", "%s", node_param_.topic.c_str());
  }

  // Judge level
  int8_t level = DiagnosticStatus::OK;
  if (topic_status == TopicStatus::Ok) {
    level = DiagnosticStatus::OK;
    stat.add("status", "OK");
  } else if (topic_status == TopicStatus::NotReceived) {
    level = DiagnosticStatus::ERROR;
    stat.add("status", "NotReceived");
  } else if (topic_status == TopicStatus::WarnRate) {
    level = DiagnosticStatus::WARN;
    stat.add("status", "WarnRate");
  } else if (topic_status == TopicStatus::ErrorRate) {
    level = DiagnosticStatus::ERROR;
    stat.add("status", "ErrorRate");
  } else if (topic_status == TopicStatus::Timeout) {
    level = DiagnosticStatus::ERROR;
    stat.add("status", "Timeout");
  }

  // Add key-value
  stat.addf("warn_rate", "%.2f [Hz]", param_.warn_rate);
  stat.addf("error_rate", "%.2f [Hz]", param_.error_rate);
  stat.addf("timeout", "%.2f [s]", param_.timeout);
  stat.addf("measured_rate", "%.2f [Hz]", topic_rate);
  stat.addf("now", "%.2f [s]", this->now().seconds());
  stat.addf("last_message_time", "%.2f [s]", last_message_time.seconds());

  // Create message
  std::string msg;
  if (level == DiagnosticStatus::OK) {
    msg = "OK";
  } else if (level == DiagnosticStatus::WARN) {
    msg = "Warn";
  } else if (level == DiagnosticStatus::ERROR) {
    msg = "Error";
  }

  // Add summary
  stat.summary(level, msg);
}

}  // namespace topic_state_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(topic_state_monitor::TopicStateMonitorNode)
