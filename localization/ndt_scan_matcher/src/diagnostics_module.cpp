// Copyright 2023 Autoware Foundation
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

#include "ndt_scan_matcher/diagnostics_module.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <algorithm>
#include <string>

DiagnosticsModule::DiagnosticsModule(rclcpp::Node * node, const std::string & diagnostic_name)
: clock_(node->get_clock())
{
  diagnostics_pub_ =
    node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  diagnostics_status_msg_.name =
    std::string(node->get_name()) + std::string(": ") + diagnostic_name;
  diagnostics_status_msg_.hardware_id = node->get_name();
}

void DiagnosticsModule::clear()
{
  diagnostics_status_msg_.values.clear();
  diagnostics_status_msg_.values.shrink_to_fit();

  diagnostics_status_msg_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diagnostics_status_msg_.message = "";
}

void DiagnosticsModule::addKeyValue(const diagnostic_msgs::msg::KeyValue & key_value_msg)
{
  auto it = std::find_if(
    std::begin(diagnostics_status_msg_.values), std::end(diagnostics_status_msg_.values),
    [key_value_msg](const auto & arg) { return arg.key == key_value_msg.key; });

  if (it != std::cend(diagnostics_status_msg_.values)) {
    it->value = key_value_msg.value;
  } else {
    diagnostics_status_msg_.values.push_back(key_value_msg);
  }
}

template <>
void DiagnosticsModule::addKeyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = key;
  key_value.value = value;
  addKeyValue(key_value);
}

template <>
void DiagnosticsModule::addKeyValue(const std::string & key, const bool & value)
{
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = key;
  key_value.value = value ? "True" : "False";
  addKeyValue(key_value);
}

void DiagnosticsModule::updateLevelAndMessage(const int8_t level, const std::string & message)
{
  if ((level > diagnostic_msgs::msg::DiagnosticStatus::OK)) {
    if (!diagnostics_status_msg_.message.empty()) {
      diagnostics_status_msg_.message += "; ";
    }
    diagnostics_status_msg_.message += message;
  }
  if (level > diagnostics_status_msg_.level) {
    diagnostics_status_msg_.level = level;
  }
}

void DiagnosticsModule::publish()
{
  diagnostics_pub_->publish(createDiagnosticsArray());
}

diagnostic_msgs::msg::DiagnosticArray DiagnosticsModule::createDiagnosticsArray() const
{
  diagnostic_msgs::msg::DiagnosticArray diagnostics_msg;
  diagnostics_msg.header.stamp = clock_->now();
  diagnostics_msg.status.push_back(diagnostics_status_msg_);

  if (diagnostics_msg.status.at(0).level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    diagnostics_msg.status.at(0).message = "OK";
  }

  return diagnostics_msg;
}
