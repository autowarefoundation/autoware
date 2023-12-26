// Copyright 2023 Tier IV, Inc.
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

#include "diagnostic_converter/converter_node.hpp"

#include <regex>

namespace
{
std::string removeInvalidTopicString(const std::string & input_string)
{
  std::regex pattern{R"([a-zA-Z0-9/_]+)"};

  std::string result;
  for (std::sregex_iterator itr(std::begin(input_string), std::end(input_string), pattern), end;
       itr != end; ++itr) {
    result += itr->str();
  }
  return result;
}

std::string removeUnitString(const std::string & input_string)
{
  for (size_t i = 0; i < input_string.size(); ++i) {
    if (input_string.at(i) == '[') {
      if (i != 0 && input_string.at(i - 1) == ' ') {
        // Blank is also removed
        return std::string{input_string.begin(), input_string.begin() + i - 1};
      }
      return std::string{input_string.begin(), input_string.begin() + i};
    }
  }
  return input_string;
}
}  // namespace

namespace diagnostic_converter
{
DiagnosticConverter::DiagnosticConverter(const rclcpp::NodeOptions & node_options)
: Node("diagnostic_converter", node_options)
{
  using std::placeholders::_1;

  size_t sub_counter = 0;
  std::vector<std::string> diagnostic_topics;
  declare_parameter<std::vector<std::string>>("diagnostic_topics", std::vector<std::string>());
  get_parameter<std::vector<std::string>>("diagnostic_topics", diagnostic_topics);
  for (const std::string & diagnostic_topic : diagnostic_topics) {
    // std::function required with multiple arguments https://answers.ros.org/question/289207
    const std::function<void(const DiagnosticArray::ConstSharedPtr)> fn =
      std::bind(&DiagnosticConverter::onDiagnostic, this, _1, sub_counter++, diagnostic_topic);
    diagnostics_sub_.push_back(create_subscription<DiagnosticArray>(diagnostic_topic, 1, fn));
  }
  params_pub_.resize(diagnostics_sub_.size());
}

void DiagnosticConverter::onDiagnostic(
  const DiagnosticArray::ConstSharedPtr diag_msg, const size_t diag_idx,
  const std::string & base_topic)
{
  for (const auto & status : diag_msg->status) {
    std::string status_topic = base_topic + (status.name.empty() ? "" : "_" + status.name);
    for (const auto & key_value : status.values) {
      const auto valid_topic_name = removeInvalidTopicString(status_topic + "_" + key_value.key);
      getPublisher(valid_topic_name, diag_idx)->publish(createUserDefinedValue(key_value));
    }
  }
}

UserDefinedValue DiagnosticConverter::createUserDefinedValue(const KeyValue & key_value) const
{
  UserDefinedValue param_msg;
  param_msg.type.data = UserDefinedValueType::DOUBLE;
  param_msg.value = removeUnitString(key_value.value);
  return param_msg;
}

rclcpp::Publisher<UserDefinedValue>::SharedPtr DiagnosticConverter::getPublisher(
  const std::string & topic, const size_t pub_idx)
{
  auto & pubs = params_pub_[pub_idx];
  if (pubs.count(topic) == 0) {
    pubs[topic] = create_publisher<UserDefinedValue>(topic, 1);
  }
  return pubs.at(topic);
}
}  // namespace diagnostic_converter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(diagnostic_converter::DiagnosticConverter)
