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

#ifndef DIAGNOSTIC_CONVERTER__CONVERTER_NODE_HPP_
#define DIAGNOSTIC_CONVERTER__CONVERTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "tier4_simulation_msgs/msg/user_defined_value.hpp"
#include "tier4_simulation_msgs/msg/user_defined_value_type.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace diagnostic_converter
{
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using tier4_simulation_msgs::msg::UserDefinedValue;
using tier4_simulation_msgs::msg::UserDefinedValueType;

/**
 * @brief Node for converting from DiagnosticArray to UserDefinedValue
 */
class DiagnosticConverter : public rclcpp::Node
{
public:
  explicit DiagnosticConverter(const rclcpp::NodeOptions & node_options);

  /**
   * @brief callback for DiagnosticArray msgs that publishes equivalent UserDefinedValue msgs
   * @param [in] diag_msg received diagnostic message
   */
  void onDiagnostic(
    const DiagnosticArray::ConstSharedPtr diag_msg, const size_t diag_idx,
    const std::string & topic);

  UserDefinedValue createUserDefinedValue(const KeyValue & key_value) const;

  rclcpp::Publisher<UserDefinedValue>::SharedPtr getPublisher(
    const std::string & topic, const size_t pub_idx);

private:
  // ROS
  std::vector<rclcpp::Subscription<DiagnosticArray>::SharedPtr> diagnostics_sub_;
  std::vector<std::unordered_map<std::string, rclcpp::Publisher<UserDefinedValue>::SharedPtr>>
    params_pub_;
};
}  // namespace diagnostic_converter

#endif  // DIAGNOSTIC_CONVERTER__CONVERTER_NODE_HPP_
