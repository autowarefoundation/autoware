// Copyright 2024 Tier IV, Inc.
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

#ifndef AUTOWARE__EVALUATOR_UTILS__EVALUATOR_UTILS_HPP_
#define AUTOWARE__EVALUATOR_UTILS__EVALUATOR_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <deque>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::evaluator_utils
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using DiagnosticQueue = std::deque<std::pair<DiagnosticStatus, rclcpp::Time>>;

void removeOldDiagnostics(const rclcpp::Time & stamp, DiagnosticQueue & diag_queue);
void removeDiagnosticsByName(const std::string & name, DiagnosticQueue & diag_queue);
void addDiagnostic(
  const DiagnosticStatus & diag, const rclcpp::Time & stamp, DiagnosticQueue & diag_queue);
void updateDiagnosticQueue(
  const DiagnosticArray & input_diagnostics, const std::string & function,
  const rclcpp::Time & stamp, DiagnosticQueue & diag_queue);

}  // namespace autoware::evaluator_utils

#endif  // AUTOWARE__EVALUATOR_UTILS__EVALUATOR_UTILS_HPP_
