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

#include "autoware/evaluator_utils/evaluator_utils.hpp"

#include <algorithm>

namespace autoware::evaluator_utils
{
void removeOldDiagnostics(const rclcpp::Time & stamp, DiagnosticQueue & diag_queue)
{
  constexpr double KEEP_TIME = 1.0;
  diag_queue.erase(
    std::remove_if(
      diag_queue.begin(), diag_queue.end(),
      [stamp](const std::pair<DiagnosticStatus, rclcpp::Time> & p) {
        return (stamp - p.second).seconds() > KEEP_TIME;
      }),
    diag_queue.end());
}

void removeDiagnosticsByName(const std::string & name, DiagnosticQueue & diag_queue)
{
  diag_queue.erase(
    std::remove_if(
      diag_queue.begin(), diag_queue.end(),
      [&name](const std::pair<DiagnosticStatus, rclcpp::Time> & p) {
        return p.first.name.find(name) != std::string::npos;
      }),
    diag_queue.end());
}

void addDiagnostic(
  const DiagnosticStatus & diag, const rclcpp::Time & stamp, DiagnosticQueue & diag_queue)
{
  diag_queue.push_back(std::make_pair(diag, stamp));
}

void updateDiagnosticQueue(
  const DiagnosticArray & input_diagnostics, const std::string & function,
  const rclcpp::Time & stamp, DiagnosticQueue & diag_queue)
{
  const auto it = std::find_if(
    input_diagnostics.status.begin(), input_diagnostics.status.end(),
    [&function](const DiagnosticStatus & diag) {
      return diag.name.find(function) != std::string::npos;
    });
  if (it != input_diagnostics.status.end()) {
    removeDiagnosticsByName(it->name, diag_queue);
    addDiagnostic(*it, input_diagnostics.header.stamp, diag_queue);
  }

  removeOldDiagnostics(stamp, diag_queue);
}
}  // namespace autoware::evaluator_utils
