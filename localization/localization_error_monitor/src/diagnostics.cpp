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

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <string>
#include <vector>

diagnostic_msgs::msg::DiagnosticStatus checkLocalizationAccuracy(
  const double ellipse_size, const double warn_ellipse_size, const double error_ellipse_size)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "localization_accuracy";
  key_value.value = std::to_string(ellipse_size);
  stat.values.push_back(key_value);

  stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat.message = "OK";
  if (ellipse_size >= warn_ellipse_size) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.message = "ellipse size is too large";
  }
  if (ellipse_size >= error_ellipse_size) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    stat.message = "ellipse size is over the expected range";
  }

  return stat;
}

diagnostic_msgs::msg::DiagnosticStatus checkLocalizationAccuracyLateralDirection(
  const double ellipse_size, const double warn_ellipse_size, const double error_ellipse_size)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "localization_accuracy_lateral_direction";
  key_value.value = std::to_string(ellipse_size);
  stat.values.push_back(key_value);

  stat.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stat.message = "OK";
  if (ellipse_size >= warn_ellipse_size) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.message = "ellipse size along lateral direction is too large";
  }
  if (ellipse_size >= error_ellipse_size) {
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    stat.message = "ellipse size along lateral direction is over the expected range";
  }

  return stat;
}

// The highest level within the stat_array will be reflected in the merged_stat.
diagnostic_msgs::msg::DiagnosticStatus mergeDiagnosticStatus(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & stat_array)
{
  diagnostic_msgs::msg::DiagnosticStatus merged_stat;

  for (const auto & stat : stat_array) {
    if ((stat.level > diagnostic_msgs::msg::DiagnosticStatus::OK)) {
      if (!merged_stat.message.empty()) {
        merged_stat.message += "; ";
      }
      merged_stat.message += stat.message;
    }
    if (stat.level > merged_stat.level) {
      merged_stat.level = stat.level;
    }
    for (const auto & value : stat.values) {
      merged_stat.values.push_back(value);
    }
  }

  if (merged_stat.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    merged_stat.message = "OK";
  }

  return merged_stat;
}
