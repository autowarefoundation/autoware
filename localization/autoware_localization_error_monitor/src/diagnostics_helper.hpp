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

#ifndef DIAGNOSTICS_HELPER_HPP_
#define DIAGNOSTICS_HELPER_HPP_

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <string>
#include <vector>

namespace autoware::localization_error_monitor
{
inline diagnostic_msgs::msg::DiagnosticStatus check_localization_accuracy(
  const double ellipse_size, const double warn_ellipse_size, const double error_ellipse_size)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

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

inline diagnostic_msgs::msg::DiagnosticStatus check_localization_accuracy_lateral_direction(
  const double ellipse_size, const double warn_ellipse_size, const double error_ellipse_size)
{
  diagnostic_msgs::msg::DiagnosticStatus stat;

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

}  // namespace autoware::localization_error_monitor

#endif  // DIAGNOSTICS_HELPER_HPP_
