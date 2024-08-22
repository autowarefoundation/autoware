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

#ifndef DIAGNOSTICS_HPP_
#define DIAGNOSTICS_HPP_

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <string>
#include <vector>

namespace autoware::localization_error_monitor
{
diagnostic_msgs::msg::DiagnosticStatus check_localization_accuracy(
  const double ellipse_size, const double warn_ellipse_size, const double error_ellipse_size);
diagnostic_msgs::msg::DiagnosticStatus check_localization_accuracy_lateral_direction(
  const double ellipse_size, const double warn_ellipse_size, const double error_ellipse_size);

diagnostic_msgs::msg::DiagnosticStatus merge_diagnostic_status(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & stat_array);
}  // namespace autoware::localization_error_monitor

#endif  // DIAGNOSTICS_HPP_
