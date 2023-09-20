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

#ifndef EKF_LOCALIZER__DIAGNOSTICS_HPP_
#define EKF_LOCALIZER__DIAGNOSTICS_HPP_

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <string>
#include <vector>

diagnostic_msgs::msg::DiagnosticStatus checkProcessActivated(const bool is_activated);

diagnostic_msgs::msg::DiagnosticStatus checkMeasurementUpdated(
  const std::string & measurement_type, const size_t no_update_count,
  const size_t no_update_count_threshold_warn, const size_t no_update_count_threshold_error);
diagnostic_msgs::msg::DiagnosticStatus checkMeasurementQueueSize(
  const std::string & measurement_type, const size_t queue_size);
diagnostic_msgs::msg::DiagnosticStatus checkMeasurementDelayGate(
  const std::string & measurement_type, const bool is_passed_delay_gate, const double delay_time,
  const double delay_time_threshold);
diagnostic_msgs::msg::DiagnosticStatus checkMeasurementMahalanobisGate(
  const std::string & measurement_type, const bool is_passed_mahalanobis_gate,
  const double mahalanobis_distance, const double mahalanobis_distance_threshold);

diagnostic_msgs::msg::DiagnosticStatus mergeDiagnosticStatus(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & stat_array);

#endif  // EKF_LOCALIZER__DIAGNOSTICS_HPP_
