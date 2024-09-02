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

namespace autoware::ekf_localizer
{

diagnostic_msgs::msg::DiagnosticStatus check_process_activated(const bool is_activated);

diagnostic_msgs::msg::DiagnosticStatus check_measurement_updated(
  const std::string & measurement_type, const size_t no_update_count,
  const size_t no_update_count_threshold_warn, const size_t no_update_count_threshold_error);
diagnostic_msgs::msg::DiagnosticStatus check_measurement_queue_size(
  const std::string & measurement_type, const size_t queue_size);
diagnostic_msgs::msg::DiagnosticStatus check_measurement_delay_gate(
  const std::string & measurement_type, const bool is_passed_delay_gate, const double delay_time,
  const double delay_time_threshold);
diagnostic_msgs::msg::DiagnosticStatus check_measurement_mahalanobis_gate(
  const std::string & measurement_type, const bool is_passed_mahalanobis_gate,
  const double mahalanobis_distance, const double mahalanobis_distance_threshold);
diagnostic_msgs::msg::DiagnosticStatus check_covariance_ellipse(
  const std::string & name, const double curr_size, const double warn_threshold,
  const double error_threshold);

diagnostic_msgs::msg::DiagnosticStatus merge_diagnostic_status(
  const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & stat_array);

}  // namespace autoware::ekf_localizer

#endif  // EKF_LOCALIZER__DIAGNOSTICS_HPP_
