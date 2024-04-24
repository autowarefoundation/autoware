// Copyright 2024 TIER IV, Inc.
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

#ifndef PERCEPTION_ONLINE_EVALUATOR__PARAMETERS_HPP_
#define PERCEPTION_ONLINE_EVALUATOR__PARAMETERS_HPP_

#include "perception_online_evaluator/metrics/metric.hpp"

#include <unordered_map>
#include <vector>

namespace perception_diagnostics
{
/**
 * @brief Enumeration of perception metrics
 */

struct ObjectParameter
{
  bool check_lateral_deviation{false};
  bool check_yaw_deviation{false};
  bool check_predicted_path_deviation{false};
  bool check_yaw_rate{false};
  bool check_total_objects_count{false};
  bool check_average_objects_count{false};
  bool check_interval_average_objects_count{false};
};

struct DebugMarkerParameter
{
  bool show_history_path{false};
  bool show_history_path_arrows{false};
  bool show_smoothed_history_path{true};
  bool show_smoothed_history_path_arrows{false};
  bool show_predicted_path{true};
  bool show_predicted_path_gt{true};
  bool show_deviation_lines{true};
  bool show_object_polygon{true};
};

struct Parameters
{
  std::vector<Metric> metrics;
  size_t smoothing_window_size{0};
  std::vector<double> prediction_time_horizons;
  double stopped_velocity_threshold{0.0};
  std::vector<double> detection_radius_list;
  std::vector<double> detection_height_list;
  double detection_count_purge_seconds;
  double objects_count_window_seconds;
  DebugMarkerParameter debug_marker_parameters;
  // parameters depend on object class
  std::unordered_map<uint8_t, ObjectParameter> object_parameters;
};

}  // namespace perception_diagnostics

#endif  // PERCEPTION_ONLINE_EVALUATOR__PARAMETERS_HPP_
