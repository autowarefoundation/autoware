// Copyright 2021 Tier IV, Inc.
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

#ifndef LOCALIZATION_EVALUATOR__METRICS__METRIC_HPP_
#define LOCALIZATION_EVALUATOR__METRICS__METRIC_HPP_

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace localization_diagnostics
{
/**
 * @brief Enumeration of localization metrics
 */
enum class Metric {
  lateral_error,
  absolute_error,
  SIZE,
};

static const std::unordered_map<std::string, Metric> str_to_metric = {
  {"lateral_error", Metric::lateral_error}, {"absolute_error", Metric::absolute_error}};
static const std::unordered_map<Metric, std::string> metric_to_str = {
  {Metric::lateral_error, "lateral_error"}, {Metric::absolute_error, "absolute_error"}};

// Metrics descriptions
static const std::unordered_map<Metric, std::string> metric_descriptions = {
  {Metric::lateral_error, "lateral error [m]"}, {Metric::absolute_error, "absolute error [m]"}};

namespace details
{
static struct CheckCorrectMaps
{
  CheckCorrectMaps()
  {
    if (
      str_to_metric.size() != static_cast<size_t>(Metric::SIZE) ||
      metric_to_str.size() != static_cast<size_t>(Metric::SIZE) ||
      metric_descriptions.size() != static_cast<size_t>(Metric::SIZE)) {
      std::cerr << "[metrics/metrics.hpp] Maps are not defined for all metrics: ";
      std::cerr << str_to_metric.size() << " " << metric_to_str.size() << " "
                << metric_descriptions.size() << std::endl;
    }
  }
} check;

}  // namespace details
}  // namespace localization_diagnostics

#endif  // LOCALIZATION_EVALUATOR__METRICS__METRIC_HPP_
